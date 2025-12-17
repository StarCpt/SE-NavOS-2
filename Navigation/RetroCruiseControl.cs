using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage;
using VRageMath;

namespace IngameScript
{
    public class RetroCruiseControl : OrientControllerBase, ICruiseController
    {
        public enum RetroCruiseStage : byte
        {
            None = 0,
            CancelPerpendicularVelocity = 1,
            OrientAndAccelerate = 2,
            OrientAndDecelerate = 3,
            DecelerateNoOrient = 4,
            //FinalDecelAndStop = 5,
            Complete = 6,
            Aborted = 7,
        }

        const double TICK = 1.0 / 60.0;
        const double DegToRadMulti = Math.PI / 180.0;
        const double RadToDegMulti = 180.0 / Math.PI;
        const float DAMPENER_TOLERANCE = 0.01f;

        public event CruiseTerminateEventDelegate CruiseTerminated = delegate { };

        public string Name => nameof(RetroCruiseControl);
        public RetroCruiseStage Stage
        {
            get { return _stage; }
            private set
            {
                if (_stage != value)
                {
                    var old = _stage;
                    _stage = value;
                    OnStageChanged();
                    Program.Log($"{old} to {value}");
                }
            }
        }
        public Vector3D Target { get; }
        public double DesiredSpeed { get; }
        public float MaxThrustRatio
        {
            get { return thrustController.MaxThrustRatio; }
            set
            {
                if (thrustController.MaxThrustRatio != value)
                {
                    thrustController.MaxThrustRatio = value;
                    UpdateForwardThrustAndAccel();
                }
            }
        }

        /// <summary>
        /// what speed end cruise routine during deceleration
        /// </summary>
        public double completionShipSpeed = 0.05;

        /// <summary>
        /// timeToStop + value to start rotating the ship for deceleration
        /// </summary>
        public double decelStartMarginSeconds = 10;

        /// <summary>
        /// aim/orient tolerance in radians
        /// </summary>
        public double OrientToleranceAngleRadians { get; set; } = 0.100 * DegToRadMulti;

        public double maxInitialPerpendicularVelocity = 1;

        //useful for overestimating stop time and dist for better cruise accuracy
        public double stopTimeAndDistanceMulti = 1.05;

        private VariableThrustController thrustController;
        private Program program;
        private Config config;

        //active variables
        private RetroCruiseStage _stage;
        private int counter = -1;
        private bool counter10 = false;
        private bool counter30 = false;

        //updated every 30 ticks
        private float gridMass;
        private float forwardAccel;
        private float forwardAccelPremultiplied; //premultiplied by maxThrustOverrideRatio
        private float forwardThrustInv;

        //updated every 10 ticks
        //how far off the aim is from the desired direction
        private double? lastAimDirectionAngleRad = null;
        private double estimatedTimeOfArrival;

        //updated every tick
        private double accelTime, timeToStartDecel, cruiseTime, currentStopDist, actualStopTime, distanceToTarget, vmax, mySpeed, lastMySpeed;
        private float lastThrustRatio;
        private Vector3D myVelocity, targetDirection, normalizedTargetDirection, gravityAtPos;
        private bool noSpeedOnStart;
        private RetroCruiseStage initialStage = RetroCruiseStage.None;
        private bool savePersistentData;

        public RetroCruiseControl(
            Vector3D target,
            double desiredSpeed,
            IAimController aimControl,
            IMyShipController controller,
            IList<IMyGyro> gyros,
            VariableThrustController thrustController,
            Program program,
            bool savePersistentData = true)
            : base(aimControl, controller, gyros)
        {
            this.Target = target;
            this.DesiredSpeed = desiredSpeed;
            this.thrustController = thrustController;
            this.program = program;
            this.config = program.config;
            this.savePersistentData = savePersistentData;

            Stage = RetroCruiseStage.None;
            gridMass = controller.CalculateShipMass().PhysicalMass;

            UpdateForwardThrustAndAccel();
        }

        public RetroCruiseControl(
            Vector3D target,
            double desiredSpeed,
            IAimController aimControl,
            IMyShipController controller,
            IList<IMyGyro> gyros,
            VariableThrustController thrustController,
            Program program,
            RetroCruiseStage stage,
            bool savePersistentData = true)
            : this(
                  target,
                  desiredSpeed,
                  aimControl,
                  controller,
                  gyros,
                  thrustController,
                  program)
        {
            initialStage = stage;
            this.savePersistentData = savePersistentData;
        }

        public void AppendStatus(StringBuilder strb)
        {
            strb.Append("\n-- Cruise Status --\n\n");

            if (timeToStartDecel < 0 || Vector3D.Dot(myVelocity, targetDirection) < 0)
            {
                strb.Append($"!! Overshoot Warning !! ({GetShortDistance(currentStopDist - distanceToTarget)})\n\n");
            }

            const string stage1 = "> Cancel Perpendicular Speed\n";

            switch (Stage)
            {
                case RetroCruiseStage.CancelPerpendicularVelocity:
                case RetroCruiseStage.OrientAndAccelerate:
                    strb.Append((byte)Stage == 1 ? stage1 : $">{stage1}>")
                        .Append(" Accelerate ").AppendTime(accelTime)
                        .Append("\nCruise ").AppendTime(cruiseTime)
                        .Append("\nDecelerate ").AppendTime(actualStopTime)
                        .Append("\nStop");
                    break;
                case RetroCruiseStage.OrientAndDecelerate:
                    strb.Append($">{stage1}>> Accelerate 0:00\n");
                    if (!decelerating)
                        strb.Append("> Cruise ").AppendTime(cruiseTime).AppendLine();
                    else
                        strb.Append(">> Cruise ").Append(timeToStartDecel.ToString("0:00.000")).Append("\n> ");
                    strb.Append("Decelerate ").AppendTime(actualStopTime).Append("\nStop");
                    break;
                case RetroCruiseStage.DecelerateNoOrient:
                    strb.Append($">{stage1}>> Accelerate 0:00\n>> Cruise 0:00\n>> Decelerate 0:00\n> Stop");
                    break;
            }

            strb.Append("\n\nETA: ").AppendTime(estimatedTimeOfArrival)
            .Append("\nEst. Stop Dist.: " + GetShortDistance(currentStopDist))
            .Append("\nDestination Dist.: " + GetShortDistance(distanceToTarget))
            .Append("\nDesired Speed: " + DesiredSpeed.ToString("0.## m/s"))
            .Append("\nAim Error: " + (lastAimDirectionAngleRad * RadToDegMulti ?? 0).ToString("0.000\n"));
        }

        public static string GetShortDistance(double meters)
        {
            if (meters >= 1000)
                return (meters / 1000).ToString("0.## km");
            else
                return meters.ToString("0 m");
        }

        private void DampenAllDirections(Vector3D shipVelocity, float tolerance = DAMPENER_TOLERANCE)
        {
            Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
            Vector3 thrustAmount = localVelocity * gridMass;
            thrustController.SetThrusts(thrustAmount, tolerance);
        }

        /// <param name="shipVelocity">Current world space velocity</param>
        /// <param name="ups">Number of times this method is run per second</param>
        private void DampenSidewaysToZero(Vector3D shipVelocity, float ups = 1)
        {
            Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
            Vector3 thrustAmount = localVelocity * gridMass * ups;
            float right = thrustAmount.X < 0 ? -thrustAmount.X : 0;
            float left  = thrustAmount.X > 0 ?  thrustAmount.X : 0;
            float up    = thrustAmount.Y < 0 ? -thrustAmount.Y : 0;
            float down  = thrustAmount.Y > 0 ?  thrustAmount.Y : 0;
            thrustController.SetSideThrusts(left, right, up, down);
        }

        public void Run()
        {
            counter++;
            counter10 = counter % 10 == 0;
            counter30 = counter % 30 == 0;
            bool counter60 = counter % 60 == 0;

            if (Stage == RetroCruiseStage.None)
            {
                ResetGyroOverride();
                thrustController.ResetThrustOverrides();
                TurnOnAllThrusters();
                thrustController.UpdateThrusts();
            }

            if (counter10)
            {
                lastAimDirectionAngleRad = null;

                SetDampenerState(false);
            }
            if (counter30)
            {
                UpdateForwardThrustAndAccel();
                gravityAtPos = ShipController.GetNaturalGravity();
            }
            if (counter60)
            {
                gridMass = ShipController.CalculateShipMass().PhysicalMass;
                thrustController.UpdateThrusts();
            }

            Vector3D myPosition = ShipController.GetPosition();
            myVelocity = ShipController.GetShipVelocities().LinearVelocity + gravityAtPos;
            lastMySpeed = mySpeed;
            mySpeed = myVelocity.Length();

            targetDirection = Target - myPosition;//aka relativePosition
            distanceToTarget = targetDirection.Length();
            normalizedTargetDirection = targetDirection / distanceToTarget;

            //time to stop: currentSpeed / acceleration;
            //stopping distance: timeToStop * (currentSpeed / 2)
            //or also: currentSpeed^2 / (2 * acceleration)
            //stopTime = mySpeed / forwardAccelPremultiplied * stopTimeAndDistanceMulti;
            //stopDist = stopTime * (mySpeed * 0.5);
            currentStopDist = (mySpeed * mySpeed) / (2 * forwardAccelPremultiplied) * stopTimeAndDistanceMulti;

            timeToStartDecel = ((distanceToTarget - currentStopDist) / mySpeed) - TICK * 10;

            if (Stage == RetroCruiseStage.None)
            {
                if (initialStage != RetroCruiseStage.None)
                {
                    Stage = initialStage;
                }
                else
                {
                    noSpeedOnStart = mySpeed <= maxInitialPerpendicularVelocity;

                    Vector3D perpVel = Vector3D.ProjectOnPlane(ref myVelocity, ref targetDirection);
                    if (perpVel.LengthSquared() > maxInitialPerpendicularVelocity * maxInitialPerpendicularVelocity)
                        Stage = RetroCruiseStage.CancelPerpendicularVelocity;
                    else
                        Stage = RetroCruiseStage.OrientAndAccelerate;
                }
            }

            Repeat:

            if (Stage == RetroCruiseStage.CancelPerpendicularVelocity)
            {
                CancelPerpendicularVelocity();
            }

            if (Stage == RetroCruiseStage.OrientAndAccelerate)
            {
                OrientAndAccelerate(myVelocity, mySpeed);
            }

            if (Stage == RetroCruiseStage.OrientAndDecelerate)
            {
                if (counter10 && timeToStartDecel * 0.25 > decelStartMarginSeconds)
                {
                    Stage = RetroCruiseStage.OrientAndAccelerate;
                    goto Repeat;
                }

                OrientAndDecelerate(mySpeed);
            }

            if (Stage == RetroCruiseStage.DecelerateNoOrient)
            {
                DecelerateNoOrient(mySpeed);
            }

            if (Stage == RetroCruiseStage.Complete)
            {
                estimatedTimeOfArrival = 0;
                Complete();
            }

            if (counter10)
            {

                if (Stage <= RetroCruiseStage.OrientAndAccelerate)
                {
                    double currentAndDesiredSpeedDelta = Math.Abs(DesiredSpeed - mySpeed);

                    accelTime = (currentAndDesiredSpeedDelta / forwardAccelPremultiplied);
                    double accelDist = accelTime * ((mySpeed + DesiredSpeed) * 0.5);

                    actualStopTime = DesiredSpeed / forwardAccelPremultiplied * stopTimeAndDistanceMulti;
                    double actualStopDist = actualStopTime * (DesiredSpeed * 0.5);

                    double cruiseDist = distanceToTarget - actualStopDist - accelDist;
                    cruiseTime = cruiseDist / DesiredSpeed;

                    if (cruiseTime < decelStartMarginSeconds)
                    {
                        //https://math.stackexchange.com/questions/637042/calculate-maximum-velocity-given-accel-decel-initial-v-final-position

                        //v0 = initial (current) speed
                        //vmax = max speed;
                        //v2 = final speed (zero)
                        //a = accel
                        //d = deceleration (NOT DISTANCE!!!!)
                        //t1 = time at max achievable speed
                        //t2 = decel time
                        //x = starting (current) position (aka. x == 0)
                        //l = end position (distance)

                        double v0 = mySpeed;
                        double a = forwardAccelPremultiplied;
                        double d = forwardAccelPremultiplied * (2 - stopTimeAndDistanceMulti);
                        double l = distanceToTarget;

                        //v0 + (a * t1) - (d * t2) == 0
                        //rearranged: t2 == (v0 + (a * t1)) / d

                        //(v0 * t1) + (00.5 * a * t1^2) + ((v0 * t2) + (a * t1 * t2) - (0.5 * d * t2^2)) == l;

                        //t1 == -(v0 / a) + (1 / a) * sqrt(((d * v0^2) + (2 * a * l * d)) / (a + d))
                        //vmax == v0 + (a * t1) == sqrt(((d * v0^2) + (2 * a * l * d)) / (a + d))

                        vmax = Math.Sqrt((d * v0 * v0 + 2 * a * l * d) / (a + d));
                        accelTime = (vmax - v0) / a;
                        actualStopTime = vmax / d;
                        estimatedTimeOfArrival = accelTime + actualStopTime;
                        cruiseTime = 0;
                    }
                    else
                    {
                        vmax = 0;
                        estimatedTimeOfArrival = accelTime + cruiseTime + actualStopTime;
                    }
                }
                else
                {
                    accelTime = 0;
                    actualStopTime = mySpeed / forwardAccelPremultiplied * stopTimeAndDistanceMulti; ;

                    double cruiseDist = distanceToTarget - currentStopDist;
                    cruiseTime = cruiseDist / mySpeed;

                    estimatedTimeOfArrival = cruiseTime + actualStopTime;
                }
            }
        }

        private void UpdateForwardThrustAndAccel()
        {
            float forwardThrust = thrustController.Thrusters[Direction.Forward].Where(t => t.IsWorking).Sum(t => t.MaxEffectiveThrust);
            forwardThrustInv = 1f / forwardThrust;
            forwardAccel = forwardThrust / gridMass;
            forwardAccelPremultiplied = forwardAccel * MaxThrustRatio;
        }

        private void ResetThrustOverridesExceptFront()
        {
            ResetBackThrusts();
            thrustController.SetSideThrusts(0, 0, 0, 0);
        }

        private void ResetBackThrusts()
        {
            var backThrusts = thrustController.Thrusters[Direction.Backward];
            for (int i = backThrusts.Count - 1; i >= 0; i--)
                backThrusts[i].ThrustOverride = 0;
        }

        public void TurnOnAllThrusters()
        {
            foreach (var kv in thrustController.Thrusters)
                for (int i = 0; i < kv.Value.Count; i++)
                    kv.Value[i].Enabled = true;
        }

        private void SetDampenerState(bool enabled) => ShipController.DampenersOverride = enabled;

        private void OnStageChanged()
        {
            thrustController.ResetThrustOverrides();
            ResetGyroOverride();
            SetDampenerState(false);
            lastAimDirectionAngleRad = null;
            decelerating = false;
            if (savePersistentData)
            {
                config.PersistStateData = $"{NavModeEnum.Cruise}|{DesiredSpeed}|{Stage}";
                program.Me.CustomData = config.ToString();
            }
        }

        private void CancelPerpendicularVelocity()
        {
            Vector3D aimDirection = -Vector3D.ProjectOnPlane(ref myVelocity, ref targetDirection);
            double perpSpeed = aimDirection.Length();

            if (perpSpeed <= maxInitialPerpendicularVelocity)
            {
                Stage = RetroCruiseStage.OrientAndAccelerate;
                return;
            }

            Orient(aimDirection);

            if (!counter10)
            {
                return;
            }

            if (!lastAimDirectionAngleRad.HasValue)
            {
                lastAimDirectionAngleRad = AngleRadiansBetweenVectorAndControllerForward(aimDirection);
            }

            if (lastAimDirectionAngleRad.Value <= OrientToleranceAngleRadians)
            {
                float overrideAmount = MathHelper.Clamp(((float)perpSpeed * 5 * gridMass) * forwardThrustInv, 0, MaxThrustRatio);
                foreach (var thruster in thrustController.Thrusters[Direction.Forward])
                {
                    thruster.ThrustOverridePercentage = overrideAmount;
                }
            }
            else
            {
                foreach (var thruster in thrustController.Thrusters[Direction.Forward])
                {
                    thruster.ThrustOverride = 0;
                }
            }

            if (counter10)
            {
                ResetThrustOverridesExceptFront();
            }
        }

        private void OrientAndAccelerate(Vector3D velocity, double velocityLength)
        {
            bool approaching = Vector3D.Dot(normalizedTargetDirection, velocity) > 0;
            if (!noSpeedOnStart && approaching && !double.IsNegativeInfinity(timeToStartDecel) && timeToStartDecel <= decelStartMarginSeconds && mySpeed > maxInitialPerpendicularVelocity)
            {
                Stage = RetroCruiseStage.OrientAndDecelerate;
                return;
            }

            Vector3D aimDirection = normalizedTargetDirection;

            Orient(aimDirection);

            if (!counter10)
            {
                return;
            }

            // ups for code below
            const float UPS = 6;

            if (!lastAimDirectionAngleRad.HasValue)
            {
                lastAimDirectionAngleRad = AngleRadiansBetweenVectorAndControllerForward(aimDirection);
            }

            if (lastAimDirectionAngleRad.Value <= OrientToleranceAngleRadians)
            {
                noSpeedOnStart = false;

                // speed directly towards the target
                // >0 if closing, <0 ottherwise
                double forwardSpeed = velocityLength > 0 ? (velocityLength * Vector3D.Dot(velocity / velocityLength, normalizedTargetDirection)) : 0;

                bool desiredSpeedReached = forwardSpeed >= DesiredSpeed;
                float thrustRatio;
                if (forwardSpeed > 0 && config.MaintainDesiredSpeed)
                {
                    // TODO: lastMySpeed is from 1 tick ago, but it should be from (60 / UPS) ticks ago
                    // FIX!!!!!
                    // lastThrustRatio is set in this method so it's fine... for now
                    double actualAccel = (forwardSpeed - lastMySpeed) * UPS;
                    double expectedAccel = forwardAccel * lastThrustRatio;

                    double speedDelta = DesiredSpeed - forwardSpeed;
                    double desiredAccel = speedDelta + (expectedAccel - actualAccel);
                    double desiredThrustRatio = desiredAccel / forwardAccel * UPS;
                    thrustRatio = MathHelper.Min(MaxThrustRatio, (float)desiredThrustRatio);
                }
                else if (desiredSpeedReached)
                {
                    Stage = RetroCruiseStage.OrientAndDecelerate;
                    return;
                }
                else
                {
                    double speedDelta = DesiredSpeed - forwardSpeed;
                    double desiredThrustRatio = speedDelta / forwardAccel * UPS;
                    thrustRatio = Math.Min(MaxThrustRatio, (float)desiredThrustRatio);
                }

                var forwardThrusters = thrustController.Thrusters[Direction.Forward];
                for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                {
                    forwardThrusters[i].ThrustOverridePercentage = thrustRatio;
                }

                lastThrustRatio = thrustRatio;

                if (counter30)
                {
                    ResetBackThrusts(); // why is this here?
                }

                Vector3D velocityPerpendicularToTarget = Vector3D.ProjectOnPlane(ref velocity, ref targetDirection);
                DampenSidewaysToZero(velocityPerpendicularToTarget, UPS);

                return;
            }

            thrustController.ResetThrustOverrides();
        }

        private bool decelerating = false;

        private void OrientAndDecelerate(double mySpeed)
        {
            if (mySpeed <= completionShipSpeed)
            {
                Stage = RetroCruiseStage.Complete;
                return;
            }

            bool approaching = Vector3D.Dot(targetDirection, myVelocity) > 0;

            if (!approaching)
            {
                decelNoOrientAimDir = -myVelocity;
                Stage = RetroCruiseStage.DecelerateNoOrient;
                return;
            }

            Vector3D orientForward = -(targetDirection + myVelocity);

            Orient(orientForward);

            if (!counter10)
            {
                return;
            }

            if (!lastAimDirectionAngleRad.HasValue)
            {
                lastAimDirectionAngleRad = AngleRadiansBetweenVectorAndControllerForward(orientForward);
            }

            if (lastAimDirectionAngleRad.Value <= OrientToleranceAngleRadians)
            {
                if (distanceToTarget < forwardAccelPremultiplied)
                {
                    decelNoOrientAimDir = -myVelocity;
                    Stage = RetroCruiseStage.DecelerateNoOrient;
                    return;
                }
            
                float overrideAmount = MathHelper.Clamp((float)-timeToStartDecel + MaxThrustRatio, 0f, MaxThrustRatio);
            
                decelerating = overrideAmount > 0;
            
                var foreThrusts = thrustController.Thrusters[Direction.Forward];
                for (int i = 0; i < foreThrusts.Count; i++)
                    foreThrusts[i].ThrustOverridePercentage = overrideAmount;

                //DampenSidewaysToZero(myVelocity * 5);
                Vector3D perp = -Vector3D.ProjectOnPlane(ref myVelocity, ref targetDirection);
                DampenSidewaysToZero(-perp * 5);

                if (counter30)
                {
                    ResetBackThrusts();
                }
            
                return;
            }
            
            if (timeToStartDecel > 0)
            {
                thrustController.ResetThrustOverrides();
                return;
            }
            
            DampenAllDirections(myVelocity * 5);
        }

        private Vector3D decelNoOrientAimDir;

        private void DecelerateNoOrient(double mySpeed)
        {
            if (mySpeed <= completionShipSpeed)
            {
                Stage = RetroCruiseStage.Complete;
                return;
            }

            Orient(decelNoOrientAimDir);

            if (!lastAimDirectionAngleRad.HasValue)
            {
                lastAimDirectionAngleRad = AngleRadiansBetweenVectorAndControllerForward(decelNoOrientAimDir);
            }

            bool approaching = Vector3D.Dot(targetDirection, myVelocity) > 0;
            
            if (!approaching)
            {
                DampenAllDirections(myVelocity * 50, 0);
                return;
            }

            //float overrideAmount = Math.Min(((float)-timeToStartDecel + MaxThrustRatio), MaxThrustRatio);
            //
            //var foreThrusts = thrustController.Thrusters[Direction.Forward];
            //for (int i = 0; i < foreThrusts.Count; i++)
            //    foreThrusts[i].ThrustOverridePercentage = overrideAmount;
            //
            //DampenSidewaysToZero(-(targetDirection - myVelocity - myVelocity));

            DampenAllDirections(-(targetDirection - myVelocity - myVelocity));
        }

        private double AngleRadiansBetweenVectorAndControllerForward(Vector3D vec)
        {
            Vector3D.Normalize(ref vec, out vec);
            double cos = Vector3D.Dot(ShipController.WorldMatrix.Forward, vec);
            double angle = Math.Acos(cos);
            return double.IsNaN(angle) ? 0 : angle;
        }

        private void Complete()
        {
            SetDampenerState(true);
            Terminate(distanceToTarget < 10 ? "Destination Reached" : "Terminated");
        }

        public void Terminate(string reason)
        {
            thrustController.ResetThrustOverrides();
            TurnOnAllThrusters();

            ResetGyroOverride();

            CruiseTerminated.Invoke(this, reason);
        }

        public void Abort()
        {
            Stage = RetroCruiseStage.Aborted;
            Terminate("Aborted");
        }

        protected override void OnNoFunctionalGyrosLeft() => Terminate("No functional gyros found");
    }
}
