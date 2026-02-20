using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    public class Autopilot : ICruiseController
    {
        private const double TARGET_REACHED_DIST = 0.05;
        private const double TARGET_REACHED_SPEED = 0.01;

        public event CruiseTerminateEventDelegate CruiseTerminated;

        public string Name => nameof(Autopilot);
        public Vector3D? Target
        {
            get { return _target; }
            set { _target = value; }
        }
        public float MaxSpeed
        {
            get { return _maxSpeed; }
            set { _maxSpeed = value; }
        }

        private Vector3D? _target;
        private float _maxSpeed;
        private float _shipMass;
        private Vector3D _gravity;

        private readonly IMyShipController _shipController;
        private readonly VariableThrustController _thrustController;

        public Autopilot(IMyShipController shipController, VariableThrustController thrustController)
        {
            _shipController = shipController;
            _thrustController = thrustController;
        }

        public void AppendStatus(StringBuilder strb)
        {
            strb.Append("\n-- Autopilot Status --\n");

            double targetDist = Vector3D.Distance(_target ?? Vector3D.Zero, _shipController.WorldAABB.Center);
            strb.Append($"Max Speed: {_maxSpeed:0.#} m/s\n");
            strb.Append($"Distance: {targetDist:0.0}m\n");
        }

        int counter = -1;
        public void Run()
        {
            counter++;

            if (!_target.HasValue)
            {
                Terminate("Target is null");
                return;
            }

            if (counter % 10 == 0)
            {
                _thrustController.UpdateThrusts();

                // update mass
                _shipMass = _shipController.CalculateShipMass().PhysicalMass;

                // update gravity
                _gravity = _shipController.GetNaturalGravity();

                // turn off dampeners
                _thrustController.SetDampenerState(false);
            }

            const int UPS = 6; // must cleanly divide 60

            if (counter % (60 / UPS) != 0)
            {
                return;
            }

            if (RunStateless(_thrustController, _target.Value, _maxSpeed, UPS, _shipMass, _gravity))
            {
                _thrustController.SetDampenerState(true);
                Terminate("Target reached");
                return;
            }

            //Program.optionalInfo = optionalInfo2.ToString();
            //optionalInfo2.Clear();
        }

        public static bool RunStateless(VariableThrustController thrustController, Vector3D target, float maxSpeed, float ups, float shipMass, Vector3D naturalGravity)
        {
            IMyShipController shipController = thrustController.ShipController;

            thrustController.UpdateThrusts();
            thrustController.SetDampenerState(false);

            Vector3D currentVelocity = shipController.GetShipVelocities().LinearVelocity;
            Vector3D displacement = target - shipController.WorldAABB.Center;

            bool targetReachedDist = displacement.LengthSquared() <= (TARGET_REACHED_DIST * TARGET_REACHED_DIST);
            bool targetReachedSpeed = currentVelocity.LengthSquared() <= (TARGET_REACHED_SPEED * TARGET_REACHED_SPEED);
            if (targetReachedDist && targetReachedSpeed)
            {
                return true;
            }
            else if (targetReachedDist)
            {
                // come to a stop asap
                thrustController.DampenAllDirections(currentVelocity, naturalGravity, shipMass, ups);
                return false;
            }

            MatrixD transposedShipControllerWorldMatrix = MatrixD.Transpose(shipController.WorldMatrix);
            Vector3D localVelocity;
            Vector3D localGravity;
            Vector3D localDisplacement; // local relative pos, we're at 0,0,0
            Vector3D.TransformNormal(ref currentVelocity, ref transposedShipControllerWorldMatrix, out localVelocity);
            Vector3D.TransformNormal(ref naturalGravity, ref transposedShipControllerWorldMatrix, out localGravity);
            Vector3D.TransformNormal(ref displacement, ref transposedShipControllerWorldMatrix, out localDisplacement);

            Vector3D maxLocalClosingVelocity = Vector3D.Abs(Vector3D.Normalize(localDisplacement)) * maxSpeed;
            float maxForwardThrustRatio = thrustController.MaxForwardThrustRatio;

            Dictionary<Direction, List<IMyThrust>> thrusters = thrustController.Thrusters;
            MovePerAxis(localVelocity.X, localDisplacement.X, shipMass, thrusters[Direction.Right],    thrusters[Direction.Left],    thrustController.GetThrustInDirection(Direction.Right),    thrustController.GetThrustInDirection(Direction.Left),    localGravity.X, maxLocalClosingVelocity.X, ups, 1, 1);
            MovePerAxis(localVelocity.Y, localDisplacement.Y, shipMass, thrusters[Direction.Up],       thrusters[Direction.Down],    thrustController.GetThrustInDirection(Direction.Up),       thrustController.GetThrustInDirection(Direction.Down),    localGravity.Y, maxLocalClosingVelocity.Y, ups, 1, 1);
            MovePerAxis(localVelocity.Z, localDisplacement.Z, shipMass, thrusters[Direction.Backward], thrusters[Direction.Forward], thrustController.GetThrustInDirection(Direction.Backward), thrustController.GetThrustInDirection(Direction.Forward), localGravity.Z, maxLocalClosingVelocity.Z, ups, 1, maxForwardThrustRatio);

            return false;
        }

        //static StringBuilder optionalInfo2 = new StringBuilder();

        // returns: on target
        private static void MovePerAxis(
            double velocity, double displacement, double shipMass,
            List<IMyThrust> approachThrusters, List<IMyThrust> stoppingThrusters,
            double approachThrust, double stoppingThrust,
            double gravity, double maxClosingVelocity, double ups,
            float maxAccelRatio, float maxDecelRatio)
        {
            if (displacement < 0)
            {
                velocity = -velocity;
                displacement = -displacement;
                gravity = -gravity;

                var tempApproachThrusters = approachThrusters;
                approachThrusters = stoppingThrusters;
                stoppingThrusters = tempApproachThrusters;

                var tempApproachAccel = approachThrust;
                approachThrust = stoppingThrust;
                stoppingThrust = tempApproachAccel;

                float tempMaxAccelRatio = maxAccelRatio;
                maxAccelRatio = maxDecelRatio;
                maxDecelRatio = tempMaxAccelRatio;
            }

            double approachAccel = approachThrust / shipMass;
            double stoppingAccel = stoppingThrust / shipMass;

            double accelMulti;
            double decelInTimeSteps; // start decel in x seconds. debug use only
            // use pre-maxRatio multiplied accel/decel
            bool shouldAccel = ComputeMaxAxisAccel(velocity, displacement, approachAccel + gravity, stoppingAccel - gravity, ups, out accelMulti, out decelInTimeSteps);

            approachThrust *= maxAccelRatio;
            stoppingThrust *= maxDecelRatio;
            approachAccel *= maxAccelRatio;
            stoppingAccel *= maxDecelRatio;

            //optionalInfo2.AppendLine($"velocity: {velocity:0.0000}");
            //optionalInfo2.AppendLine($"displacement: {displacement:0.0000}");
            //optionalInfo2.AppendLine($"approachAccel: {approachAccel:0.0000}");
            //optionalInfo2.AppendLine($"stoppingAccel: {stoppingAccel:0.0000}");
            //optionalInfo2.AppendLine($"accelMulti: {accelMulti:0.0000}");
            //optionalInfo2.AppendLine($"decelInTicks: {decelInTimeSteps:0.00}");
            //optionalInfo2.AppendLine($"g: {gravity:0.0000}");

            double timeStep = 1.0 / ups;

            double totalAccelRatio = 0;
            double totalDecelRatio = 0;
            if (shouldAccel)
            {
                totalAccelRatio += accelMulti;
                //optionalInfo2.AppendLine("accel");
            }
            else if (velocity <= 0)
            {
                // kill speed
                // this branch should rarely be taken

                double accelRatio = (-velocity * shipMass / approachThrust) * ups;
                totalAccelRatio += accelRatio;
                //optionalInfo2.AppendLine("decel (forced)");
            }
            else
            {
                double desiredStopDist = displacement;
                double desiredStopAccel = (velocity * velocity) / (desiredStopDist * 2);
                double desiredStopTime = velocity / desiredStopAccel;

                // if desiredStopTime < actualStopTime, it's not good
                // it means we need to brake faster than we can which is impossible

                double decelRatio;
                double targetDecelTime = desiredStopTime - timeStep;
                if (targetDecelTime <= 0 || desiredStopDist <= 0.0001)
                {
                    decelRatio = (velocity * shipMass / stoppingThrust) * ups;
                    //optionalInfo2.AppendLine("decel (full)");
                }
                else
                {
                    double targetDecel = velocity / targetDecelTime;
                    decelRatio = targetDecel / stoppingAccel;
                    //optionalInfo2.AppendLine("decel (smooth)");
                }

                totalDecelRatio += decelRatio;
            }

            // reject nonsense values
            totalAccelRatio = MathHelper.Max(0, totalAccelRatio);
            totalDecelRatio = MathHelper.Max(0, totalDecelRatio);

            // gravity compensation
            {
                totalAccelRatio += approachAccel != 0 && gravity < 0 ? (-gravity / approachAccel) : 0;
                totalDecelRatio += stoppingAccel != 0 && gravity > 0 ? ( gravity / stoppingAccel) : 0;

                // not sure if I have to clamp again?
                //totalAccelRatio = MathHelper.Saturate(totalAccelRatio);
                //totalDecelRatio = MathHelper.Saturate(totalDecelRatio);
            }

            // speed limiter
            {
                double nextVelocity = velocity + ((totalAccelRatio * approachAccel) - (totalDecelRatio * stoppingAccel) + gravity) * timeStep;
                if (nextVelocity > maxClosingVelocity)
                {
                    double excessVelocity = nextVelocity - maxClosingVelocity;
                    totalAccelRatio -= (excessVelocity * ups) / approachAccel;
                    totalDecelRatio -= totalAccelRatio * approachAccel / stoppingAccel;
                }
            }

            // reject nonsense values (again)
            totalAccelRatio = MathHelper.Max(0, totalAccelRatio);
            totalDecelRatio = MathHelper.Max(0, totalDecelRatio);

            // cancel out thrust if both accel and decel are nonzero
            double minThrust = Math.Min(totalAccelRatio * approachThrust, totalDecelRatio * stoppingThrust);
            totalAccelRatio -= minThrust / approachThrust;
            totalDecelRatio -= minThrust / stoppingThrust;

            totalAccelRatio = MathHelper.Saturate(totalAccelRatio);
            totalDecelRatio = MathHelper.Saturate(totalDecelRatio);

            totalAccelRatio *= maxAccelRatio;
            totalDecelRatio *= maxDecelRatio;

            SetThrustRatio(approachThrusters, (float)totalAccelRatio);
            SetThrustRatio(stoppingThrusters, (float)totalDecelRatio);

            //optionalInfo2.AppendLine();
        }

        public static bool ComputeMaxAxisAccel(double velocity, double displacement, double accel, double decel, double ups, out double bestAccelMulti, out double bestDecelInTimeSteps)
        {
            // displacement is > 0

            bestDecelInTimeSteps = 0; // min valid candidate

            const double lookAhead = 2;

            // use iterative binary search to find best valid accelMulti.
            // not the best method... TODO: replace with analytic algorithm
            double lowerBound = 0; // doubles as bestAccelMulti
            double upperBound = 1;
            for (int i = 0; i < 10; i++)
            {
                double accelMulti = i == 0 ? upperBound : (lowerBound + upperBound) * 0.5;
                double decelInSeconds = ComputeTimeToDecel(velocity, displacement, accel * accelMulti, decel); // start decel after this amount of time

                bool valid = decelInSeconds * ups > lookAhead;
                if (valid)
                {
                    // lower bound is the previous best accel,
                    // so the current value is as good or better than it
                    lowerBound = accelMulti;
                    bestDecelInTimeSteps = decelInSeconds;
                }
                else
                {
                    upperBound = accelMulti;
                }
            }

            bestAccelMulti = lowerBound; // max valid thrust ratio
            bestDecelInTimeSteps *= ups;
            return bestAccelMulti != 0;
        }

        private static void SetThrustRatio(List<IMyThrust> thrusters, float percentage)
        {
            percentage = percentage < 0 ? 0 : (percentage > 1 ? 1 : percentage); // saturate
            // negative values are ok as ThrustOverridePercentage setter clamps it to [0,1]
            for (int i = thrusters.Count - 1; i >= 0; i--)
            {
                var thruster = thrusters[i];
                if (thruster.ThrustOverridePercentage != percentage)
                {
                    thrusters[i].ThrustOverridePercentage = percentage;
                }
            }
        }

        private static double ComputeTimeToDecel(double velocity, double displacement, double accel, double decel)
        {
            if (decel <= 0)
            {
                return 0;
            }

            if (accel <= 0)
            {
                double stopDist = (velocity * velocity) / (decel * 2);
                return (displacement - stopDist) / velocity;
            }

            double initialTimeToStop = 0;
            if (velocity < 0)
            {
                initialTimeToStop = -velocity / accel;
                velocity = 0;
                displacement += 0.5 * accel * initialTimeToStop * initialTimeToStop;
            }

            double vMax = Math.Sqrt((decel * (velocity * velocity + 2 * accel * displacement)) / (accel + decel));
            double timeToDecel = (vMax - velocity) / accel;
            return (timeToDecel + initialTimeToStop);
        }

        public void Abort() => Terminate("Aborted");

        public void Terminate(string reason)
        {
            _thrustController.ResetThrustOverrides();
            CruiseTerminated.Invoke(this, reason);
        }
    }
}
