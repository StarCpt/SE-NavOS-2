using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public class OneWayCruise : OrientControllerBase, ICruiseController
        {
            public enum OneWayCruiseStage : byte
            {
                None = 0,
                CancelPerpendicularVelocity = 1,
                OrientAndAccelerate = 2,
                Complete = 6,
                Aborted = 7,
            }

            const double TICK = 1.0 / 60.0;
            const double DegToRadMulti = Math.PI / 180.0;
            const double RadToDegMulti = 180.0 / Math.PI;
            const float DAMPENER_TOLERANCE = 0.01f;

            public event CruiseTerminateEventDelegate CruiseTerminated = delegate { };

            public string Name => nameof(RetroCruiseControl);
            public OneWayCruiseStage Stage
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
            /// aim/orient tolerance in radians
            /// </summary>
            public double OrientToleranceAngleRadians { get; set; } = 0.075 * DegToRadMulti;

            public double maxInitialPerpendicularVelocity = 0.5;

            private VariableThrustController thrustController;

            //active variables
            private OneWayCruiseStage _stage;
            private int counter = -1;
            private bool counter10 = false;

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
            private double accelTime, cruiseTime, distanceToTarget, vmax, mySpeed, lastMySpeed;
            private float lastThrustRatio;
            private Vector3D myVelocity, targetDirection, gravityAtPos;
            private bool noSpeedOnStart, approachingTarget;

            public OneWayCruise(
                Vector3D target,
                double desiredSpeed,
                IAimController aimControl,
                IMyShipController controller,
                IList<IMyGyro> gyros,
                VariableThrustController thrustController)
                : base(aimControl, controller, gyros)
            {
                this.Target = target;
                this.DesiredSpeed = desiredSpeed;
                this.thrustController = thrustController;

                Stage = OneWayCruiseStage.None;
                gridMass = controller.CalculateShipMass().PhysicalMass;

                UpdateForwardThrustAndAccel();
            }

            public void AppendStatus(StringBuilder strb)
            {
                strb.Append("\n-- OneWayCruise Status --\n\n");

                const string stage1 = "> Cancel Perpendicular Speed\n";

                strb.Append((byte)Stage == 1 ? stage1 : $">{stage1}>")
                .Append(" Accelerate ").AppendTime(accelTime)
                .Append("\nCruise ").AppendTime(cruiseTime)
                .Append("\n\nETA: ").AppendTime(estimatedTimeOfArrival);

                if (vmax != 0)
                    strb.Append("\nMax Speed: ").Append(vmax.ToString("0.00"));

                strb.Append("\nTargetDistance: ").Append(distanceToTarget.ToString("0.0"))
                .Append("\nDesired Speed: ").Append(DesiredSpeed.ToString("0.##"))
                .Append("\nAim Error: ").Append(((lastAimDirectionAngleRad ?? 0) * RadToDegMulti).ToString("0.000\n"));
            }

            private void DampenSideways(Vector3D shipVelocity, float tolerance = DAMPENER_TOLERANCE)
            {
                Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
                Vector3 thrustAmount = localVelocity * gridMass;
                float right = thrustAmount.X < tolerance ? -thrustAmount.X : 0;
                float left = thrustAmount.X > tolerance ? thrustAmount.X : 0;
                float up = thrustAmount.Y < tolerance ? -thrustAmount.Y : 0;
                float down = thrustAmount.Y > tolerance ? thrustAmount.Y : 0;
                thrustController.SetSideThrusts(left, right, up, down);
            }

            public void Run()
            {
                counter++;
                counter10 = counter % 10 == 0;
                bool counter30 = counter % 30 == 0;
                bool counter60 = counter % 60 == 0;

                if (Stage == OneWayCruiseStage.None)
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

                double currentAndDesiredSpeedDelta = Math.Abs(DesiredSpeed - mySpeed);

                if (Stage == OneWayCruiseStage.None)
                {
                    noSpeedOnStart = mySpeed <= 1;

                    Vector3D perpVel = Vector3D.ProjectOnPlane(ref myVelocity, ref targetDirection);
                    if (perpVel.LengthSquared() > maxInitialPerpendicularVelocity * maxInitialPerpendicularVelocity)
                        Stage = OneWayCruiseStage.CancelPerpendicularVelocity;
                    else
                        Stage = OneWayCruiseStage.OrientAndAccelerate;
                }

                if (Stage == OneWayCruiseStage.CancelPerpendicularVelocity)
                {
                    CancelPerpendicularVelocity();
                }

                if (Stage == OneWayCruiseStage.OrientAndAccelerate)
                {
                    OrientAndAccelerate(mySpeed);
                }

                if (Stage == OneWayCruiseStage.Complete)
                {
                    estimatedTimeOfArrival = 0;
                    SetDampenerState(true);
                    Terminate(distanceToTarget < 10 ? "Destination Reached" : "Terminated");
                }

                if (counter10)
                {

                    if (Stage <= OneWayCruiseStage.OrientAndAccelerate)
                    {
                        accelTime = (currentAndDesiredSpeedDelta / forwardAccelPremultiplied);
                        double accelDist = accelTime * ((mySpeed + DesiredSpeed) * 0.5);

                        double cruiseDist = distanceToTarget - accelDist;
                        cruiseTime = cruiseDist / DesiredSpeed;

                        vmax = 0;
                        estimatedTimeOfArrival = accelTime + cruiseTime;
                    }
                    else
                    {
                        accelTime = 0;
                        cruiseTime = distanceToTarget / mySpeed;
                        estimatedTimeOfArrival = cruiseTime;
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

            private void ResetThrustOverridesExceptBack()
            {
                foreach (var thruster in thrustController.Thrusters[Direction.Forward])
                    thruster.ThrustOverride = 0;
                thrustController.SetSideThrusts(0, 0, 0, 0);
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
            }

            private void CancelPerpendicularVelocity()
            {
                Vector3D aimDirection = -Vector3D.ProjectOnPlane(ref myVelocity, ref targetDirection);
                double perpSpeed = aimDirection.Length();

                if (perpSpeed <= maxInitialPerpendicularVelocity)
                {
                    Stage = OneWayCruiseStage.OrientAndAccelerate;
                    return;
                }

                Orient(aimDirection);

                if (!lastAimDirectionAngleRad.HasValue)
                {
                    lastAimDirectionAngleRad = AngleRadiansBetweenVectorAndControllerForward(aimDirection);
                }

                if (lastAimDirectionAngleRad.Value <= OrientToleranceAngleRadians)
                {
                    float overrideAmount = MathHelper.Clamp(((float)perpSpeed * 2 * gridMass) * forwardThrustInv, 0, MaxThrustRatio);
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
                    ResetThrustOverridesExceptBack();
                }
            }

            private void OrientAndAccelerate(double mySpeed)
            {
                bool approaching = Vector3D.Dot(targetDirection, myVelocity) > 0;
                if (!approaching && approachingTarget)
                {
                    Stage = OneWayCruiseStage.Complete;
                    return;
                }

                if (approaching)
                {
                    approachingTarget = true;
                }

                Vector3D aimDirection = targetDirection;
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
                    noSpeedOnStart = false;

                    double accel = mySpeed - lastMySpeed;
                    float expectedAccel = forwardAccel * lastThrustRatio / 6;
                    double speedDelta = DesiredSpeed - mySpeed;

                    float desiredAccel = (float)((speedDelta) + (expectedAccel - accel) * 6);
                    float thrustRatio = MathHelper.Clamp(desiredAccel / forwardAccel, 0f, MaxThrustRatio);

                    foreach (var thruster in thrustController.Thrusters[Direction.Forward])
                    {
                        thruster.ThrustOverridePercentage = thrustRatio;
                    }

                    lastThrustRatio = thrustRatio;

                    DampenSideways(myVelocity * 0.1);
                    return;
                }

                ResetThrustOverridesExceptBack();
            }

            private double AngleRadiansBetweenVectorAndControllerForward(Vector3D vec)
            {
                //don't do unnecessary sqrt for controller.matrix.forward because its already a unit vector
                double cos = ShipController.WorldMatrix.Forward.Dot(vec) / vec.Length();
                double angle = Math.Acos(cos);
                if (double.IsNaN(angle))
                    angle = 0;
                return angle;
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
                Stage = OneWayCruiseStage.Aborted;
                Terminate("Aborted");
            }

            protected override void OnNoFunctionalGyrosLeft() => Terminate("No functional gyros found");
        }
    }
}
