using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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

            const double DegToRadMulti = Math.PI / 180.0;
            const double RadToDegMulti = 180.0 / Math.PI;

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
                        UpdateThrustAndAccel();
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

            //updated every 10 ticks
            //how far off the aim is from the desired direction
            private double? lastAimDirectionAngleRad = null;
            private Vector3D naturalGravity;
            private double estimatedTimeOfArrival;

            // accel stage variables
            private double lastForwardSpeedDuringAccel;
            private double lastForwardThrustRatioDuringAccel;

            //updated every tick
            private double accelTime, cruiseTime, distanceToTarget, vmax;
            private bool approachingTarget;

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

                UpdateThrustAndAccel();
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

            private void DampenSidewaysToZero(Vector3D shipVelocity, float ups)
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
                bool counter30 = counter % 30 == 0;

                if (Stage == OneWayCruiseStage.None)
                {
                    ResetGyroOverride();
                    thrustController.ResetThrustOverrides();
                    thrustController.TurnOnAllThrusters();
                    UpdateThrustAndAccel();
                }

                if (counter10)
                {
                    lastAimDirectionAngleRad = null;
                    naturalGravity = ShipController.GetNaturalGravity();
                    SetDampenerState(false);
                }
                if (counter30)
                {
                    gridMass = ShipController.CalculateShipMass().PhysicalMass;
                    UpdateThrustAndAccel();
                }

                Vector3D velocity = ShipController.GetShipVelocities().LinearVelocity + naturalGravity;
                double velocityLength = velocity.Length();

                Vector3D displacement = Target - ShipController.WorldAABB.Center;
                Vector3D targetDirection = Utils.Normalize(ref displacement, out distanceToTarget); // normalize

                if (Stage == OneWayCruiseStage.None)
                {
                    Vector3D perpVel = Vector3D.ProjectOnPlane(ref velocity, ref targetDirection);
                    if (perpVel.LengthSquared() > maxInitialPerpendicularVelocity * maxInitialPerpendicularVelocity)
                        Stage = OneWayCruiseStage.CancelPerpendicularVelocity;
                    else
                        Stage = OneWayCruiseStage.OrientAndAccelerate;
                }

                if (Stage == OneWayCruiseStage.CancelPerpendicularVelocity)
                {
                    CancelPerpendicularVelocity(velocity, targetDirection);
                }

                if (Stage == OneWayCruiseStage.OrientAndAccelerate)
                {
                    OrientAndAccelerate(velocity, velocityLength, targetDirection);
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
                        double currentAndDesiredSpeedDelta = Math.Abs(DesiredSpeed - velocityLength);

                        accelTime = currentAndDesiredSpeedDelta / (forwardAccel * MaxThrustRatio);
                        double accelDist = accelTime * ((velocityLength + DesiredSpeed) * 0.5);

                        double cruiseDist = distanceToTarget - accelDist;
                        cruiseTime = cruiseDist / DesiredSpeed;

                        vmax = 0;
                        estimatedTimeOfArrival = accelTime + cruiseTime;
                    }
                    else
                    {
                        accelTime = 0;
                        cruiseTime = distanceToTarget / velocityLength;
                        estimatedTimeOfArrival = cruiseTime;
                    }
                }
            }

            private void UpdateThrustAndAccel()
            {
                thrustController.UpdateThrusts();
                forwardAccel = (float)(thrustController.GetThrustInDirection(Direction.Forward) / gridMass);
            }

            private void ResetThrustOverridesExceptFront()
            {
                var backwardThrusters = thrustController.Thrusters[Direction.Backward];
                for (int i = backwardThrusters.Count - 1; i >= 0; i--)
                {
                    backwardThrusters[i].ThrustOverridePercentage = 0;
                }
                thrustController.SetSideThrusts(0, 0, 0, 0);
            }

            private void SetDampenerState(bool enabled) => ShipController.DampenersOverride = enabled;

            private void OnStageChanged()
            {
                thrustController.ResetThrustOverrides();
                ResetGyroOverride();
                SetDampenerState(false);
                lastAimDirectionAngleRad = null;

                // reset stage-specific variables
                lastForwardSpeedDuringAccel = 0;
                lastForwardThrustRatioDuringAccel = 0;
            }

            private void CancelPerpendicularVelocity(Vector3D velocity, Vector3D targetDir)
            {
                Vector3D aimDirection = -Vector3D.ProjectOnPlane(ref velocity, ref targetDir);
                double perpSpeed = aimDirection.Length();

                if (perpSpeed <= maxInitialPerpendicularVelocity)
                {
                    Stage = OneWayCruiseStage.OrientAndAccelerate;
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

                const float UPS = 6;

                float forwardOverrideRatio = lastAimDirectionAngleRad.Value <= OrientToleranceAngleRadians ? Math.Min(MaxThrustRatio, (float)(perpSpeed / forwardAccel * UPS)) : 0;
                var forwardThrusters = thrustController.Thrusters[Direction.Forward];
                for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                {
                    forwardThrusters[i].ThrustOverridePercentage = forwardOverrideRatio;
                }

                thrustController.SetSideThrusts(0, 0, 0, 0);
                ResetThrustOverridesExceptFront();
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="velocity">World velocity</param>
            /// <param name="velocityLength">Length of velocity</param>
            /// <param name="targetDir">Normalized direction toward the target</param>
            private void OrientAndAccelerate(Vector3D velocity, double velocityLength, Vector3D targetDir)
            {
                bool closing = Vector3D.Dot(targetDir, velocity) > 0;
                if (!closing && approachingTarget) // we just passed the target
                {
                    Stage = OneWayCruiseStage.Complete;
                    return;
                }

                approachingTarget = closing;

                Orient(targetDir);

                if (!counter10)
                {
                    return;
                }

                if (!lastAimDirectionAngleRad.HasValue)
                {
                    lastAimDirectionAngleRad = AngleRadiansBetweenVectorAndControllerForward(targetDir);
                }

                const float UPS = 6;

                if (lastAimDirectionAngleRad.Value <= OrientToleranceAngleRadians)
                {
                    // speed directly towards the target
                    // >0 if closing, <0 ottherwise
                    double forwardSpeed = velocityLength > 0 ? (velocityLength * Vector3D.Dot(velocity / velocityLength, targetDir)) : 0;

                    double actualAccel = (forwardSpeed - lastForwardSpeedDuringAccel) * UPS;
                    double expectedAccel = forwardAccel * lastForwardThrustRatioDuringAccel;

                    double speedDelta = DesiredSpeed - forwardSpeed;
                    double desiredAccel = speedDelta + (expectedAccel - actualAccel);
                    float thrustRatio = Math.Min(MaxThrustRatio, (float)(desiredAccel / forwardAccel * UPS));

                    var forwardThrusters = thrustController.Thrusters[Direction.Forward];
                    for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                    {
                        forwardThrusters[i].ThrustOverridePercentage = thrustRatio;
                    }

                    Vector3D velocityPerpendicularToTarget = Vector3D.ProjectOnPlane(ref velocity, ref targetDir);
                    DampenSidewaysToZero(velocityPerpendicularToTarget, UPS);

                    lastForwardSpeedDuringAccel = forwardSpeed;
                    lastForwardThrustRatioDuringAccel = thrustRatio;

                    return;
                }

                thrustController.ResetThrustOverrides();
            }

            private double AngleRadiansBetweenVectorAndControllerForward(Vector3D vec)
            {
                Vector3D.Normalize(ref vec, out vec);
                double cos = Vector3D.Dot(ShipController.WorldMatrix.Forward, vec);
                double angle = Math.Acos(cos);
                return double.IsNaN(angle) ? 0 : angle;
            }

            public void Terminate(string reason)
            {
                thrustController.ResetThrustOverrides();
                thrustController.TurnOnAllThrusters();

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
