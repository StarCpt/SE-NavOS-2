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
    public class RetroCruiseControl2 : OrientControllerBase, ICruiseController
    {
        public enum Stage
        {
            None,
            CancelPerpendicularVelocity,
            CancelPerpendicularVelocityFullStop,
            Accelerate,
            Decelerate,
            Approach,

            // emergency stages
            CollisionAvoidance,
            FullStopOvershoot,

            Complete,
            Terminated,
        }

        const double PERPENDICULAR_SPEED_THRESHOLD = 1;
        const double DECEL_RESERVE_THRUST = 0.05; // % of allowed thrust to reserve to recover from minor overshoots
        const double COMPLETION_SPEED = 0.05;
        const double COMPLETION_DISTANCE = 5;
        const double AIM_ONTARGET_ANGLE_COS = 0.99999; // cos(rad) of desired angle

        public event CruiseTerminateEventDelegate CruiseTerminated;

        public string Name { get; } = "CruiseNew";
        public double ShipFlipTime { get; set; } = 10;

        private Stage _stage = Stage.None;
        private Vector3D _prevPos;
        private Vector3D _prevVelocity;
        private Vector3D _prevAimDir;

        private readonly IMyShipController _controller;
        private readonly VariableThrustController _thrustController;
        private readonly Vector3D _target;

        public RetroCruiseControl2(IMyShipController shipController, VariableThrustController thrustController, IAimController aimController, IList<IMyGyro> gyros, Vector3D target)
            : base(aimController, shipController, gyros)
        {
            _controller = shipController;
            _thrustController = thrustController;
            _target = target;
        }

        private int _counter = -1;

        public void Run()
        {
            if (_stage == Stage.Terminated)
            {
                return;
            }

            _counter++;

            Vector3D currentPos = _controller.WorldAABB.Center;
            Vector3D currentVelocity = _controller.GetShipVelocities().LinearVelocity;
            double currentSpeed = currentVelocity.Length();

            Vector3D targetDir = _target - currentPos;
            double targetDist;
            targetDir = Utils.Normalize(ref targetDir, out targetDist);

            bool closing = Vector3D.Dot(currentVelocity, targetDir) > 0;

            _thrustController.UpdateThrusts();
            _controller.DampenersOverride = false;

            float gridMass = _controller.CalculateShipMass().PhysicalMass;
            double forwardThrust = _thrustController.GetThrustInDirection(Direction.Forward);
            double forwardAccel = forwardThrust / gridMass;

            const int THRUST_UPS = 6;
            const double THRUST_TIME_STEP = 1.0 / THRUST_UPS;

            bool update10 = _counter % 10 == 0;
            bool stageChanged = false;

            if (_stage == Stage.None)
            {
                if (targetDist <= Math.Max(COMPLETION_DISTANCE, _controller.CubeGrid.WorldVolume.Radius))
                {
                    Terminate("Aborted, too close to target.");
                    return;
                }

                double perpendicularSpeed = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir).Length();
                _stage = perpendicularSpeed > PERPENDICULAR_SPEED_THRESHOLD ? Stage.CancelPerpendicularVelocity : Stage.Accelerate;
                stageChanged = true;

                // do other init stuff
            }

            if (_stage == Stage.CancelPerpendicularVelocity)
            {
                // if moving away, full stop
                //   we can loosen aim threshold to decel-perpendicular plane between full perpendicular and full decel directions

                // if closing, stop perp vel wrt entire decel
                //   if we can't kill perp vel before needing to decel, preemptively do a full stop

                if (!closing)
                {
                    _stage = Stage.CancelPerpendicularVelocityFullStop;
                    stageChanged = true;
                }
                else
                {
                    Vector3D targetDir2 = targetDir;
                    Vector3D perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir2);
                    double perpSpeed = perpVel.Length();
                    Vector3D velocityInTargetDir = Vector3D.ProjectOnVector(ref currentVelocity, ref targetDir2);
                    double timeToStopPerpVel = perpSpeed / forwardAccel;
                    Vector3D drift = (velocityInTargetDir * timeToStopPerpVel) + (perpVel * 0.5 * timeToStopPerpVel);

                    for (int i = 0; i < 10; i++)
                    {
                        // perpendicular vector will have changed at this point, recompute using the resulting perpendicular vector
                        targetDir2 = Vector3D.Normalize(_target - (currentPos + drift));
                        perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir2);
                        perpSpeed = perpVel.Length();
                        velocityInTargetDir = Vector3D.ProjectOnVector(ref currentVelocity, ref targetDir2);
                        timeToStopPerpVel = perpSpeed / forwardAccel;
                        drift = (velocityInTargetDir * timeToStopPerpVel) + (perpVel * 0.5 * timeToStopPerpVel);
                    }

                    // if final velocity heads away from target, just do a full stop instead
                    bool approachingAtEnd = Vector3D.Dot(velocityInTargetDir, targetDir2) > 0;

                    // or if we're approaching but don't have enough dist to stop, also full stop
                    double actualStopTimeAtEnd = velocityInTargetDir.Length() / forwardAccel;
                    // flipTime * 0.5 since we're only turning 90 degrees
                    double actualStopDistAtEndSq = ((velocityInTargetDir * (ShipFlipTime * 0.5)) + (velocityInTargetDir * 0.5 * actualStopTimeAtEnd)).LengthSquared();
                    double availableStopDistAtEndSq = Vector3D.DistanceSquared(currentPos + drift, _target);
                    bool canStopAtEnd = actualStopDistAtEndSq < availableStopDistAtEndSq;

                    if (!approachingAtEnd || !canStopAtEnd)
                    {
                        _stage = Stage.CancelPerpendicularVelocityFullStop;
                        stageChanged = true;
                    }
                    else if (perpSpeed < PERPENDICULAR_SPEED_THRESHOLD)
                    {
                        _stage = Stage.Accelerate;
                        stageChanged = true;
                    }
                    else
                    {
                        Vector3D desiredAimDir = -perpVel.Normalized();
                        Orient(desiredAimDir);
                        bool onTarget = Vector3D.Dot(desiredAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS; // ~0.256 degrees

                        if (update10 || stageChanged)
                        {
                            float forwardThrustRatio = onTarget ? (float)(perpSpeed / (forwardAccel * THRUST_TIME_STEP)) : 0;
                            List<IMyThrust> forwardThrusters = _thrustController.Thrusters[Direction.Forward];
                            for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                            {
                                forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
                            }
                        }
                    }
                }
            }

            if (_stage == Stage.CancelPerpendicularVelocityFullStop)
            {
                Vector3D desiredAimDir = -currentVelocity.Normalized();
                Orient(desiredAimDir);
                bool onTarget = Vector3D.Dot(desiredAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                if (update10 || stageChanged)
                {
                    float forwardThrustRatio = onTarget ? (float)(currentSpeed / (forwardAccel * THRUST_TIME_STEP)) : 0;
                    List<IMyThrust> forwardThrusters = _thrustController.Thrusters[Direction.Forward];
                    for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                    {
                        forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
                    }
                }

                if (currentSpeed < 0.05)
                {
                    _stage = Stage.Accelerate;
                    stageChanged = true;
                }
            }

            if (_stage == Stage.Accelerate)
            {
                // TODO: if perpendicular velocity exceeds a certain amount, switch to CancelPerpendicularVelocity

                double stopTime = currentSpeed / forwardAccel;
                double stopDist = (currentSpeed * ShipFlipTime) + (currentSpeed * 0.5 * stopTime);
                double availableDist = targetDist;
                if (stopDist >= availableDist)
                {
                    _stage = Stage.Decelerate;
                    stageChanged = true;
                }
                else
                {
                    Orient(targetDir);
                    bool onTarget = Vector3D.Dot(targetDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;
                    
                    if (update10 || stageChanged)
                    {
                        float forwardThrustRatio = onTarget ? 1 : 0;
                        List<IMyThrust> forwardThrusters = _thrustController.Thrusters[Direction.Forward];
                        for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                        {
                            forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
                        }

                        if (onTarget)
                        {
                            Vector3D perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir);
                            DampenSidewaysToZero(perpVel, gridMass, THRUST_UPS);
                        }
                    }
                }
            }

            if (_stage == Stage.Decelerate)
            {
                if (!closing)
                {
                    _stage = Stage.FullStopOvershoot;
                    stageChanged = true;
                }
                else if (targetDist < Math.Max(COMPLETION_DISTANCE, _controller.CubeGrid.WorldVolume.Radius) || targetDist < currentSpeed * THRUST_TIME_STEP)
                {
                    _stage = Stage.Approach;
                    stageChanged = true;
                    _prevAimDir = _controller.WorldMatrix.Forward;
                }
                else
                {
                    Orient(-targetDir);
                    bool onTarget = Vector3D.Dot(-targetDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                    if (update10 || stageChanged)
                    {
                        double closingSpeed = currentSpeed > 0 ? currentSpeed * Vector3D.Dot(currentVelocity.Normalized(), targetDir) : 0;

                        double desiredStopDist = closing ? targetDist : 0;
                        double desiredStopTime = desiredStopDist / (closingSpeed * 0.5) - THRUST_TIME_STEP;
                        double desiredStopAccel = (closing && desiredStopTime > 0 && closingSpeed > 0) ? (1.0 / (desiredStopTime / closingSpeed)) : forwardAccel;
                        bool shouldDecel = desiredStopAccel >= forwardAccel * (1 - DECEL_RESERVE_THRUST);

                        float forwardThrustRatio = (onTarget && shouldDecel) ? (float)(desiredStopAccel / forwardAccel) : 0;
                        List<IMyThrust> forwardThrusters = _thrustController.Thrusters[Direction.Forward];
                        for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                        {
                            forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
                        }

                        if (onTarget)
                        {
                            Vector3D perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir);
                            DampenSidewaysToZero(perpVel, gridMass, THRUST_UPS);
                        }
                    }
                }
            }

            if (_stage == Stage.Approach)
            {
                Orient(_prevAimDir);
                if (currentSpeed < COMPLETION_SPEED)
                {
                    _stage = Stage.Complete;
                    stageChanged = true;
                }
                else if (!closing)
                {
                    _thrustController.DampenAllDirections(currentVelocity, gridMass, THRUST_UPS);
                }
                else
                {
                    bool onTarget = Vector3D.Dot(-targetDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;
                    
                    if (update10 || stageChanged)
                    {
                        double closingSpeed = currentSpeed > 0 ? currentSpeed * Vector3D.Dot(currentVelocity.Normalized(), targetDir) : 0;

                        double desiredStopDist = closing ? targetDist : 0;
                        double desiredStopTime = desiredStopDist / (closingSpeed * 0.5) - THRUST_TIME_STEP;
                        double desiredStopAccel = (closing && desiredStopTime > 0 && closingSpeed > 0) ? (1.0 / (desiredStopTime / closingSpeed)) : forwardAccel;

                        float forwardThrustRatio = onTarget ? (float)(desiredStopAccel / forwardAccel) : 0;
                        List<IMyThrust> forwardThrusters = _thrustController.Thrusters[Direction.Forward];
                        for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                        {
                            forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
                        }

                        if (onTarget)
                        {
                            Vector3D perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir);
                            DampenSidewaysToZero(perpVel, gridMass, THRUST_UPS);
                        }
                    }
                }
            }

            if (_stage == Stage.CollisionAvoidance)
            {
                throw new Exception("Not Implemented");
            }

            if (_stage == Stage.FullStopOvershoot)
            {
                Vector3D desiredAimDir = -currentVelocity.Normalized();
                Orient(desiredAimDir);
                bool onTarget = Vector3D.Dot(desiredAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                if (update10 || stageChanged)
                {
                    float forwardThrustRatio = onTarget ? (float)(currentSpeed / (forwardAccel * THRUST_TIME_STEP)) : 0;
                    List<IMyThrust> forwardThrusters = _thrustController.Thrusters[Direction.Forward];
                    for (int i = forwardThrusters.Count - 1; i >= 0; i--)
                    {
                        forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
                    }
                }

                if (currentSpeed < 0.05)
                {
                    _controller.DampenersOverride = true;
                    Terminate("Cruise terminated due to target overshoot");
                    return;
                }
            }

            if (_stage == Stage.Complete)
            {
                _controller.DampenersOverride = true;
                Terminate(targetDist < COMPLETION_DISTANCE ? "Destination reached" : "Terminated");
            }
        }

        private void DampenSidewaysToZero(Vector3D shipVelocity, float gridMass, float ups)
        {
            Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
            Vector3 thrustAmount = localVelocity * gridMass * ups;
            float right = thrustAmount.X < 0 ? -thrustAmount.X : 0;
            float left  = thrustAmount.X > 0 ? thrustAmount.X : 0;
            float up    = thrustAmount.Y < 0 ? -thrustAmount.Y : 0;
            float down  = thrustAmount.Y > 0 ? thrustAmount.Y : 0;
            _thrustController.SetSideThrusts(left, right, up, down);
        }

        public void AppendStatus(StringBuilder strb)
        {
        }

        public void Abort() => Terminate("Aborted");
        protected override void OnNoFunctionalGyrosLeft() => Terminate("No functional gyros");
        public void Terminate(string reason)
        {
            _stage = Stage.Terminated;

            if (_controller.GetShipSpeed() < COMPLETION_SPEED)
            {
                _controller.DampenersOverride = true;
            }

            _thrustController.ResetThrustOverrides();
            ResetGyroOverride();
            CruiseTerminated.Invoke(this, reason);
        }
    }
}
