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
    public class RetroCruiseControl : OrientControllerBase, ICruiseController
    {
        public enum CruiseStage : byte
        {
            None,
            CancelPerpendicularVelocity,
            CancelPerpendicularVelocityFullStop,
            Accelerate,
            Decelerate,
            DecelerateNoOrient,

            //CollisionAvoidance,
            Overshoot,

            Complete,
            Aborted,
            Terminated,
        }

        public event CruiseTerminateEventDelegate CruiseTerminated = delegate { };

        public string Name { get; } = "Cruise";
        public CruiseStage Stage
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
        public float MaxThrustRatio => _thrustController.MaxForwardThrustRatio;
        public double ShipFlipTimeInSeconds { get; set; } = 10;

        const double PERPENDICULAR_SPEED_THRESHOLD = 1;
        const double DECEL_RESERVE_THRUST = 0.05; // % of allowed thrust to reserve to recover from minor overshoots
        const double TARGET_REACHED_SPEED = 0.05;
        const double TARGET_REACHED_DISTANCE = 5;
        const double AIM_ONTARGET_ANGLE_COS = 0.99999; // cos(rad) of desired angle

        private readonly VariableThrustController _thrustController;
        private readonly Program _program;
        private readonly Config _config;

        //active variables
        private CruiseStage _stage;
        private int _counter = -1;

        //updated every 30 ticks
        private float _gridMass;
        private float _forwardAccelPremult;

        //updated every 10 ticks
        private Vector3D _naturalGravity;

        // accel stage variables
        private double _lastForwardSpeedDuringAccel;
        private double _lastForwardThrustRatioDuringAccel;

        //updated every tick
        private double _targetDist;
        private Vector3D _targetDir;
        private Vector3D _prevAimDir;
        private CruiseStage _initialStage = CruiseStage.None;

        private bool savePersistentData;
        private readonly IMyShipController _controller;

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
            this._thrustController = thrustController;
            this._program = program;
            this._config = program.config;
            this.savePersistentData = savePersistentData;
            this._controller = controller;

            Stage = CruiseStage.None;
            _gridMass = controller.CalculateShipMass().PhysicalMass;

            UpdateThrustAndAccel();
        }

        public RetroCruiseControl(
            Vector3D target,
            double desiredSpeed,
            IAimController aimControl,
            IMyShipController controller,
            IList<IMyGyro> gyros,
            VariableThrustController thrustController,
            Program program,
            CruiseStage stage,
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
            _initialStage = stage;
            this.savePersistentData = savePersistentData;
        }

        public void AppendStatus(StringBuilder strb)
        {
        }

        public static string GetShortDistance(double meters)
        {
            return meters >= 1000 ? (meters / 1000).ToString("0.## km") : meters.ToString("0 m");
        }

        /// <param name="shipVelocity">Current world space velocity</param>
        /// <param name="ups">Number of times this method is run per second</param>
        private void DampenSidewaysToZero(Vector3D shipVelocity, float ups)
        {
            Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
            Vector3 thrustAmount = localVelocity * _gridMass * ups;
            const float EPSILON = 0.00001f;
            float right = thrustAmount.X < -EPSILON ? -thrustAmount.X : 0;
            float left  = thrustAmount.X >  EPSILON ?  thrustAmount.X : 0;
            float up    = thrustAmount.Y < -EPSILON ? -thrustAmount.Y : 0;
            float down  = thrustAmount.Y >  EPSILON ?  thrustAmount.Y : 0;
            _thrustController.SetSideThrusts(left, right, up, down);
        }
        
        const int THRUST_UPS = 6;
        const double THRUST_TIME_STEP = 1.0 / THRUST_UPS;

        public void Run()
        {
            _counter++;
            bool update10 = _counter % 10 == 0;
            bool update30 = _counter % 30 == 0;

            if (Stage == CruiseStage.None)
            {
                ResetGyroOverride();
                _thrustController.ResetThrustOverrides();
            }

            if (update10 || Stage == CruiseStage.None)
            {
                _naturalGravity = ShipController.GetNaturalGravity();
                SetDampenerState(false);
            }
            if (update30 || Stage == CruiseStage.None)
            {
                _gridMass = ShipController.CalculateShipMass().PhysicalMass;
                UpdateThrustAndAccel();
            }

            Vector3D currentPos = ShipController.WorldAABB.Center;
            Vector3D currentVelocity = ShipController.GetShipVelocities().LinearVelocity + _naturalGravity;
            double currentSpeed = currentVelocity.Length();

            Vector3D displacement = Target - currentPos;
            _targetDir = Utils.Normalize(ref displacement, out this._targetDist);

            bool closing = Vector3D.Dot(currentVelocity, _targetDir) > 0;

            bool stageChanged = false;
            Vector3D targetDir = _targetDir;
            double targetDist = _targetDist;

            if (update10)
            {
                // compute ETA
            }

            if (Stage == CruiseStage.None)
            {
                if (targetDist <= Math.Max(TARGET_REACHED_DISTANCE, _controller.CubeGrid.WorldVolume.Radius))
                {
                    Stage = CruiseStage.Terminated;
                    Terminate("Aborted, too close to target.");
                    return;
                }

                if (_initialStage != CruiseStage.None)
                {
                    Stage = _initialStage;
                }
                else
                {
                    double perpSpeedSq = Vector3D.ProjectOnPlane(ref currentVelocity, ref _targetDir).LengthSquared();
                    Stage = perpSpeedSq > PERPENDICULAR_SPEED_THRESHOLD * PERPENDICULAR_SPEED_THRESHOLD
                        ? CruiseStage.CancelPerpendicularVelocity
                        : CruiseStage.Accelerate;
                }
            }

            if (Stage == CruiseStage.CancelPerpendicularVelocity)
            {
                // if moving away, full stop

                // if closing, stop perp vel wrt entire decel
                //   if we can't kill perp vel before needing to decel, preemptively do a full stop

                if (!closing)
                {
                    _stage = CruiseStage.CancelPerpendicularVelocityFullStop;
                    stageChanged = true;
                }
                else
                {
                    Vector3D targetDir2 = targetDir;
                    Vector3D perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir2);
                    double perpSpeed = perpVel.Length();
                    Vector3D velocityInTargetDir = Vector3D.ProjectOnVector(ref currentVelocity, ref targetDir2);
                    double timeToStopPerpVel = perpSpeed / _forwardAccelPremult;
                    Vector3D drift = (velocityInTargetDir * timeToStopPerpVel) + (perpVel * 0.5 * timeToStopPerpVel);

                    for (int i = 0; i < 10; i++)
                    {
                        // perpendicular vector will have changed at this point, recompute using the resulting perpendicular vector
                        targetDir2 = Vector3D.Normalize(Target - (currentPos + drift));
                        perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir2);
                        perpSpeed = perpVel.Length();
                        velocityInTargetDir = Vector3D.ProjectOnVector(ref currentVelocity, ref targetDir2);
                        timeToStopPerpVel = perpSpeed / _forwardAccelPremult;
                        drift = (velocityInTargetDir * timeToStopPerpVel) + (perpVel * 0.5 * timeToStopPerpVel);
                    }

                    // if final velocity heads away from target, just do a full stop instead
                    bool approachingAtEnd = Vector3D.Dot(velocityInTargetDir, targetDir2) > 0;

                    // or if we're approaching but don't have enough dist to stop, also full stop
                    double actualStopTimeAtEnd = velocityInTargetDir.Length() / _forwardAccelPremult;
                    // flipTime * 0.5 since we're only turning 90 degrees
                    double actualStopDistAtEndSq = ((velocityInTargetDir * (ShipFlipTimeInSeconds * 0.5)) + (velocityInTargetDir * 0.5 * actualStopTimeAtEnd)).LengthSquared();
                    double availableStopDistAtEndSq = Vector3D.DistanceSquared(currentPos + drift, Target);
                    bool canStopAtEnd = actualStopDistAtEndSq < availableStopDistAtEndSq;

                    if (!approachingAtEnd || !canStopAtEnd)
                    {
                        _stage = CruiseStage.CancelPerpendicularVelocityFullStop;
                        stageChanged = true;
                    }
                    else if (perpSpeed < PERPENDICULAR_SPEED_THRESHOLD)
                    {
                        _stage = CruiseStage.Accelerate;
                        stageChanged = true;
                    }
                    else
                    {
                        Vector3D desiredAimDir = -perpVel.Normalized();
                        Orient(desiredAimDir);
                        bool onTarget = Vector3D.Dot(desiredAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS; // ~0.256 degrees

                        if (update10 || stageChanged)
                        {
                            float forwardThrustRatio = onTarget ? (float)(perpSpeed / (_forwardAccelPremult * THRUST_TIME_STEP)) : 0;
                            forwardThrustRatio = MathHelper.Saturate(forwardThrustRatio) * MaxThrustRatio;
                            SetForwardThrustAndResetBackThrusts(forwardThrustRatio);
                            _thrustController.SetSideThrusts(0, 0, 0, 0);
                        }
                    }
                }
            }

            if (_stage == CruiseStage.CancelPerpendicularVelocityFullStop)
            {
                Vector3D desiredAimDir = -currentVelocity.Normalized();
                Orient(desiredAimDir);
                bool onTarget = Vector3D.Dot(desiredAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                if (update10 || stageChanged)
                {
                    float forwardThrustRatio = onTarget ? (float)(currentSpeed / (_forwardAccelPremult * THRUST_TIME_STEP)) : 0;
                    forwardThrustRatio = MathHelper.Saturate(forwardThrustRatio) * MaxThrustRatio;
                    SetForwardThrustAndResetBackThrusts(forwardThrustRatio);
                    _thrustController.SetSideThrusts(0, 0, 0, 0);
                }

                if (currentSpeed < 0.05)
                {
                    _stage = CruiseStage.Accelerate;
                    stageChanged = true;
                }
            }

            if (Stage == CruiseStage.Accelerate)
            {
                double stopTime = currentSpeed / _forwardAccelPremult;
                double stopDist = (currentSpeed * ShipFlipTimeInSeconds) + (currentSpeed * 0.5 * stopTime);
                double availableDist = targetDist;
                if (stopDist >= availableDist)
                {
                    _stage = CruiseStage.Decelerate;
                    stageChanged = true;
                }
                else
                {
                    Orient(targetDir);
                    bool onTarget = Vector3D.Dot(targetDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                    if (onTarget && (update10 || stageChanged))
                    {
                        double closingSpeed = currentSpeed > 0 ? currentSpeed * Vector3D.Dot(currentVelocity.Normalized(), targetDir) : 0;

                        float forwardThrustRatio;
                        if (closingSpeed <= 0)
                        {
                            double speedDelta = DesiredSpeed - closingSpeed;
                            double desiredThrustRatio = _forwardAccelPremult > 0 ? (speedDelta / _forwardAccelPremult * THRUST_UPS) : 0;
                            forwardThrustRatio = (float)desiredThrustRatio;
                        }
                        else
                        {
                            double expectedAccel = _forwardAccelPremult * _lastForwardThrustRatioDuringAccel * THRUST_TIME_STEP;
                            double actualAccel = closingSpeed - _lastForwardSpeedDuringAccel;

                            double speedDelta = DesiredSpeed - closingSpeed;
                            double desiredAccel = speedDelta + (expectedAccel - actualAccel);
                            double desiredThrustRatio = _forwardAccelPremult > 0 ? (desiredAccel / _forwardAccelPremult * THRUST_UPS) : 0;
                            forwardThrustRatio = (float)desiredThrustRatio;
                        }
                        forwardThrustRatio = MathHelper.Saturate(forwardThrustRatio) * MaxThrustRatio;

                        _lastForwardSpeedDuringAccel = MathHelper.IsValid(closingSpeed) ? closingSpeed : 0;
                        _lastForwardThrustRatioDuringAccel = MathHelper.IsValid(forwardThrustRatio) ? forwardThrustRatio : 0;

                        SetForwardThrustAndResetBackThrusts(forwardThrustRatio);

                        Vector3D perpVel = Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir);
                        DampenSidewaysToZero(perpVel, THRUST_UPS);
                    }
                    else if (update10 || stageChanged)
                    {
                        _thrustController.ResetThrustOverrides();
                    }
                }
            }

            if (Stage == CruiseStage.Decelerate)
            {
                if (!closing)
                {
                    Stage = CruiseStage.Overshoot;
                    stageChanged = true;
                }
                else if (targetDist < Math.Max(TARGET_REACHED_DISTANCE, _controller.CubeGrid.WorldVolume.Radius) || targetDist < currentSpeed * THRUST_TIME_STEP || currentSpeed <= TARGET_REACHED_SPEED)
                {
                    Stage = CruiseStage.DecelerateNoOrient;
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
                        double desiredStopAccel = (closing && desiredStopTime > 0 && closingSpeed > 0) ? (1.0 / (desiredStopTime / closingSpeed)) : _forwardAccelPremult;
                        bool shouldDecel = desiredStopAccel >= _forwardAccelPremult * (1 - DECEL_RESERVE_THRUST);

                        if (!shouldDecel && _forwardAccelPremult > 0)
                        {
                            // decel anyway if we'll otherwise overshoot
                            double actualStopTime = currentSpeed / _forwardAccelPremult;
                            double actualStopDist = currentSpeed * 0.5 * actualStopTime;
                            shouldDecel |= actualStopDist >= targetDist - (closingSpeed * THRUST_TIME_STEP);
                        }

                        float forwardThrustRatio = (onTarget && shouldDecel) ? (float)(desiredStopAccel / _forwardAccelPremult) : 0;
                        forwardThrustRatio = MathHelper.Saturate(forwardThrustRatio) * MaxThrustRatio;
                        SetForwardThrustAndResetBackThrusts(forwardThrustRatio);

                        Vector3D perpVel = onTarget ? Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir) : Vector3D.Zero;
                        DampenSidewaysToZero(perpVel, THRUST_UPS);
                    }
                }
            }

            if (Stage == CruiseStage.DecelerateNoOrient)
            {
                Orient(_prevAimDir);
                if (currentSpeed < TARGET_REACHED_SPEED)
                {
                    Stage = CruiseStage.Complete;
                    stageChanged = true;
                }
                else if (!closing && (update10 || stageChanged))
                {
                    _thrustController.DampenAllDirections(currentVelocity, _gridMass, THRUST_UPS);
                }
                else if (closing && (update10 || stageChanged))
                {
                    bool onTarget = Vector3D.Dot(_prevAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                    double closingSpeed = currentSpeed > 0 ? currentSpeed * Vector3D.Dot(currentVelocity.Normalized(), targetDir) : 0;

                    double desiredStopDist = closing ? targetDist : 0;
                    double desiredStopTime = desiredStopDist / (closingSpeed * 0.5) - THRUST_TIME_STEP;
                    double desiredStopAccel = (closing && desiredStopTime > 0 && closingSpeed > 0) ? (1.0 / (desiredStopTime / closingSpeed)) : _forwardAccelPremult;

                    float forwardThrustRatio = onTarget ? (float)(desiredStopAccel / _forwardAccelPremult) : 0;
                    forwardThrustRatio = MathHelper.Saturate(forwardThrustRatio) * MaxThrustRatio;
                    SetForwardThrustAndResetBackThrusts(forwardThrustRatio);

                    Vector3D perpVel = onTarget ? Vector3D.ProjectOnPlane(ref currentVelocity, ref targetDir) : Vector3D.Zero;
                    DampenSidewaysToZero(perpVel, THRUST_UPS);
                }
            }

            //if (_stage == CruiseStage.CollisionAvoidance)
            //{
            //    throw new Exception("Not Implemented");
            //}

            if (_stage == CruiseStage.Overshoot)
            {
                Vector3D desiredAimDir = -currentVelocity.Normalized();
                Orient(desiredAimDir);
                bool onTarget = Vector3D.Dot(desiredAimDir, _controller.WorldMatrix.Forward) > AIM_ONTARGET_ANGLE_COS;

                if (onTarget && (update10 || stageChanged))
                {
                    float forwardThrustRatio = onTarget ? (float)(currentSpeed / (_forwardAccelPremult * THRUST_TIME_STEP)) : 0;
                    forwardThrustRatio = MathHelper.Saturate(forwardThrustRatio) * MaxThrustRatio;
                    SetForwardThrustAndResetBackThrusts(forwardThrustRatio);
                    _thrustController.DampenAllDirections(currentVelocity, _gridMass, THRUST_UPS);
                }
                else if (update10 || stageChanged)
                {
                    _thrustController.ResetThrustOverrides();
                }

                if (currentSpeed < 0.05)
                {
                    Stage = CruiseStage.Terminated;
                    Terminate("Cruise terminated due to target overshoot");
                    return;
                }
            }

            if (Stage == CruiseStage.Complete)
            {
                Terminate(_targetDist < TARGET_REACHED_DISTANCE ? "Destination Reached" : "Terminated");
            }
        }

        private void SetForwardThrustAndResetBackThrusts(float forwardThrustRatio)
        {
            var forwardThrusters = _thrustController.Thrusters[Direction.Forward];
            for (int i = forwardThrusters.Count - 1; i >= 0; i--)
            {
                forwardThrusters[i].ThrustOverridePercentage = forwardThrustRatio;
            }
            var backThrusters = _thrustController.Thrusters[Direction.Backward];
            for (int i = backThrusters.Count - 1; i >= 0; i--)
            {
                backThrusters[i].ThrustOverridePercentage = 0;
            }
        }

        private void UpdateThrustAndAccel()
        {
            _thrustController.UpdateThrusts();
            _forwardAccelPremult = (float)(_thrustController.GetThrustInDirection(Direction.Forward) / _gridMass) * MaxThrustRatio;
        }

        private void ResetBackThrusts()
        {
            var backThrusts = _thrustController.Thrusters[Direction.Backward];
            for (int i = backThrusts.Count - 1; i >= 0; i--)
                backThrusts[i].ThrustOverridePercentage = 0;
        }

        private void SetDampenerState(bool enabled) => ShipController.DampenersOverride = enabled;

        private void OnStageChanged()
        {
            _thrustController.ResetThrustOverrides();
            ResetGyroOverride();
            SetDampenerState(false);

            // reset stage-specific variables
            _lastForwardSpeedDuringAccel = 0;
            _lastForwardThrustRatioDuringAccel = 0;

            if (savePersistentData)
            {
                _config.PersistStateData = $"{NavModeEnum.Cruise}|{DesiredSpeed}|{Stage}";
                _program.Me.CustomData = _config.ToString();
            }
        }

        public void Terminate(string reason)
        {
            if (ShipController.GetShipSpeed() < TARGET_REACHED_SPEED)
            {
                ShipController.DampenersOverride = true;
            }

            _thrustController.ResetThrustOverrides();
            ResetGyroOverride();
            CruiseTerminated.Invoke(this, reason);
        }

        public void Abort()
        {
            Stage = CruiseStage.Aborted;
            Terminate("Aborted");
        }

        protected override void OnNoFunctionalGyrosLeft() => Terminate("No functional gyros found");
    }
}
