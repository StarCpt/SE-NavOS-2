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
        public float MaxThrustRatio
        {
            get { return _thrustController.MaxThrustRatio; }
            set { _thrustController.MaxThrustRatio = value; }
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
                _shipController.DampenersOverride = false;
            }

            const int UPS = 6; // must cleanly divide 60

            if (counter % (60 / UPS) != 0)
            {
                return;
            }

            if (RunStateless(_shipController, _thrustController, _target.Value, _maxSpeed, UPS, _shipMass, _gravity))
            {
                _shipController.DampenersOverride = true;
                Terminate("Target reached");
                return;
            }

            //Program.optionalInfo =
            //    $"DisplacementX {localDisplacement.X:0.0000}\n" +
            //    $"DisplacementY {localDisplacement.Y:0.0000}\n" +
            //    $"DisplacementZ {localDisplacement.Z:0.0000}\n" +
            //    $"\n{optionalInfo2}";
            //optionalInfo2.Clear();
        }

        public static bool RunStateless(
            IMyShipController shipController, VariableThrustController thrustController,
            Vector3D target, float maxSpeed, float ups, float shipMass, Vector3D naturalGravity)
        {
            thrustController.UpdateThrusts();
            shipController.DampenersOverride = false;

            Vector3D currentVelocity = shipController.GetShipVelocities().LinearVelocity;
            Vector3D displacement = target - shipController.WorldAABB.Center;

            bool targetReachedDist = displacement.LengthSquared() <= (TARGET_REACHED_DIST * TARGET_REACHED_DIST);
            bool targetReachedSpeed = currentVelocity.LengthSquared() <= (TARGET_REACHED_SPEED * TARGET_REACHED_SPEED);
            if (targetReachedDist && targetReachedSpeed)
            {
                shipController.DampenersOverride = true;
                return true;
            }
            else if (targetReachedDist)
            {
                // come to a stop asap
                thrustController.DampenAllDirections(currentVelocity + (naturalGravity / ups), shipMass, 0, ups);
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

            Dictionary<Direction, List<IMyThrust>> thrusters = thrustController.Thrusters;
            MovePerAxis(localVelocity.X, localDisplacement.X, shipMass, thrusters[Direction.Right],    thrusters[Direction.Left],    thrustController.GetThrustInDirection(Direction.Right),    thrustController.GetThrustInDirection(Direction.Left),    localGravity.X, maxLocalClosingVelocity.X, ups);
            MovePerAxis(localVelocity.Y, localDisplacement.Y, shipMass, thrusters[Direction.Up],       thrusters[Direction.Down],    thrustController.GetThrustInDirection(Direction.Up),       thrustController.GetThrustInDirection(Direction.Down),    localGravity.Y, maxLocalClosingVelocity.Y, ups);
            MovePerAxis(localVelocity.Z, localDisplacement.Z, shipMass, thrusters[Direction.Backward], thrusters[Direction.Forward], thrustController.GetThrustInDirection(Direction.Backward), thrustController.GetThrustInDirection(Direction.Forward), localGravity.Z, maxLocalClosingVelocity.Z, ups);

            return false;
        }

        //private static void DampenAllDirectionsFast(Dictionary<Direction, List<IMyThrust>> thrusters, Vector3D localVelocity, double shipMass, double[] thrustDirections)
        //{
        //    SetThrustRatio(thrusters[Direction.Right],    (float)(-localVelocity.X * shipMass / thrustDirections[(int)Direction.Right]    * UPS));
        //    SetThrustRatio(thrusters[Direction.Left],     (float)( localVelocity.X * shipMass / thrustDirections[(int)Direction.Left]     * UPS));
        //    SetThrustRatio(thrusters[Direction.Up],       (float)(-localVelocity.Y * shipMass / thrustDirections[(int)Direction.Up]       * UPS));
        //    SetThrustRatio(thrusters[Direction.Down],     (float)( localVelocity.Y * shipMass / thrustDirections[(int)Direction.Down]     * UPS));
        //    SetThrustRatio(thrusters[Direction.Backward], (float)(-localVelocity.Z * shipMass / thrustDirections[(int)Direction.Backward] * UPS));
        //    SetThrustRatio(thrusters[Direction.Forward],  (float)( localVelocity.Z * shipMass / thrustDirections[(int)Direction.Forward]  * UPS));
        //}

        //static StringBuilder optionalInfo2 = new StringBuilder();

        // returns: on target
        private static void MovePerAxis(
            double velocity, double displacement, double shipMass,
            List<IMyThrust> approachThrusters, List<IMyThrust> stoppingThrusters,
            double approachThrust, double stoppingThrust,
            double gravity, double maxClosingVelocity, double ups)
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
            }

            double approachAccel = approachThrust / shipMass;
            double stoppingAccel = stoppingThrust / shipMass;

            double accelMulti;
            double decelInTimeSteps; // start decel in x seconds. debug use only
            bool shouldAccel = ComputeMaxAxisAccel(velocity, displacement, approachAccel + gravity, stoppingAccel - gravity, ups, out accelMulti, out decelInTimeSteps);

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
                if (targetDecelTime <= 0)
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

            totalAccelRatio = MathHelper.Saturate(totalAccelRatio);
            totalDecelRatio = MathHelper.Saturate(totalDecelRatio);

            totalAccelRatio += MathHelper.Saturate(-gravity / approachAccel);
            totalDecelRatio += MathHelper.Saturate( gravity / stoppingAccel);

            //totalAccelRatio = MathHelper.Saturate(totalAccelRatio);
            //totalDecelRatio = MathHelper.Saturate(totalDecelRatio);

            // only accelRatio or decelRatio may be nonzero per axis, both may be zero but not nonzero.
            double minRatio = Math.Min(totalAccelRatio, totalDecelRatio);
            totalAccelRatio -= minRatio;
            totalDecelRatio -= minRatio;

            double nextVelocity = velocity + (totalAccelRatio * approachAccel - totalDecelRatio * stoppingAccel) * timeStep;
            if (nextVelocity > maxClosingVelocity)
            {
                double excessVelocity = nextVelocity - maxClosingVelocity;
                totalAccelRatio -= (excessVelocity * ups) / approachAccel;
                totalDecelRatio += -totalAccelRatio;
            }

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
            // assume accel and decel are >= 0
            if (accel == 0 || decel == 0)
            {
                return double.PositiveInfinity;
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

        public void AppendStatus(StringBuilder strb)
        {

        }

        public void Abort() => Terminate("Aborted");

        public void Terminate(string reason)
        {
            _thrustController.ResetThrustOverrides();
            _shipController.DampenersOverride = true;
            CruiseTerminated.Invoke(this, reason);
        }
    }
}
