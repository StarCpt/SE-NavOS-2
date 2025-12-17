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
        private struct State
        {
            public Vector3D? Target;
            public float MaxSpeed;
        }

        private const double TARGET_REACHED_DIST = 0.05;
        private const double TARGET_REACHED_SPEED = 0.01;

        public event CruiseTerminateEventDelegate CruiseTerminated;

        public string Name => nameof(Autopilot);
        public Vector3D? Target
        {
            get { return _state.Target; }
            set { _state.Target = value; }
        }
        public float MaxSpeed
        {
            get { return _state.MaxSpeed; }
            set { _state.MaxSpeed = value; }
        }
        public float MaxThrustRatio
        {
            get { return _thrustController.MaxThrustRatio; }
            set { _thrustController.MaxThrustRatio = value; }
        }

        private State _state;
        // forward, backward, left, right, up, down
        private double[] _thrustDirections = new double[6];
        private float _shipMass;
        private Vector3D _gravity;

        private readonly IMyShipController _shipController;
        private readonly VariableThrustController _thrustController;
        private readonly List<IMyGyro> _gyros;

        public Autopilot(IMyShipController shipController, VariableThrustController thrustController, List<IMyGyro> gyros)
        {
            _shipController = shipController;
            _thrustController = thrustController;
            _gyros = gyros;
        }

        int counter = -1;

        public void Run()
        {
            counter++;

            State state = _state;
            if (!state.Target.HasValue)
            {
                Terminate("Target is null");
                return;
            }

            if (counter % 10 == 0)
            {
                UpdateLocalThrusts();

                // update mass
                _shipMass = _shipController.CalculateShipMass().PhysicalMass;

                // update gravity
                _gravity = _shipController.GetNaturalGravity();

                // turn off dampeners
                _shipController.DampenersOverride = false;
            }

            if (counter % (60 / UPS) != 0)
            {
                return;
            }

            Vector3D currentPos = _shipController.WorldAABB.Center;
            Vector3D currentVelocity = _shipController.GetShipVelocities().LinearVelocity;
            Vector3D displacement = state.Target.Value - currentPos;

            MatrixD transposedShipControllerWorldMatrix = MatrixD.Transpose(_shipController.WorldMatrix);
            Vector3D localVelocity;
            Vector3D localGravity;
            Vector3D localDisplacement; // local relative pos, we're at 0,0,0
            Vector3D.TransformNormal(ref currentVelocity, ref transposedShipControllerWorldMatrix, out localVelocity);
            Vector3D.TransformNormal(ref _gravity, ref transposedShipControllerWorldMatrix, out localGravity);
            Vector3D.TransformNormal(ref displacement, ref transposedShipControllerWorldMatrix, out localDisplacement);

            bool targetReachedDist = displacement.LengthSquared() <= (TARGET_REACHED_DIST * TARGET_REACHED_DIST);
            bool targetReachedSpeed = currentVelocity.LengthSquared() <= (TARGET_REACHED_SPEED * TARGET_REACHED_SPEED);
            if (targetReachedDist && targetReachedSpeed)
            {
                _shipController.DampenersOverride = true;
                Terminate("Target reached");
                return;
            }
            else if (targetReachedDist)
            {
                // come to a stop asap
                DampenAllDirectionsFast(_thrustController.Thrusters, localVelocity + (localGravity * TIME_STEP), _shipMass, _thrustDirections);
                return; 
            }

            Dictionary<Direction, List<IMyThrust>> thrusters = _thrustController.Thrusters;
            MovePerAxis(localVelocity.X, localDisplacement.X, _shipMass, thrusters[Direction.Right],    thrusters[Direction.Left],    _thrustDirections[(int)Direction.Right],    _thrustDirections[(int)Direction.Left],    localGravity.X);
            MovePerAxis(localVelocity.Y, localDisplacement.Y, _shipMass, thrusters[Direction.Up],       thrusters[Direction.Down],    _thrustDirections[(int)Direction.Up],       _thrustDirections[(int)Direction.Down],    localGravity.Y);
            MovePerAxis(localVelocity.Z, localDisplacement.Z, _shipMass, thrusters[Direction.Backward], thrusters[Direction.Forward], _thrustDirections[(int)Direction.Backward], _thrustDirections[(int)Direction.Forward], localGravity.Z);
            //return;

            double invShipMass = 1.0 / _shipMass;
            double accelRight    = invShipMass * _thrustDirections[(int)Direction.Right];
            double accelLeft     = invShipMass * _thrustDirections[(int)Direction.Left];
            double accelUp       = invShipMass * _thrustDirections[(int)Direction.Up];
            double accelDown     = invShipMass * _thrustDirections[(int)Direction.Down];
            double accelBackward = invShipMass * _thrustDirections[(int)Direction.Backward];
            double accelForward  = invShipMass * _thrustDirections[(int)Direction.Forward];
            Vector3D localAccel = new Vector3D // accel toward target
            {
                X = localDisplacement.X > 0 ? accelRight    : accelLeft,
                Y = localDisplacement.Y > 0 ? accelUp       : accelDown,
                Z = localDisplacement.Z > 0 ? accelBackward : accelForward,
            };
            Vector3D localDecel = new Vector3D // accel away from target
            {
                X = localDisplacement.X > 0 ? accelLeft    : accelRight,
                Y = localDisplacement.Y > 0 ? accelDown    : accelUp,
                Z = localDisplacement.Z > 0 ? accelForward : accelBackward,
            };
            
            Vector3 timeToDecel = new Vector3
            {
                X = (float)ComputeTimeToDecel(Math.Sign(localDisplacement.X) * localVelocity.X, Math.Abs(localDisplacement.X), localAccel.X, localDecel.X),
                Y = (float)ComputeTimeToDecel(Math.Sign(localDisplacement.Y) * localVelocity.Y, Math.Abs(localDisplacement.Y), localAccel.Y, localDecel.Y),
                Z = (float)ComputeTimeToDecel(Math.Sign(localDisplacement.Z) * localVelocity.Z, Math.Abs(localDisplacement.Z), localAccel.Z, localDecel.Z),
            };

            Vector3 ticksToDecel = timeToDecel * UPS;

            Program.optionalInfo =
                $"DisplacementX {localDisplacement.X:0.0000}\n" +
                $"DisplacementY {localDisplacement.Y:0.0000}\n" +
                $"DisplacementZ {localDisplacement.Z:0.0000}\n" +
                $"\n{optionalInfo2}";
            optionalInfo2.Clear();
        }

        private static void DampenAllDirectionsFast(Dictionary<Direction, List<IMyThrust>> thrusters, Vector3D localVelocity, double shipMass, double[] thrustDirections)
        {
            SetThrustRatio(thrusters[Direction.Right],    (float)(-localVelocity.X * shipMass / thrustDirections[(int)Direction.Right]    * UPS));
            SetThrustRatio(thrusters[Direction.Left],     (float)( localVelocity.X * shipMass / thrustDirections[(int)Direction.Left]     * UPS));
            SetThrustRatio(thrusters[Direction.Up],       (float)(-localVelocity.Y * shipMass / thrustDirections[(int)Direction.Up]       * UPS));
            SetThrustRatio(thrusters[Direction.Down],     (float)( localVelocity.Y * shipMass / thrustDirections[(int)Direction.Down]     * UPS));
            SetThrustRatio(thrusters[Direction.Backward], (float)(-localVelocity.Z * shipMass / thrustDirections[(int)Direction.Backward] * UPS));
            SetThrustRatio(thrusters[Direction.Forward],  (float)( localVelocity.Z * shipMass / thrustDirections[(int)Direction.Forward]  * UPS));
        }

        static StringBuilder optionalInfo2 = new StringBuilder();

        // returns: on target
        private static void MovePerAxis(double velocity, double displacement, double shipMass, List<IMyThrust> approachThrusters, List<IMyThrust> stoppingThrusters, double approachThrust, double stoppingThrust, double gravity)
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

            optionalInfo2.AppendLine($"velocity: {velocity:0.0000}");
            optionalInfo2.AppendLine($"displacement: {displacement:0.0000}");
            optionalInfo2.AppendLine($"approachAccel: {approachAccel:0.0000}");
            optionalInfo2.AppendLine($"stoppingAccel: {stoppingAccel:0.0000}");

            double accelMulti;
            double decelInTimeSteps; // start decel in x seconds. debug use only
            bool shouldAccel = ComputeMaxAxisAccel(velocity, displacement, approachAccel, stoppingAccel, out accelMulti, out decelInTimeSteps);

            optionalInfo2.AppendLine($"accelMulti: {accelMulti:0.0000}");
            optionalInfo2.AppendLine($"decelInTicks: {decelInTimeSteps:0.00}");

            double totalAccelRatio = 0;
            double totalDecelRatio = 0;
            if (shouldAccel)
            {
                totalAccelRatio += accelMulti;
                optionalInfo2.AppendLine("accel");
            }
            else if (velocity <= 0)
            {
                // kill speed
                // this branch should rarely be taken

                double accelRatio = (-velocity * shipMass / approachThrust) * UPS;
                totalAccelRatio += accelRatio;
                optionalInfo2.AppendLine("decel (forced)");
            }
            else
            {
                double desiredStopDist = displacement;
                double desiredStopAccel = (velocity * velocity) / (desiredStopDist * 2);
                double desiredStopTime = velocity / desiredStopAccel;

                // if desiredStopTime < actualStopTime, it's not good
                // it means we need to brake faster than we can which is impossible

                double decelRatio;
                double targetDecelTime = desiredStopTime - TIME_STEP;
                if (targetDecelTime <= 0)
                {
                    decelRatio = (velocity * shipMass / stoppingThrust) * UPS;
                    optionalInfo2.AppendLine("decel (full)");
                }
                else
                {
                    double targetDecel = velocity / targetDecelTime;
                    decelRatio = targetDecel / stoppingAccel;
                    optionalInfo2.AppendLine("decel (smooth)");
                }

                totalDecelRatio += decelRatio;
            }

            totalAccelRatio = MathHelper.Saturate(totalAccelRatio);
            totalDecelRatio = MathHelper.Saturate(totalDecelRatio);

            totalAccelRatio += MathHelper.Saturate(-gravity / approachAccel);
            totalDecelRatio += MathHelper.Saturate( gravity / stoppingAccel);

            //totalAccelRatio = MathHelper.Saturate(totalAccelRatio);
            //totalDecelRatio = MathHelper.Saturate(totalDecelRatio);

            double minRatio = Math.Min(totalAccelRatio, totalDecelRatio);
            totalAccelRatio -= minRatio;
            totalDecelRatio -= minRatio;

            optionalInfo2.AppendLine($"g: {gravity:0.0000}");

            SetThrustRatio(approachThrusters, (float)totalAccelRatio);
            SetThrustRatio(stoppingThrusters, (float)totalDecelRatio);

            optionalInfo2.AppendLine();
        }

        public static bool ComputeMaxAxisAccel(double velocity, double displacement, double accel, double decel, out double bestAccelMulti, out double bestDecelInTimeSteps)
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

                bool valid = decelInSeconds * UPS > lookAhead;
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
            bestDecelInTimeSteps *= UPS;
            return bestAccelMulti != 0;
        }

        const float UPS = 6; // must cleanly divide 60
        const float TIME_STEP = 1f / UPS;

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

        private void UpdateLocalThrusts()
        {
            for (int dir = 5; dir >= 0; dir--)
            {
                var thrusters = _thrustController.Thrusters[(Direction)dir];
                double total = 0;
                for (int i = thrusters.Count - 1; i >= 0; i--)
                {
                    var thruster = thrusters[i];
                    total += thruster.IsWorking ? thruster.MaxEffectiveThrust : 0;
                }
                _thrustDirections[dir] = total;
            }
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
