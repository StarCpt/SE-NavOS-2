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

namespace IngameScript.Navigation
{
    public class Autopilot : ICruiseController
    {
        private struct State
        {
            public Vector3D? Target;
            public float MaxSpeed;
        }

        private const double TARGET_REACHED_DIST = 0.05;
        private const double TARGET_REACHED_SPEED = 0.05;

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

        private readonly IMyShipController _shipController;
        private readonly IVariableThrustController _thrustController;
        private readonly List<IMyGyro> _gyros;

        public Autopilot(IMyShipController shipController, IVariableThrustController thrustController, List<IMyGyro> gyros)
        {
            _shipController = shipController;
            _thrustController = thrustController;
            _gyros = gyros;
        }

        int counter = 0;
        Vector3D lastLocalVelocity = Vector3D.Zero;

        public void Run()
        {
            State state = _state;
            if (!state.Target.HasValue)
            {
                Terminate("Target is null");
                return;
            }

            if (++counter % 10 == 0)
            {
                UpdateLocalThrusts();

                // update mass
                _shipMass = _shipController.CalculateShipMass().PhysicalMass;
            }

            Vector3D currentPos = _shipController.WorldAABB.Center;
            Vector3D currentVelocity = _shipController.GetShipVelocities().LinearVelocity;
            Vector3D displacement = state.Target.Value - currentPos;

            bool targetReachedDist = displacement.LengthSquared() <= (TARGET_REACHED_DIST * TARGET_REACHED_DIST);
            bool targetReachedSpeed = currentVelocity.LengthSquared() <= (TARGET_REACHED_SPEED * TARGET_REACHED_SPEED);
            //if (targetReachedDist && targetReachedSpeed)
            //{
            //    _shipController.DampenersOverride = true;
            //    Terminate("Target reached");
            //    return;
            //}
            //else if (targetReachedDist)
            //{
            //    // come to a stop asap
            //    _thrustController.DampenAllDirections(currentVelocity, _shipMass, 0);
            //    return;
            //}

            MatrixD transposedShipControllerWorldMatrix = MatrixD.Transpose(_shipController.WorldMatrix);
            Vector3D localVelocity;
            Vector3D localDisplacement; // local relative pos, we're at 0,0,0
            Vector3D.TransformNormal(ref currentVelocity, ref transposedShipControllerWorldMatrix, out localVelocity);
            Vector3D.TransformNormal(ref displacement, ref transposedShipControllerWorldMatrix, out localDisplacement);

            Dictionary<Direction, List<IMyThrust>> thrusters = _thrustController.Thrusters;
            MovePerAxis(localVelocity.X, localDisplacement.X, _shipMass, thrusters[Direction.Right], thrusters[Direction.Left], _thrustDirections[(int)Direction.Right], _thrustDirections[(int)Direction.Left]);
            MovePerAxis(localVelocity.Y, localDisplacement.Y, _shipMass, thrusters[Direction.Up], thrusters[Direction.Down], _thrustDirections[(int)Direction.Up], _thrustDirections[(int)Direction.Down]);
            MovePerAxis(localVelocity.Z, localDisplacement.Z, _shipMass, thrusters[Direction.Backward], thrusters[Direction.Forward], _thrustDirections[(int)Direction.Backward], _thrustDirections[(int)Direction.Forward]);
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
            
            bool xClosing = (localVelocity.X * localDisplacement.X) > 0;
            bool yClosing = (localVelocity.Y * localDisplacement.Y) > 0;
            bool zClosing = (localVelocity.Z * localDisplacement.Z) > 0;

            Vector3 timeToDecel = new Vector3
            {
                X = xClosing ? ComputeTimeToDecel(Math.Abs(localVelocity.X), Math.Abs(localDisplacement.X), localAccel.X, localDecel.X) : float.PositiveInfinity,
                Y = yClosing ? ComputeTimeToDecel(Math.Abs(localVelocity.Y), Math.Abs(localDisplacement.Y), localAccel.Y, localDecel.Y) : float.PositiveInfinity,
                Z = zClosing ? ComputeTimeToDecel(Math.Abs(localVelocity.Z), Math.Abs(localDisplacement.Z), localAccel.Z, localDecel.Z) : float.PositiveInfinity,
            };

            if (counter % 10 != 0)
            {
                return;
            }

            int lookAhead = 1;
            Vector3 ticksToDecel = timeToDecel * UPS;

            Vector3D accel = localVelocity - lastLocalVelocity;
            lastLocalVelocity = localVelocity;

            Program.optionalInfo =
                $"TicksToDecelX {ticksToDecel.X:0.0000}\n" +
                $"TicksToDecelY {ticksToDecel.Y:0.0000}\n" +
                $"TicksToDecelZ {ticksToDecel.Z:0.0000}\n" +
                $"Displacement {localDisplacement.X:0.00}";
        }

        // returns: on target
        private static bool MovePerAxis(double velocity, double displacement, double shipMass, List<IMyThrust> approachThrusters, List<IMyThrust> stoppingThrusters, double approachThrust, double stoppingThrust)
        {
            if (displacement < 0)
            {
                velocity = -velocity;
                displacement = -displacement;

                var tempApproachThrusters = approachThrusters;
                approachThrusters = stoppingThrusters;
                stoppingThrusters = tempApproachThrusters;

                var tempApproachAccel = approachThrust;
                approachThrust = stoppingThrust;
                stoppingThrust = tempApproachAccel;
            }

            double approachAccel = approachThrust / shipMass;
            double stoppingAccel = stoppingThrust / shipMass;

            // apply full accel @ timeStepsToDecel == 2
            // apply full decel @ timeStepsToDecel == 0

            double accelMulti = 1;
            bool found = false;
            for (int i = 0; i < 10; i++)
            {
                float secondsToDecel = ComputeTimeToDecel(velocity, displacement, approachAccel * accelMulti, stoppingAccel); // start decel after this amount of time
                float timeStepsToDecel = secondsToDecel * UPS;

                if (timeStepsToDecel > 1)
                {
                    float accelRatio = MathHelper.Clamp(timeStepsToDecel - 1, 0, 1);
                    accelRatio = (float)(Math.Min(1, accelRatio) * accelMulti);
                    SetThrustRatio(approachThrusters, accelRatio);
                    SetThrustRatio(stoppingThrusters, 0);
                    found = true;
                    break;
                }

                accelMulti /= 2;
            }

            if (!found)
            {
                // kill speed
                float accelRatio = (float)(-velocity * shipMass / approachThrust) * UPS;
                float decelRatio = (float)(velocity * shipMass / stoppingThrust) * UPS;
                SetThrustRatio(approachThrusters, accelRatio);
                SetThrustRatio(stoppingThrusters, decelRatio);
            }

            return false;
        }

        const int UPS = 6;
        const float TIME_STEP = 1f / UPS;

        private static void SetThrustRatio(List<IMyThrust> thrusters, float percentage)
        {
            // negative values are ok as ThrustOverridePercentage setter clamps it to [0,1]
            for (int i = thrusters.Count - 1; i >= 0; i--)
            {
                thrusters[i].ThrustOverridePercentage = percentage;
            }
        }

        private static float ComputeTimeToDecel(double velocity, double displacement, double accel, double decel)
        {
            // assume accel and decel are >= 0
            if (accel == 0 || decel == 0)
            {
                return float.PositiveInfinity;
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
            return (float)(timeToDecel + initialTimeToStop);
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
            CruiseTerminated.Invoke(this, reason);
        }
    }
}
