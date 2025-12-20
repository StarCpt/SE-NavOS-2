using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using VRage;
using VRageMath;

namespace IngameScript
{
    public class VariableThrustController
    {
        public float MaxForwardThrustRatio
        {
            get { return _maxForwardThrustRatio; }
            set
            {
                value = MathHelper.Saturate(value);
                if (_maxForwardThrustRatio != value)
                {
                    _maxForwardThrustRatio = value;
                    UpdateThrusts();
                }
            }
        }
        public IMyShipController ShipController => _shipController;

        public readonly Dictionary<Direction, List<IMyThrust>> Thrusters;

        private float _maxForwardThrustRatio = 1f;
        private readonly IMyShipController _shipController;
        private readonly List<IMyThrust>[] _thrusters;

        private readonly double[] _thrusts = new double[6];
        private float rightThrustInv, leftThrustInv, upThrustInv, downThrustInv, backThrustInv, forwardThrustInv;

        public VariableThrustController(Dictionary<Direction, List<IMyThrust>> thrusters, IMyShipController shipController)
        {
            this.Thrusters = thrusters;
            this._shipController = shipController;
            _thrusters = new[]
            {
                thrusters[(Direction)0],
                thrusters[(Direction)1],
                thrusters[(Direction)2],
                thrusters[(Direction)3],
                thrusters[(Direction)4],
                thrusters[(Direction)5],
            };
        }

        public double GetThrustInDirection(Direction direction)
        {
            return _thrusts[(int)direction];
        }

        public void UpdateThrusts()
        {
            for (int dir = 5; dir >= 0; dir--)
            {
                var thrusters = _thrusters[dir];
                double total = 0;
                for (int i = thrusters.Count - 1; i >= 0; i--)
                {
                    var thruster = thrusters[i];
                    if (thruster.IsWorking)
                    {
                        total += thruster.MaxEffectiveThrust;
                    }
                }
                _thrusts[dir] = total;
                switch ((Direction)dir)
                {
                    case Direction.Right:    rightThrustInv   = (float)(1.0 / total); break;
                    case Direction.Left:     leftThrustInv    = (float)(1.0 / total); break;
                    case Direction.Up:       upThrustInv      = (float)(1.0 / total); break;
                    case Direction.Down:     downThrustInv    = (float)(1.0 / total); break;
                    case Direction.Forward:  forwardThrustInv = (float)(1.0 / total); break;
                    case Direction.Backward: backThrustInv    = (float)(1.0 / total); break;
                }
            }
        }

        public void DampenAllDirections(Vector3D shipVelocity, float gridMass, float tolerance, float ups = 1)
        {
            Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(_shipController.WorldMatrix));
            SetThrusts(localVelocity * gridMass * ups, tolerance);
        }

        public void SetThrusts(Vector3 thrustAmount, float tolerance)
        {
            float right    = thrustAmount.X < tolerance ? -thrustAmount.X : 0;
            float left     = thrustAmount.X > tolerance ? thrustAmount.X : 0;
            float up       = thrustAmount.Y < tolerance ? -thrustAmount.Y : 0;
            float down     = thrustAmount.Y > tolerance ? thrustAmount.Y : 0;
            float backward = thrustAmount.Z < tolerance ? -thrustAmount.Z : 0;
            float forward  = thrustAmount.Z > tolerance ? thrustAmount.Z : 0;

            SetSideThrusts(left, right, up, down);

            backward *= backThrustInv;
            forward = Math.Min(forward * forwardThrustInv, MaxForwardThrustRatio);

            var backwardThrusters = _thrusters[(int)Direction.Backward];
            for (int i = backwardThrusters.Count - 1; i >= 0; i--)
            {
                backwardThrusters[i].ThrustOverridePercentage = backward;
            }

            var forwardThrusters = _thrusters[(int)Direction.Forward];
            for (int i = forwardThrusters.Count - 1; i >= 0; i--)
            {
               forwardThrusters[i].ThrustOverridePercentage = forward;
            }
        }

        public void SetSideThrusts(float left, float right, float up, float down)
        {
            right *= rightThrustInv;
            left *= leftThrustInv;
            up *= upThrustInv;
            down *= downThrustInv;

            var rightThrusters = _thrusters[(int)Direction.Right];
            for (int i = rightThrusters.Count - 1; i >= 0; i--)
            {
                rightThrusters[i].ThrustOverridePercentage = right;
            }

            var leftThrusters = _thrusters[(int)Direction.Left];
            for (int i = leftThrusters.Count - 1; i >= 0; i--)
            {
                leftThrusters[i].ThrustOverridePercentage = left;
            }

            var upThrusters = _thrusters[(int)Direction.Up];
            for (int i = upThrusters.Count - 1; i >= 0; i--)
            {
                upThrusters[i].ThrustOverridePercentage = up;
            }

            var downThrusters = _thrusters[(int)Direction.Down];
            for (int i = downThrusters.Count - 1; i >= 0; i--)
            {
                downThrusters[i].ThrustOverridePercentage = down;
            }
        }

        public void ResetThrustOverrides()
        {
            for (int dir = 5; dir >= 0; dir--)
            {
                var thrusters = _thrusters[dir];
                for (int i = thrusters.Count - 1; i >= 0; i--)
                {
                    thrusters[i].ThrustOverridePercentage = 0;
                }
            }
        }

        public void TurnOnAllThrusters()
        {
            for (int dir = 5; dir >= 0; dir--)
            {
                var thrusters = _thrusters[dir];
                for (int i = thrusters.Count - 1; i >= 0; i--)
                {
                    thrusters[i].Enabled = true;
                }
            }
        }

        public void SetDampenerState(bool enabled) => _shipController.DampenersOverride = enabled;
    }
}
