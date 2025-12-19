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
        public float MaxThrustRatio
        {
            get { return _maxThrustOverrideRatio; }
            set
            {
                value = MathHelper.Saturate(value);
                if (_maxThrustOverrideRatio != value)
                {
                    _maxThrustOverrideRatio = value;
                    UpdateThrusts();
                }
            }
        }

        public Dictionary<Direction, List<IMyThrust>> Thrusters { get; }

        private float _maxThrustOverrideRatio = 1f;
        private IMyShipController shipController;

        private double rightThrust, leftThrust, upThrust, downThrust, backThrust, forwardThrust;
        private float rightThrustInv, leftThrustInv, upThrustInv, downThrustInv, backThrustInv, forwardThrustInv;

        public VariableThrustController(Dictionary<Direction, List<IMyThrust>> thrusters, IMyShipController shipController)
        {
            this.Thrusters = thrusters;
            this.shipController = shipController;
        }

        public double GetThrustInDirection(Direction direction)
        {
            switch (direction)
            {
                case Direction.Right: return rightThrust;
                case Direction.Left: return leftThrust;
                case Direction.Up: return upThrust;
                case Direction.Down: return downThrust;
                case Direction.Backward: return backThrust;
                case Direction.Forward: return forwardThrust;
                default: throw new ArgumentException("Invalid direction.");
            }
        }

        public void UpdateThrusts()
        {
            for (Direction dir = 0; dir < Direction.MAX_COUNT; dir++)
            {
                var thrusters = Thrusters[dir];
                double total = 0;
                for (int i = thrusters.Count - 1; i >= 0; i--)
                {
                    var thruster = thrusters[i];
                    if (thruster.IsWorking)
                    {
                        total += thruster.MaxEffectiveThrust;
                    }
                }
                switch (dir)
                {
                    case Direction.Forward:
                        forwardThrust = total;
                        forwardThrustInv = (float)(1.0 / total);
                        break;
                    case Direction.Backward:
                        backThrust = total;
                        backThrustInv = (float)(1.0 / total);
                        break;
                    case Direction.Left:
                        leftThrust = total;
                        leftThrustInv = (float)(1.0 / total);
                        break;
                    case Direction.Right:
                        rightThrust = total;
                        rightThrustInv = (float)(1.0 / total);
                        break;
                    case Direction.Up:
                        upThrust = total;
                        upThrustInv = (float)(1.0 / total);
                        break;
                    case Direction.Down:
                        downThrust = total;
                        downThrustInv = (float)(1.0 / total);
                        break;
                }
            }
        }

        public void DampenAllDirections(Vector3D shipVelocity, float gridMass, float tolerance, float ups = 1)
        {
            Vector3 localVelocity = Vector3D.TransformNormal(shipVelocity, MatrixD.Transpose(shipController.WorldMatrix));
            Vector3 thrustAmount = localVelocity * gridMass * ups;
            SetThrusts(thrustAmount, tolerance);
        }

        public void SetThrusts(Vector3 thrustAmount, float tolerance)
        {
            float backward = thrustAmount.Z < tolerance ? -thrustAmount.Z : 0;
            float forward = thrustAmount.Z > tolerance ? thrustAmount.Z : 0;
            float right = thrustAmount.X < tolerance ? -thrustAmount.X : 0;
            float left = thrustAmount.X > tolerance ? thrustAmount.X : 0;
            float up = thrustAmount.Y < tolerance ? -thrustAmount.Y : 0;
            float down = thrustAmount.Y > tolerance ? thrustAmount.Y : 0;

            SetSideThrusts(left, right, up, down);

            backward *= backThrustInv;
            forward = Math.Min(forward * forwardThrustInv, MaxThrustRatio);

            foreach (var thrust in Thrusters[Direction.Forward]) thrust.ThrustOverridePercentage = forward;
            foreach (var thrust in Thrusters[Direction.Backward]) thrust.ThrustOverridePercentage = backward;
        }

        public void SetSideThrusts(float left, float right, float up, float down)
        {
            left *= leftThrustInv;
            right *= rightThrustInv;
            up *= upThrustInv;
            down *= downThrustInv;

            foreach (var thrust in Thrusters[Direction.Left]) thrust.ThrustOverridePercentage = left;
            foreach (var thrust in Thrusters[Direction.Right]) thrust.ThrustOverridePercentage = right;
            foreach (var thrust in Thrusters[Direction.Up]) thrust.ThrustOverridePercentage = up;
            foreach (var thrust in Thrusters[Direction.Down]) thrust.ThrustOverridePercentage = down;
        }

        public void ResetThrustOverrides()
        {
            foreach (var kv in Thrusters)
                for (int i = kv.Value.Count - 1; i >= 0; i--)
                    kv.Value[i].ThrustOverride = 0;
        }
    }
}
