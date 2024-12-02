using Sandbox.Game.Entities;
using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    internal class Range : ICruiseController
    {

        private enum TargetAcquisitionMode
        {
            None = 0,
            AiFocus = 1,
            SortedThreat = 2,
            Obstruction = 3,
        }

        public event CruiseTerminateEventDelegate CruiseTerminated;

        public string Name => nameof(SpeedMatch);
        public IMyShipController ShipController { get; set; }

        public double relativeSpeedThreshold = 0.01;//stop dampening under this relative speed

        private long targetEntityId;
        private WcPbApi wcApi;
        private IMyTerminalBlock pb;

        private IVariableThrustController thrustController;
        private int counter = 0;
        private Dictionary<MyDetectedEntityInfo, float> threats = new Dictionary<MyDetectedEntityInfo, float>();
        private List<MyDetectedEntityInfo> obstructions = new List<MyDetectedEntityInfo>();
        private Vector3D relativeVelocity;
        private float gridMass;
        private double dist;
        private double targetDistance;

        private MyDetectedEntityInfo? target;
        private TargetAcquisitionMode targetInfoMode;

        public Range(
            long targetEntityId,
            WcPbApi wcApi,
            IMyShipController shipController,
            IMyTerminalBlock programmableBlock,
            IVariableThrustController thrustController,
            double dist
            )
        {
            this.targetEntityId = targetEntityId;
            this.wcApi = wcApi;
            this.ShipController = shipController;
            this.pb = programmableBlock;
            this.thrustController = thrustController;
            this.gridMass = ShipController.CalculateShipMass().PhysicalMass;
            this.dist = dist;
        }

        public void AppendStatus(StringBuilder strb)
        {
            strb.AppendLine("-- Range Status --");
            strb.Append("Target: ").Append(targetEntityId);
            if (target.HasValue)
            {
                strb.Append("\nName: ").Append(target.Value.Name);
                strb.Append("\nRelativeVelocity: ").AppendLine(relativeVelocity.Length().ToString("0.0"));
                strb.Append("\nRange: ").AppendLine(this.dist.ToString("0.0"));
                strb.Append("\nDistance: ").AppendLine(targetDistance.ToString("0.0"));
                strb.Append("\nMode: ").AppendLine(targetInfoMode.ToString());
                //maybe add some more info about the target?
            }
        }

        private bool TryGetTarget(out MyDetectedEntityInfo? target, bool counter30)
        {
            target = null;

            try
            {
                if(targetEntityId != -1)
                {
                    //support changing main target after running speedmatch
                    var aifocus = wcApi.GetAiFocus(pb.EntityId);
                    if (aifocus?.EntityId == targetEntityId)
                    {
                        target = aifocus.Value;
                        targetInfoMode = TargetAcquisitionMode.AiFocus;
                        return true;
                    }
                    else
                    {
                        MyDetectedEntityInfo? ent = null;
                        wcApi.GetSortedThreats(pb, threats);
                        foreach (var threat in threats.Keys)
                        {
                            if (threat.EntityId == targetEntityId)
                            {
                                ent = threat;
                                targetInfoMode = TargetAcquisitionMode.SortedThreat;
                                break;
                            }
                        }

                        threats.Clear();

                        if (ent.HasValue)
                        {
                            target = ent.Value;
                            return true;
                        }
                    }

                    if (counter30)
                    {
                        MyDetectedEntityInfo? ent = null;
                        //if neither methods found the target try looking thru obstructions
                        wcApi.GetObstructions(pb, obstructions);
                        foreach (var obs in obstructions)
                        {
                            if (obs.EntityId == targetEntityId)
                            {
                                ent = obs;
                                targetInfoMode = TargetAcquisitionMode.Obstruction;
                                break;
                            }
                        }


                        if (ent.HasValue)
                        {
                            target = ent.Value;
                            return true;
                        }
                    }

                    targetInfoMode = TargetAcquisitionMode.None;
                    return false;
                } else
                {
                    wcApi.GetSortedThreats(pb, threats);
                    if (threats.Keys.Count == 0)
                    {
                        return false;
                    }

                    target = threats.Keys.FirstOrDefault();

                    threats.Clear();

                    return true;
                }
                
            }
            catch
            {
                return false;
            }
        }

        public void Run()
        {
            counter++;
            bool counter10 = counter % 10 == 0;
            bool counter30 = counter % 30 == 0;

            if (counter10)
            {
                ShipController.DampenersOverride = false;

                target = null;

                if (!TryGetTarget(out target, counter30))
                {
                    thrustController.ResetThrustOverrides();
                    return;
                }

                thrustController.UpdateThrusts();
            }

            if (counter30 && counter % 60 == 0)
            {
                gridMass = ShipController.CalculateShipMass().PhysicalMass;
            }

            if (target.HasValue && counter % 5 == 0)
            {
                Vector3D targetPosition = target.Value.Position;
                Vector3D currentPosition = ShipController.GetPosition();
                Vector3D currentVelocity = ShipController.GetShipVelocities().LinearVelocity;

                relativeVelocity = target.Value.Velocity - currentVelocity;

                Vector3D targetDirection = (currentPosition - targetPosition).SafeNormalize();
                targetDistance = Vector3D.Distance(currentPosition, targetPosition);    

                double effectiveThrust = CalculateEffectiveThrust(targetDirection, 1) > 0 ? CalculateEffectiveThrust(targetDirection, 1) : 100000;
                double effectiveThrustNeg = CalculateEffectiveThrust(targetDirection, -1) > 0 ? CalculateEffectiveThrust(targetDirection, -1) : 100000;
                double stoppingDistance = (relativeVelocity.LengthSquared()) / (effectiveThrustNeg / gridMass);

                if (Math.Abs(dist - targetDistance) > Math.Max(stoppingDistance, 5))
                {
                    Vector3D fakeVelocity = targetDirection * (dist - targetDistance) * Math.Abs((dist - targetDistance) / Math.Max(dist, targetDistance));
                    relativeVelocity += fakeVelocity;
                }

                Vector3 relativeVelocityLocal = Vector3D.TransformNormal(relativeVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
                
                Vector3 thrustAmount = -relativeVelocityLocal * 10 * gridMass;

                Vector3 input = ShipController.MoveIndicator;
                thrustAmount = new Vector3D(
                    Math.Abs(input.X) <= 0.01 ? thrustAmount.X : 0,
                    Math.Abs(input.Y) <= 0.01 ? thrustAmount.Y : 0,
                    Math.Abs(input.Z) <= 0.01 ? thrustAmount.Z : 0);

                thrustController.SetThrusts(thrustAmount, 0);
            }
        }

        private float CalculateEffectiveThrust(Vector3D direction, int axisDirection)
        {
            float effectiveThrust = 0f;

            // Normalize the current velocity to get the direction of motion
            Vector3D velocityDirection = direction;
            if (!velocityDirection.IsZero())
            {
                velocityDirection.Normalize();
            }

            // Iterate over all thruster directions
            foreach (var kv in thrustController.Thrusters)
            {
                // Iterate over each thruster in this direction
                foreach (var thrust in kv.Value)
                {
                    // Calculate the thrust vector in world space for this thruster
                    Vector3D thrustDirection = thrust.WorldMatrix.Forward;

                    // Calculate the cosine of the angle between the thrust direction and the negative velocity direction
                    double cosAngle = Vector3D.Dot(thrustDirection, -velocityDirection);

                    // Calculate the contribution of this thruster to deceleration
                    float thrustContribution = (float)(thrust.MaxEffectiveThrust * cosAngle);

                    if ((axisDirection > 0 && thrustContribution > 0) || (axisDirection < 0 && thrustContribution < 0))
                    {
                        effectiveThrust += thrustContribution;
                    }
                }
            }

            return Math.Abs(effectiveThrust);
        }

        public void Abort() => Terminate("Aborted");
        public void Terminate(string reason)
        {
            thrustController.ResetThrustOverrides();
            CruiseTerminated.Invoke(this, reason);
        }
    }
}
