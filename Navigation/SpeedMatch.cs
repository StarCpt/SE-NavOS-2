using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using VRage;
using VRageMath;

namespace IngameScript
{
    internal class SpeedMatch : ICruiseController
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

        private VariableThrustController thrustController;
        private int counter = 0;
        private Dictionary<MyDetectedEntityInfo, float> threats = new Dictionary<MyDetectedEntityInfo, float>();
        private List<MyDetectedEntityInfo> obstructions = new List<MyDetectedEntityInfo>();
        private Vector3D relativeVelocity;
        private float gridMass;

        private MyDetectedEntityInfo? target;
        private TargetAcquisitionMode targetInfoMode;

        public SpeedMatch(
            long targetEntityId,
            WcPbApi wcApi,
            IMyShipController shipController,
            IMyTerminalBlock programmableBlock,
            VariableThrustController thrustController)
        {
            this.targetEntityId = targetEntityId;
            this.wcApi = wcApi;
            this.ShipController = shipController;
            this.pb = programmableBlock;
            this.thrustController = thrustController;
            this.gridMass = ShipController.CalculateShipMass().PhysicalMass;
        }

        public void AppendStatus(StringBuilder strb)
        {
            strb.AppendLine("-- SpeedMatch Status --");
            strb.Append("Target: ").Append(targetEntityId);
            if (target.HasValue)
            {
                strb.Append("\nName: ").Append(target.Value.Name);
                strb.Append("\nRelativeVelocity: ").AppendLine(relativeVelocity.Length().ToString("0.0"));
                strb.Append("\nMode: ").AppendLine(targetInfoMode.ToString());
                //maybe add some more info about the target?
            }
        }

        private bool TryGetTarget(out MyDetectedEntityInfo? target, bool counter30)
        {
            target = null;

            try
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
                relativeVelocity = target.Value.Velocity - ShipController.GetShipVelocities().LinearVelocity;
                Vector3 relativeVelocityLocal = Vector3D.TransformNormal(relativeVelocity, MatrixD.Transpose(ShipController.WorldMatrix));
                Vector3 thrustAmount = -relativeVelocityLocal * 10 * gridMass;

                Vector3 input = ShipController.MoveIndicator;
                thrustAmount = new Vector3D(
                    Math.Abs(input.X) <= 0.01 ? thrustAmount.X : 0,
                    Math.Abs(input.Y) <= 0.01 ? thrustAmount.Y : 0,
                    Math.Abs(input.Z) <= 0.01 ? thrustAmount.Z : 0);

                thrustController.SetThrusts(thrustAmount);
            }
        }

        public void Abort() => Terminate("Aborted");
        public void Terminate(string reason)
        {
            thrustController.ResetThrustOverrides();
            CruiseTerminated.Invoke(this, reason);
        }
    }
}
