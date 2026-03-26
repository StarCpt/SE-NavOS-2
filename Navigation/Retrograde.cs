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
    public class Retrograde : Orient
    {
        const double TERMINATE_SPEED = 5;
        Vector3D initVelocity = Vector3D.Zero;
        bool _useInitVector = false;

        public override string Name => nameof(Retrograde);

        public Retrograde(IAimController aimControl, IMyShipController controller, IList<IMyGyro> gyros)
            : base(aimControl, controller, gyros)
        {
        }

        public override void Run()
        {
            Vector3D shipVelocity = ShipController.GetShipVelocities().LinearVelocity;
            if (initVelocity == Vector3D.Zero)
                initVelocity = shipVelocity;
            double diff = Math.Round(Vector3D.Dot(shipVelocity.SafeNormalize(), initVelocity.SafeNormalize()), 5);
            if (diff < 0.9999 && !ShipController.DampenersOverride) _useInitVector = true;        

            if (shipVelocity.LengthSquared() <= TERMINATE_SPEED * TERMINATE_SPEED)
            {
                Terminate($"Speed is less than {TERMINATE_SPEED:0.#} m/s");
                return;
            }

            Orient(_useInitVector ? -initVelocity : -shipVelocity);
        }
    }
}
