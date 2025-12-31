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
    public class RadialIn : Orient
    {
        public override string Name => nameof(RadialIn);

        private Vector3D _gravity;

        public RadialIn(IAimController aimControl, IMyShipController controller, IList<IMyGyro> gyros)
            : base(aimControl, controller, gyros)
        {
            _gravity = controller.GetNaturalGravity();
        }

        public override void Run()
        {
            if ((Program.counter % 10) == 0)
            {
                _gravity = ShipController.GetNaturalGravity();
            }

            if (_gravity == Vector3D.Zero)
            {
                Terminate("No gravity detected");
            }

            Orient(_gravity);
        }
    }
}
