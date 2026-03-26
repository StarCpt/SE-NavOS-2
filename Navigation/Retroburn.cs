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
    public class Retroburn : Orient, ICruiseController
    {
        const double ORIENT_SPEED_THRESHOLD = 5; // don't orient under this speed since it can make the ship turn violently
        const float DAMPENER_TOLERANCE = 0.005f;
        Vector3D initVelocity = Vector3D.Zero;
        bool _useInitVector = false;
        
        public override string Name => nameof(Retroburn);

        private VariableThrustController thrustController;
        private float gridMass;

        public Retroburn(
            IAimController aimControl,
            IMyShipController controller,
            List<IMyGyro> gyros,
            VariableThrustController thrustController)
            : base(aimControl, controller, gyros)
        {
            this.thrustController = thrustController;
        }

        public override void Run()
        {
            if (Program.counter % 30 == 0)
            {
                gridMass = ShipController.CalculateShipMass().PhysicalMass;
                thrustController.UpdateThrusts();
            }

            Vector3D shipVelocity = ShipController.GetShipVelocities().LinearVelocity;
            if (initVelocity == Vector3D.Zero)
                initVelocity = shipVelocity;
            double diff = Math.Round(Vector3D.Dot(shipVelocity.SafeNormalize(), initVelocity.SafeNormalize()), 5);
            if (diff < 0.9999) _useInitVector = true;

            double velocitySq = shipVelocity.LengthSquared();

            Vector3D gravity = ShipController.GetNaturalGravity();

            if (velocitySq > ORIENT_SPEED_THRESHOLD * ORIENT_SPEED_THRESHOLD)
                Orient(_useInitVector ? -initVelocity : -shipVelocity);
            else if (gravity != Vector3D.Zero)
                Orient(-gravity);
            else
                ResetGyroOverride();

            if (Program.counter % 10 == 0)
            {
                ShipController.DampenersOverride = false;

                const float UPS = 6;

                if (Vector3D.Dot((_useInitVector ? -initVelocity : -shipVelocity).SafeNormalize(), ShipController.WorldMatrix.Forward) > 0.9999 || velocitySq <= ORIENT_SPEED_THRESHOLD * ORIENT_SPEED_THRESHOLD)
                    thrustController.DampenAllDirections(shipVelocity, gravity, gridMass, UPS - 1); // UPS - 1 to smooth the decel
                else
                    thrustController.ResetThrustOverrides();
            }

            if (velocitySq <= DAMPENER_TOLERANCE * DAMPENER_TOLERANCE)
            {
                thrustController.ResetThrustOverrides();
                ShipController.DampenersOverride = true;
                Terminate($"Speed is less than {DAMPENER_TOLERANCE} m/s");
                return;
            }
        }
    }
}
