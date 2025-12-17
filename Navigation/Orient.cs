using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace IngameScript
{
    public class Orient : OrientControllerBase, ICruiseController
    {
        public event CruiseTerminateEventDelegate CruiseTerminated = delegate { };

        public string Name => GetType().Name;
        public Vector3D OrientTarget { get; set; }

        public Orient(IAimController aimControl, IMyShipController controller, IList<IMyGyro> gyros, Vector3D target)
            : base(aimControl, controller, gyros)
        {
            this.OrientTarget = target;
        }

        public void AppendStatus(StringBuilder strb) { }

        public virtual void Run()
        {
            Orient(OrientTarget - ShipController.GetPosition());
        }

        public void Terminate(string reason)
        {
            ResetGyroOverride();
            CruiseTerminated.Invoke(this, reason);
        }

        public void Abort() => Terminate("Aborted");
        protected override void OnNoFunctionalGyrosLeft() => Terminate("No functional gyros found");
    }
}
