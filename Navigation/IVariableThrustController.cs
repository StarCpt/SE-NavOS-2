using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage;

namespace IngameScript
{
    public interface IVariableThrustController : IThrustController
    {
        Dictionary<Direction, List<IMyThrust>> Thrusters { get; }
        float MaxThrustRatio { get; set; }
        void UpdateThrusts();
    }
}
