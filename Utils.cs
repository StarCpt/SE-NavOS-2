using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public static class Utils
    {
        public static Vector3D SafeNormalize(this Vector3D a)
        {
            if (Vector3D.IsZero(a))
                return Vector3D.Zero;
            if (Vector3D.IsUnit(ref a))
                return a;
            return Vector3D.Normalize(a);
        }

        public static StringBuilder AppendTime(this StringBuilder strb, double totalSeconds)
        {
            int minutes = (int)totalSeconds / 60;
            totalSeconds %= 60;
            strb.Append(minutes).Append(":").Append(totalSeconds.ToString("00.0"));
            return strb;
        }

        public static string MinuteAndSeconds(double totalSeconds)
        {
            return $"{(int)totalSeconds / 60}:{totalSeconds % 60:00.0}";
        }

        public static Vector3D Normalize(ref Vector3D vec, out double length)
        {
            length = vec.Length();
            return length < 0.00001 ? Vector3D.Zero : (vec / length);
        }

        public static double GetWorldMaxSpeed(this Program program)
        {
            return program.Me.CubeGrid.GridSizeEnum == MyCubeSize.Large ? program.World.LargeShipMaxSpeed : program.World.SmallShipMaxSpeed;
        }
    }
}
