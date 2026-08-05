using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Game;
using Sandbox.ModAPI.Ingame;
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
            // MaxSpeedOverride lets commanded speed exceed the world limit on servers where a mod
            // (e.g. Flip and Burn) moves the grid above it. 0 = use the world limit.
            double over = program.config?.MaxSpeedOverride ?? 0;
            if (over > 0)
                return over;

            return program.Me.CubeGrid.GridSizeEnum == MyCubeSize.Large ? program.World.LargeShipMaxSpeed : program.World.SmallShipMaxSpeed;
        }

        /// <summary>
        /// Ship velocity that stays correct when Flip and Burn is clamping physics velocity.
        /// Falls back to the physics reading at normal speeds, where it is the more accurate of
        /// the two. See <see cref="TrueVelocity"/>.
        /// </summary>
        public static Vector3D GetTrueVelocity(this IMyShipController controller)
        {
            Vector3D physics = controller.GetShipVelocities().LinearVelocity;
            Vector3D tracked = TrueVelocity.Value;

            // Only prefer the tracked value once it clearly exceeds the physics reading, which is
            // the signature of the clamp being applied.
            return tracked.Length() > physics.Length() * 1.05 + 5 ? tracked : physics;
        }
    }
}
