using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Recovers the ship's real velocity when the Flip and Burn mod is active.
    ///
    /// Above its trigger speed the mod stops using physics to move the grid: it clamps
    /// Grid.Physics.LinearVelocity to the world ship speed limit and repositions the grid with
    /// Grid.Teleport() every tick. GetShipVelocities() therefore reports the clamp - typically
    /// 1000 m/s - while the ship may actually be doing tens of km/s.
    ///
    /// Position is still truthful, because the teleport target is a real world matrix. Differencing
    /// it across ticks recovers the true velocity with no cooperation from the mod. (The mod does
    /// expose the real figure through a ModAPI delegate, but that is unreachable from a
    /// Programmable Block.)
    ///
    /// Note the client/server split: the mod writes the FULL virtual velocity into physics on the
    /// client to keep the speedometer honest, but the server - where this script runs in
    /// multiplayer - sees the clamped value. Anything built on this must be tested on the server,
    /// not in single-player.
    /// </summary>
    public static class TrueVelocity
    {
        /// <summary>Reject samples implying more than this acceleration; they are instance
        /// transfers or teleports, not flight.</summary>
        private const double MaxPlausibleAcceleration = 2000;

        private static Vector3D _lastPosition;
        private static Vector3D _velocity;
        private static bool _havePosition;
        private static bool _haveVelocity;

        /// <summary>Last good position-differenced velocity. Zero until primed.</summary>
        public static Vector3D Value => _velocity;

        /// <summary>Call once per tick, before any navigation controller runs.</summary>
        public static void Sample(Vector3D position, double deltaSeconds)
        {
            if (deltaSeconds <= 0)
                return;

            if (!_havePosition)
            {
                _lastPosition = position;
                _havePosition = true;
                return;
            }

            Vector3D sample = (position - _lastPosition) / deltaSeconds;
            _lastPosition = position;

            // First real sample has nothing to compare against, so take it as-is.
            if (!_haveVelocity)
            {
                _velocity = sample;
                _haveVelocity = true;
                return;
            }

            // A grid jump (Nexus instance transfer, teleport) shows up as impossible acceleration.
            // Drop that one sample; the next tick is continuous again and will be accepted.
            if ((sample - _velocity).Length() / deltaSeconds > MaxPlausibleAcceleration)
                return;

            _velocity = sample;
        }

        public static void Reset()
        {
            _havePosition = false;
            _haveVelocity = false;
            _velocity = Vector3D.Zero;
        }
    }
}
