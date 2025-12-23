using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;
using Sandbox.ModAPI.Ingame;
using VRage.Game;

namespace IngameScript
{
    //Original class by klassekatze: https://pastebin.com/Yz9ixdQF
    public class JitAim : IAimController
    {
        public static double gyroMaxRPM = 3.1415;

        public JitAim(MyCubeSize gridSize)
        {
            double angleMultiplier = gridSize == MyCubeSize.Small ? 2 : 1;
            gyroMaxRPM *= angleMultiplier;
            lastAngleRoll = lastAnglePitch = lastAngleYaw = 0;
            lMPTRoll = lMPTPitch = lMPTYaw = 0;
        }

        double lastAngleRoll, lastAnglePitch, lastAngleYaw;
        double lMPTRoll, lMPTPitch, lMPTYaw;
        double modr, modp, mody = 0;

        private bool active = false;

        public void flush()
        {
            if (!active)
            {
                active = true;
                lastAngleRoll = lastAnglePitch = lastAngleYaw = 0;
                lMPTRoll = lMPTPitch = lMPTYaw = 0;
                modr = modp = mody = 0;
            }
        }

        private void calculateAxisSpecificData(double now, ref double prior, ref double lastMPT, ref double mod, out bool ontarg, out bool braking)//, out double ticksToStop, out double ticksToTarget)
        {
            ontarg = false;

            var radMovedPerTick = Math.Abs(prior - now);
            var ticksToTarget = Math.Abs(now) / radMovedPerTick;
            var initVel = radMovedPerTick;
            var rateOfDecel = Math.Abs(lastMPT - radMovedPerTick);
            if (rateOfDecel > mod) mod = rateOfDecel;

            //if (Math.Abs(now) > nobrake_threshold) rateOfDecel *= 1.5;//overestimating here did not improve timings
            var ticksToStop = initVel / rateOfDecel;
            //mod - maximum observed decel - saved 0.1s on large sluggish ship but lost .3s on sg snappy ship.
            //sticking to the conservative metric

            bool closing = Math.Abs(now) < Math.Abs(prior);

            if (!closing)
            {
                lastMPT = 0.0001;
                mod = MathHelper.EPSILON;
            }
            else lastMPT = radMovedPerTick;

            if (closing)
            {
                //if (ticksToStop > ticksToTarget + 1) braking = true;
                if (ticksToStop > ticksToTarget) braking = true;
                else braking = false;
            }
            else braking = false;

            if (Math.Abs(now) < error_threshold)
            {
                braking = true;
                if (radMovedPerTick < minVelThreshold) ontarg = true;
            }

            prior = now;
        }

        double error_threshold = MathHelperD.ToRadians(0.025);
        double minVelThreshold = MathHelperD.ToRadians(0.005);

        double nobrake_threshold = MathHelperD.ToRadians(45);
        double amp_threshold = MathHelperD.ToRadians(10);

        int minTicksOnTarget = 5;
        int ticksOnTarget = 0;

        public static void GetRotationAnglesSimultaneous(Vector3D desiredForwardVector, MatrixD worldMatrix, out double yaw, out double pitch, out double roll)
        {
            desiredForwardVector = Utils.SafeNormalize(desiredForwardVector);

            MatrixD transposedWm;
            MatrixD.Transpose(ref worldMatrix, out transposedWm);
            Vector3D.Rotate(ref desiredForwardVector, ref transposedWm, out desiredForwardVector);

            Vector3D axis = new Vector3D(desiredForwardVector.Y, -desiredForwardVector.X, 0);
            double angle = Math.Acos(MathHelper.Clamp(-desiredForwardVector.Z, -1.0, 1.0));

            if (Vector3D.IsZero(axis))
            {
                angle = desiredForwardVector.Z < 0 ? 0 : Math.PI;
                yaw = angle;
                pitch = 0;
                roll = 0;
                return;
            }

            axis = Utils.SafeNormalize(axis);
            yaw = -axis.Y * angle;
            pitch = axis.X * angle;
            roll = -axis.Z * angle;
        }

        public static void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, IMyGyro gyro, MatrixD refMatrix)
        {
            var rotationVec = new Vector3D(-pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs
            var shipMatrix = refMatrix;
            var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);

            var gyroMatrix = gyro.WorldMatrix;
            var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));
            gyro.Pitch = (float)transformedRotationVec.X;
            gyro.Yaw = (float)transformedRotationVec.Y;
            gyro.Roll = (float)transformedRotationVec.Z;
            gyro.GyroOverride = true;
        }

        public void Orient(Vector3D forward, IMyGyro gyro, MatrixD refMatrix)
        {
            flush();

            double pitch, yaw, roll;
            GetRotationAnglesSimultaneous(forward, refMatrix, out yaw, out pitch, out roll);

            bool yT, pT, rT, yB, pB, rB;
            calculateAxisSpecificData(roll, ref lastAngleRoll, ref lMPTRoll, ref modr, out rT, out rB);
            calculateAxisSpecificData(pitch, ref lastAnglePitch, ref lMPTPitch, ref modp, out pT, out pB);
            calculateAxisSpecificData(yaw, ref lastAngleYaw, ref lMPTYaw, ref mody, out yT, out yB);

            Vector3D a_impulse = new Vector3D(pB ? 0 : pitch, yB ? 0 : yaw, rB ? 0 : roll);
            if (a_impulse != Vector3D.Zero)
            {
                var m = a_impulse.AbsMax();
                if (m > amp_threshold) m = gyroMaxRPM;
                else m = m / amp_threshold * gyroMaxRPM;


                a_impulse = a_impulse / a_impulse.AbsMax() * m;
            }

            ApplyGyroOverride(a_impulse.X, a_impulse.Y, a_impulse.Z, gyro, refMatrix);

            if (yT && pT && rT)
            {
                ticksOnTarget += 1;
            }
            else ticksOnTarget = 0;

            if (ticksOnTarget > minTicksOnTarget)
            {
                //gyro.GyroOverride = false;
                gyro.Pitch = 0;
                gyro.Yaw = 0;
                gyro.Roll = 0;
            }
        }

        public void Orient(Vector3D forward, Vector3D up, IMyGyro gyro, MatrixD refMatrix) =>
            Orient(forward, gyro, refMatrix);

        public void Reset()
        {
            active = false;
        }
    }
}
