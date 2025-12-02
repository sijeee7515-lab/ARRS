using UnityEngine;
using Microsoft.Azure.Kinect.BodyTracking;

public static class SkeletonCalibrationMath
{
    // Quaternion + vector averaging utilities

    public static Quaternion AverageQuaternions(Quaternion[] quaternions)
    {
        if (quaternions == null || quaternions.Length == 0)
            return Quaternion.identity;

        Quaternion avg = quaternions[0];

        for (int i = 1; i < quaternions.Length; i++)
        {
            Quaternion q = quaternions[i];

            // Ensure same hemisphere
            if (Quaternion.Dot(avg, q) < 0f)
                q = new Quaternion(-q.x, -q.y, -q.z, -q.w);

            avg = Quaternion.Slerp(avg, q, 1f / (i + 1));
        }

        return avg.normalized;
    }

    public static Vector3 AverageVectors(Vector3[] vectors)
    {
        if (vectors == null || vectors.Length == 0)
            return Vector3.zero;

        Vector3 sum = Vector3.zero;

        for (int i = 0; i < vectors.Length; i++)
            sum += vectors[i];

        return sum / vectors.Length;
    }

    //Extract torso basis in Unity coordinates
    public static bool TryGetBodyBasisUnity(
        Body body,
        out Vector3 up,
        out Vector3 forward,
        out Vector3 center)
    {
        int pelvisIdx = (int)JointId.Pelvis;
        int chestIdx = (int)JointId.SpineChest;
        int shoulderLIdx = (int)JointId.ShoulderLeft;
        int shoulderRIdx = (int)JointId.ShoulderRight;

        up = Vector3.up;
        forward = Vector3.forward;
        center = Vector3.zero;

        if (body.Length <= shoulderRIdx)
            return false;

        var p = body.JointPositions3D[pelvisIdx];
        var c = body.JointPositions3D[chestIdx];
        var sl = body.JointPositions3D[shoulderLIdx];
        var sr = body.JointPositions3D[shoulderRIdx];

        Vector3 P = new Vector3(p.X, -p.Y, p.Z);
        Vector3 C = new Vector3(c.X, -c.Y, c.Z);
        Vector3 SL = new Vector3(sl.X, -sl.Y, sl.Z);
        Vector3 SR = new Vector3(sr.X, -sr.Y, sr.Z);

        Vector3 upDir = (C - P);
        Vector3 rightDir = (SR - SL);

        if (upDir.sqrMagnitude < 1e-5f || rightDir.sqrMagnitude < 1e-5f)
            return false;

        upDir.Normalize();
        rightDir.Normalize();

        Vector3 fwd = Vector3.Cross(upDir, rightDir);
        if (fwd.sqrMagnitude < 1e-5f)
            return false;

        fwd.Normalize();

        up = upDir;
        forward = fwd;
        center = 0.5f * (P + C);

        return true;
    }
}
