using System.Collections.Generic;
using UnityEngine;

public static class RansacPlane
{
    public struct PlaneResult
    {
        public Vector3 normal;   // Unity-space normal
        public float d;
        public int inliers;
        public bool valid;
    }

    // NOTE: points[] must be in KINECT CAMERA SPACE: (+X right, +Y down, +Z forward)
    // This function AUTOCONVERTS to UNITY SPACE: (+X right, +Y up, +Z forward)
    public static PlaneResult FitPlaneRANSAC(Vector3[] points, int iterations = 200, float threshold = 0.03f)
    {
        PlaneResult best = new PlaneResult();
        best.inliers = -1;

        int N = points.Length;
        if (N < 3)
            return best;

        for (int i = 0; i < iterations; i++)
        {
            // --- Sample 3 non-zero points ---
            Vector3 p1 = Vector3.zero, p2 = Vector3.zero, p3 = Vector3.zero;

            for (int tries = 0; tries < 20; tries++)
            {
                p1 = points[Random.Range(0, N)];
                p2 = points[Random.Range(0, N)];
                p3 = points[Random.Range(0, N)];

                if (p1 != Vector3.zero && p2 != Vector3.zero && p3 != Vector3.zero)
                    break;
            }

            // --- Compute raw normal in Kinect camera coordinates ---
            Vector3 v1 = p2 - p1;
            Vector3 v2 = p3 - p1;
            Vector3 normalKinect = Vector3.Cross(v1, v2);

            if (normalKinect.sqrMagnitude < 1e-6f)
                continue;

            normalKinect.Normalize();

            // --- Convert Kinect → Unity (+Y down → +Y up) ---
            Vector3 normalUnity = new Vector3(
                normalKinect.x,
                -normalKinect.y,
                normalKinect.z
            );
            normalUnity.Normalize();

            // Convert one point to Unity so d is in the proper coordinate system
            Vector3 p1Unity = ConvertPointKinectToUnity(p1);

            // Plane form: dot(n, x) + d = 0
            float d = -Vector3.Dot(normalUnity, p1Unity);

            // --- Count inliers in Unity space ---
            int count = 0;
            for (int k = 0; k < N; k++)
            {
                if (points[k] == Vector3.zero) continue;

                Vector3 pUnity = ConvertPointKinectToUnity(points[k]);
                float dist = Mathf.Abs(Vector3.Dot(normalUnity, pUnity) + d);

                if (dist < threshold)
                    count++;
            }

            // --- Update best model ---
            if (count > best.inliers)
            {
                best.normal = normalUnity;
                best.d = d;
                best.inliers = count;
                best.valid = true;
            }
        }

        // ---------------------------------------------------------
        // SAFETY FIX: Force plane normal to face upward in Unity
        // ---------------------------------------------------------
        if (best.valid)
        {
            // If the normal is pointing downward, flip it & plane offset
            if (Vector3.Dot(best.normal, Vector3.up) < 0f)
            {
                best.normal = -best.normal;
                best.d = -best.d;
            }
        }

        return best;
    }

    // Kinect → Unity position mapping
    private static Vector3 ConvertPointKinectToUnity(Vector3 p)
    {
        return new Vector3(p.x, -p.y, p.z);
    }
}
