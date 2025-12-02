using UnityEngine;

public static class PointCloudExtractor
{
    /// <summary>
    /// Build a point cloud in camera space from a depth buffer and intrinsics.
    /// depthBuffer: depth in millimeters, size = width * height.
    /// fx, fy, cx, cy: depth camera intrinsics.
    /// </summary>
    public static Vector3[] BuildPointCloudFromDepth(
        ushort[] depthBuffer,
        int width,
        int height,
        float fx,
        float fy,
        float cx,
        float cy)
    {
        int N = width * height;
        Vector3[] cloud = new Vector3[N];

        for (int i = 0; i < N; i++)
        {
            ushort d = depthBuffer[i];
            if (d == 0)
            {
                cloud[i] = Vector3.zero;
                continue;
            }

            int u = i % width;
            int v = i / width;

            float z = d / 1000f;
            float x = ((u - cx) * z) / fx;
            float y = ((v - cy) * z) / fy;

            cloud[i] = new Vector3(x, y, z);
        }

        return cloud;
    }
}
