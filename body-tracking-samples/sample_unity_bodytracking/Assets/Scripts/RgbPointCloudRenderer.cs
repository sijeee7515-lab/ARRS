using UnityEngine;
using Microsoft.Azure.Kinect.Sensor; // for Calibration intrinsics

public class RgbPointCloudRenderer : MonoBehaviour
{
    [Header("References")]
    public MultiCameraManager multiCameraManager;
    public MultiCameraCalibrator multiCameraCalibrator;

    [Header("Camera Index")]
    [Tooltip("Which camera (index into MultiCameraManager.deviceIds) to visualize.")]
    public int cameraIndex = 0;

    [Header("Clipping (meters)")]
    public float nearClip = 0.3f;
    public float farClip = 5.0f;

    private Mesh mesh;
    private Vector3[] vertices;
    private Color32[] colors;
    private int[] indices;

    private int currentWidth = -1;
    private int currentHeight = -1;

    //Latency
    private bool latencyBaselineSet = false;
    private float latencyBaselineMs = 0f;
    private float nextLogTime = 0f;
    public float latencyLogInterval = 1.5f;

    void Awake()
    {
        if (multiCameraManager == null)
            multiCameraManager = FindObjectOfType<MultiCameraManager>();

        if (multiCameraCalibrator == null)
            multiCameraCalibrator = FindObjectOfType<MultiCameraCalibrator>();

        // Ensure there is a MeshFilter + MeshRenderer
        if (GetComponent<MeshFilter>() == null)
            gameObject.AddComponent<MeshFilter>();
        if (GetComponent<MeshRenderer>() == null)
            gameObject.AddComponent<MeshRenderer>();
    }

    void Update()
    {
        if (multiCameraManager == null)
            return;

        BackgroundData frame = multiCameraManager.GetLatestFrameForCamera(cameraIndex);
        SkeletalTrackingProvider provider = multiCameraManager.GetProvider(cameraIndex);

        if (frame == null || provider == null || provider.SensorCalibration == null)
            return;

        int width = frame.DepthImageWidth;
        int height = frame.DepthImageHeight;
        int N = width * height;

        // Compute point cloud latency

        float nowMs = Time.realtimeSinceStartup * 1000f;
        float rawAgeMs = nowMs - frame.TimestampInMs;

        // Calibrate once to remove the constant offset between Unity & Kinect clocks
        if (!latencyBaselineSet)
        {
            latencyBaselineSet = true;
            latencyBaselineMs = rawAgeMs;
        }

        // Actual end-to-end latency relative to first frame
        float latencyMs = rawAgeMs - latencyBaselineMs;

        // Log occasionally, not every frame
        if (Time.time >= nextLogTime)
        {
            Debug.Log($"[Latency][Cam {cameraIndex}] {latencyMs:F2} ms (raw={rawAgeMs:F2} ms)");
            nextLogTime = Time.time + latencyLogInterval;
        }

        if (width <= 0 || height <= 0 || frame.DepthImageMm == null)
            return;

        // Initialize / resize mesh if needed
        if (mesh == null || width != currentWidth || height != currentHeight)
        {
            InitMesh(width, height);
        }

        ushort[] depthMm = frame.DepthImageMm;
        byte[] colorBgra = frame.ColorImageBgra;

        if (depthMm == null || depthMm.Length < N)
            return;

        // If color isn't ready yet, just skip drawing
        if (colorBgra == null || colorBgra.Length < N * 4)
            return;

        // Sync transform with calibrated camera root
        if (multiCameraCalibrator != null &&
            multiCameraCalibrator.cameraRoots != null &&
            cameraIndex >= 0 &&
            cameraIndex < multiCameraCalibrator.cameraRoots.Length)
        {
            Transform root = multiCameraCalibrator.cameraRoots[cameraIndex];
            if (root != null)
            {
                transform.position = root.position;
                transform.rotation = root.rotation;
            }
        }

        // Depth intrinsics
        float[] intr = provider.SensorCalibration.DepthCameraCalibration.Intrinsics.Parameters;
        float cx = intr[0];
        float cy = intr[1];
        float fx = intr[2];
        float fy = intr[3];

        // Build vertices + colors in-place (no allocations)
        for (int v = 0; v < height; v++)
        {
            for (int u = 0; u < width; u++)
            {
                int idx = v * width + u;
                ushort dMm = depthMm[idx];

                if (dMm == 0)
                {
                    vertices[idx] = Vector3.zero;
                    colors[idx].a = 0;
                    continue;
                }

                float z = dMm * 0.001f; // mm -> meters

                if (z < nearClip || z > farClip)
                {
                    vertices[idx] = Vector3.zero;
                    colors[idx].a = 0;
                    continue;
                }

                float x = ((u - cx) * z) / fx;
                float yCam = ((v - cy) * z) / fy;

                // Kinect: +Y down; Unity: +Y up, so flip
                vertices[idx] = new Vector3(x, -yCam, z);

                int cIndex = idx * 4;
                byte B = colorBgra[cIndex + 0];
                byte G = colorBgra[cIndex + 1];
                byte R = colorBgra[cIndex + 2];
                byte A = colorBgra[cIndex + 3];

                

                colors[idx] = new Color32(R, G, B, A);

                //if (idx == width * height / 2) {
                    //Debug.Log($"Midpoint color R={R}, G={G}, B={B}, A={A}");
                //}
            }
        }  


        mesh.vertices = vertices;
        mesh.colors32 = colors;
        mesh.RecalculateBounds();
        // indices never change
    }

    private void InitMesh(int width, int height)
    {
        currentWidth = width;
        currentHeight = height;
        int N = width * height;

        mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

        vertices = new Vector3[N];
        colors = new Color32[N];
        indices = new int[N];

        for (int i = 0; i < N; i++)
        {
            indices[i] = i;
            colors[i] = new Color32(0, 0, 0, 0);
        }

        mesh.vertices = vertices;
        mesh.colors32 = colors;
        mesh.SetIndices(indices, MeshTopology.Points, 0);

        GetComponent<MeshFilter>().mesh = mesh;
    }
}
