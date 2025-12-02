using UnityEngine;
using System;
using Microsoft.Azure.Kinect.Sensor;
using Microsoft.Azure.Kinect.BodyTracking;

// Aliases for clarity
using UEVector3 = UnityEngine.Vector3;
using UEQuaternion = UnityEngine.Quaternion;

public class FloorAndImuCalibrator : MonoBehaviour
{
    [Header("References")]
    public MultiCameraManager multiCameraManager;
    public MultiCameraCalibrator multiCameraCalibrator;

    [Header("RANSAC Settings")]
    public int ransacIterations = 200;
    public float inlierThresholdMeters = 0.03f;

    [Header("IMU Blend")]
    [Range(0f, 1f)]
    public float imuBlendWeight = 0.5f;

    private void Awake()
    {
        if (multiCameraManager == null)
            multiCameraManager = GetComponent<MultiCameraManager>();

        if (multiCameraCalibrator == null)
            multiCameraCalibrator = GetComponent<MultiCameraCalibrator>();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F))
        {
            Debug.Log("[Floor+IMU] Triggered Calibration for Camera 0");
            CalibrateCamera0FloorAndIMU();
        }
    }

    private void CalibrateCamera0FloorAndIMU()
    {
        //Grab depth & provider
        BackgroundData frame0 = multiCameraManager.GetLatestFrameForCamera(0);
        SkeletalTrackingProvider provider0 = multiCameraManager.GetProvider(0);



        if (frame0 == null || provider0 == null || provider0.SensorCalibration == null)
        {
            Debug.LogWarning("[Floor+IMU] Missing frame or calibration.");
            return;
        }

        Debug.Log("[FLOOR CALIB] Depth frame source serial: " + frame0.SensorId + " (should be Camera 0)");
        //Depth dimensions
        int width = frame0.DepthImageWidth;
        int height = frame0.DepthImageHeight;
        int N = width * height;

        //Convert depth (mm)
        ushort[] depthMm = ExtractDepthFromBackgroundData(frame0, width, height);

        //Get intrinsics
        var intr = provider0.SensorCalibration.DepthCameraCalibration.Intrinsics.Parameters;
        float cx = intr[0];
        float cy = intr[1];
        float fx = intr[2];
        float fy = intr[3];

        //Build point cloud in Kinect Camera Space
        UEVector3[] cloudKinect = PointCloudExtractor.BuildPointCloudFromDepth(depthMm, width, height, fx, fy, cx, cy);

        //RANSAC (returns Unity-space normal)
        RansacPlane.PlaneResult plane = RansacPlane.FitPlaneRANSAC(cloudKinect, ransacIterations, inlierThresholdMeters);

        if (!plane.valid)
        {
            Debug.LogWarning("[Floor+IMU] RANSAC failed.");
            return;
        }

        //Rotation from floor
        UEQuaternion floorRot = UEQuaternion.FromToRotation(plane.normal, UEVector3.up);

        // IMU orientation 
        UEQuaternion finalRot = floorRot;
        //UEQuaternion? imuRot = TryGetImuOrientation(provider0);

        //if (imuRot.HasValue)
        //finalRot = UEQuaternion.Slerp(floorRot, imuRot.Value, imuBlendWeight);

        //Height = plane distance
        float cameraHeight = Mathf.Abs(plane.d);

        Transform root = multiCameraCalibrator.cameraRoots[0];
        root.position = new UEVector3(0f, cameraHeight, 0f);
        root.rotation = finalRot;

        Debug.Log($"[Floor+IMU] Camera0Root set to height {cameraHeight:F3} m, rot {finalRot.eulerAngles}");
    }

    // Converts your 8-bit depth to mm (not ideal, but needed for now)
    private ushort[] ExtractDepthFromBackgroundData(BackgroundData frame, int width, int height)
    {
        ushort[] depth = new ushort[width * height];
        byte[] src = frame.DepthImage;

        // Map 0–255 to 0–maxDepthMm
        const int maxDepth = 4000;
        int count = Mathf.Min(depth.Length, src.Length);

        for (int i = 0; i < count; i++)
        {
            byte v = src[i];
            depth[i] = (ushort)Mathf.RoundToInt((v / 255f) * maxDepth);
        }

        return depth;
    }

    private UEQuaternion? TryGetImuOrientation(SkeletalTrackingProvider provider)
    {
        try
        {
            provider.StartImuIfNeeded();
            ImuSample sample = provider.SensorDevice.GetImuSample();
            var gSys = sample.AccelerometerSample;

            // Kinect to Unity conversion
            UEVector3 gUnity = new UEVector3(
                gSys.X,
                -gSys.Y,
                gSys.Z
            );

            if (gUnity.sqrMagnitude < 1e-6f)
                return null;

            gUnity.Normalize();

            return UEQuaternion.FromToRotation(UEVector3.down, gUnity);
        }
        catch (Exception e)
        {
            Debug.LogWarning("[Floor+IMU] IMU read failed: " + e.Message);
            return null;
        }
    }

    public void CalibrateCameraFloorAndIMU(int camIndex)
    {
        BackgroundData frame = multiCameraManager.GetLatestFrameForCamera(camIndex);
        SkeletalTrackingProvider provider = multiCameraManager.GetProvider(camIndex);

        if (frame == null || provider == null || provider.SensorCalibration == null)
        {
            Debug.LogWarning("[Floor+IMU] Missing frame or calibration.");
            return;
        }

        int width = frame.DepthImageWidth;
        int height = frame.DepthImageHeight;

        ushort[] depthMm = ExtractDepthFromBackgroundData(frame, width, height);

        var intr = provider.SensorCalibration.DepthCameraCalibration.Intrinsics.Parameters;
        float cx = intr[0];
        float cy = intr[1];
        float fx = intr[2];
        float fy = intr[3];

        Vector3[] cloud = PointCloudExtractor.BuildPointCloudFromDepth(depthMm, width, height, fx, fy, cx, cy);

        var plane = RansacPlane.FitPlaneRANSAC(cloud, ransacIterations, inlierThresholdMeters);
        if (!plane.valid) return;

        Quaternion floorRot = Quaternion.FromToRotation(plane.normal, Vector3.up);

        float camHeight = Mathf.Abs(plane.d);

        Transform root = multiCameraCalibrator.cameraRoots[camIndex];
        root.position = new Vector3(0, camHeight, 0);
        root.rotation = floorRot;

        Debug.Log($"[Floor+IMU] Camera {camIndex} height={camHeight:F3}, rot={floorRot.eulerAngles}");
    }

}


