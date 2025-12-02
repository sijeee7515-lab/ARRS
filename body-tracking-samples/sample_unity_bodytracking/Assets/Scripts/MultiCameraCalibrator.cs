using UnityEngine;
using System.Collections.Generic;
using Microsoft.Azure.Kinect.BodyTracking;
using System.Collections;

public class MultiCameraCalibrator : MonoBehaviour
{
    [Header("Camera Roots")]
    public Transform[] cameraRoots;

    [Header("Calibration State")]
    public bool[] cameraCalibrated;

    [Header("References")]
    public MultiCameraManager multiCameraManager;

    [Header("Settings")]
    public int targetCameraIndex = 1;
    public float sampleDuration = 1.0f;

    [Header("Visualization")]
    public GameObject cameraMarkerPrefab;

    // Translation samples (in Cam0’s local space)
    private readonly Dictionary<int, List<Vector3>> translationSamples =
        new Dictionary<int, List<Vector3>>();

    private bool isCollecting = false;

    // Persistent camera model instances
    private GameObject[] cameraMarkers;

    void Awake()
    {
        if (cameraRoots != null && cameraRoots.Length > 0)
        {
            cameraCalibrated = new bool[cameraRoots.Length];
            cameraMarkers = new GameObject[cameraRoots.Length];

            for (int i = 0; i < cameraCalibrated.Length; i++)
                cameraCalibrated[i] = false;
        }

        if (multiCameraManager == null)
            multiCameraManager = FindObjectOfType<MultiCameraManager>();

        // ----------------------------------------------------------
        // 🔥 Create ONE persistent camera model per cameraRoot
        // ----------------------------------------------------------
        if (cameraMarkerPrefab != null && cameraRoots != null)
        {
            for (int i = 0; i < cameraRoots.Length; i++)
            {
                cameraMarkers[i] = Instantiate(cameraMarkerPrefab, cameraRoots[i]);
                cameraMarkers[i].name = $"CameraModel_{i}";
                cameraMarkers[i].transform.localPosition = Vector3.zero;
                cameraMarkers[i].transform.localRotation = Quaternion.identity;
            }
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.C) && !isCollecting)
            StartCoroutine(CalibrationRoutine());
    }

    IEnumerator CalibrationRoutine()
    {
        Debug.Log("[CALIB] Starting translation-only calibration...");
        isCollecting = true;

        float startTime = Time.time;

        while (Time.time - startTime < sampleDuration)
        {
            BackgroundData frame0 = multiCameraManager.GetLatestFrameForCamera(0);
            BackgroundData frame1 = multiCameraManager.GetLatestFrameForCamera(targetCameraIndex);

            if (frame0 != null && frame1 != null &&
                frame0.NumOfBodies > 0 && frame1.NumOfBodies > 0)
            {
                Body refBody = frame0.Bodies[0];
                Body targetBody = frame1.Bodies[0];

                AddCalibrationSample(targetCameraIndex, refBody, targetBody);

                int n = translationSamples.ContainsKey(targetCameraIndex)
                        ? translationSamples[targetCameraIndex].Count : 0;

                Debug.Log($"[CALIB] Added translation sample #{n}");
            }

            yield return null;
        }

        Debug.Log("[CALIB] Finishing calibration...");
        FinishCalibrationForCamera(targetCameraIndex);
        isCollecting = false;
    }

    // ----------------------------------------------------------------------
    // Add a translation sample
    // ----------------------------------------------------------------------
    public void AddCalibrationSample(int targetCamIndex, Body bodyRef, Body bodyTarget)
    {
        if (!translationSamples.ContainsKey(targetCamIndex))
            translationSamples[targetCamIndex] = new List<Vector3>();

        if (TryEstimateSingleSample(bodyRef, bodyTarget, out Vector3 tSample))
        {
            translationSamples[targetCamIndex].Add(tSample);

            // Debug projections on Cam0 axes
            Vector3 f = cameraRoots[0].forward;
            Vector3 r = cameraRoots[0].right;
            Vector3 u = cameraRoots[0].up;

            Debug.Log(
                $"[CALIB][Sample] Cam{targetCamIndex} t={tSample} |t|={tSample.magnitude:F3} " +
                $"fwd={Vector3.Dot(tSample, f):F3} right={Vector3.Dot(tSample, r):F3} up={Vector3.Dot(tSample, u):F3}"
            );
        }
    }

    // ----------------------------------------------------------------------
    // Translation estimation (averaged from several stable joints)
    // ----------------------------------------------------------------------
    bool TryEstimateSingleSample(Body b0, Body b1, out Vector3 tSample)
    {
        tSample = Vector3.zero;

        int[] joints =
        {
            (int)JointId.Pelvis,
            (int)JointId.SpineNavel,
            (int)JointId.SpineChest,
            (int)JointId.Neck,
            (int)JointId.ClavicleLeft,
            (int)JointId.ClavicleRight
        };

        int validCount = 0;

        foreach (int j in joints)
        {
            Vector3 p0 = b0.JointPositions3D[j].ToUnityKinect();
            Vector3 p1 = b1.JointPositions3D[j].ToUnityKinect();

            if (p0 == Vector3.zero || p1 == Vector3.zero)
                continue;

            tSample += (p0 - p1);
            validCount++;
        }

        if (validCount == 0)
            return false;

        tSample /= validCount;
        return true;
    }

    // ----------------------------------------------------------------------
    // Apply calibration
    // ----------------------------------------------------------------------
    public void FinishCalibrationForCamera(int targetCamIndex)
    {
        if (!translationSamples.ContainsKey(targetCamIndex) ||
            translationSamples[targetCamIndex].Count == 0)
        {
            Debug.LogWarning($"[CALIB] No translation samples for camera {targetCamIndex}");
            return;
        }

        Vector3 T = SkeletonCalibrationMath.AverageVectors(
            translationSamples[targetCamIndex].ToArray());

        Transform refRoot = cameraRoots[0];
        Transform targetRoot = cameraRoots[targetCamIndex];

        // Move target camera to match body overlap
        targetRoot.position = refRoot.TransformPoint(T);

        // Copy rotation of camera 0 (for now)
        targetRoot.rotation = refRoot.rotation;

        cameraCalibrated[targetCamIndex] = true;

        Debug.Log($"[CALIB] Camera {targetCamIndex} calibrated.");
        Debug.Log($"[CALIB] Final Translation T = {T}");
        Debug.Log($"[CALIB] Applied World Pos = {targetRoot.position}");
        Debug.Log($"[CALIB] Applied World Rot = {targetRoot.rotation.eulerAngles}");
    }

    public void ResetCalibration(int camIndex)
    {
        if (cameraRoots == null || camIndex < 0 || camIndex >= cameraRoots.Length)
            return;

        cameraCalibrated[camIndex] = false;

        if (translationSamples.ContainsKey(camIndex))
            translationSamples.Remove(camIndex);
    }
}

// ----------------------------------------------------------------------
// Kinect → Unity mapping helpers
// ----------------------------------------------------------------------
public static class VecExtensions
{
    // Kinect coordinate (x, y-down, z) → Unity (x, y-up, z)
    public static Vector3 ToUnityKinect(this System.Numerics.Vector3 v)
    {
        return new Vector3(v.X, -v.Y, v.Z);
    }

    public static Quaternion ToUnity(this System.Numerics.Quaternion q)
    {
        return new Quaternion(q.X, q.Y, q.Z, q.W);
    }
}
