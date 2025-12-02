using UnityEngine;
using System.Collections.Generic;
using Microsoft.Azure.Kinect.BodyTracking;

public class SkeletonFusionManager : MonoBehaviour
{
    public MultiCameraManager multiCameraManager;
    public MultiCameraCalibrator calibrator;

    [Header("Output")]
    public Body fusedBody;
    public bool hasFusedBody = false;

    void Awake()
    {
        if (multiCameraManager == null)
            multiCameraManager = GetComponent<MultiCameraManager>();
        if (calibrator == null)
            calibrator = GetComponent<MultiCameraCalibrator>();
    }

    void Update()
    {
        if (calibrator.cameraRoots == null || calibrator.cameraRoots.Length == 0)
        {
            hasFusedBody = false;
            return;
        }

        List<Vector3[]> worldJointsPerCam = new List<Vector3[]>();
        List<JointConfidenceLevel[]> confidencesPerCam = new List<JointConfidenceLevel[]>();
        int jointCount = -1;

        // Loop through all calibrated cameras
        for (int camIndex = 0; camIndex < calibrator.cameraRoots.Length; camIndex++)
        {
            if (!calibrator.cameraCalibrated[camIndex])
                continue;

            BackgroundData frame = multiCameraManager.GetLatestFrameForCamera(camIndex);

            if (frame == null)
                continue;

            if (frame.NumOfBodies == 0 || frame.Bodies == null || frame.Bodies.Length == 0)
                continue;

            Body body = frame.Bodies[0];

            // Validate body joint data
            if (body.JointPositions3D == null || body.JointPositions3D.Length == 0)
                continue;
            if (body.Length == 0)
                continue;

            // First valid skeleton sets the expected joint count
            if (jointCount < 0)
                jointCount = body.Length;

            // Skip if mismatched skeleton sizes
            if (body.Length != jointCount)
            {
                Debug.LogWarning($"Camera {camIndex}: skeleton size mismatch ({body.Length} vs {jointCount}). Skipping this camera frame.");
                continue;
            }

            // Convert to world space
            Vector3[] worldJoints = BodyTransformUtils.GetWorldJointPositions(body, calibrator.cameraRoots[camIndex]);
            if (worldJoints == null || worldJoints.Length != jointCount)
                continue;

            // Validate confidence array
            if (body.JointPrecisions == null ||
                body.JointPrecisions.Length == 0 ||
                body.JointPrecisions.Length < jointCount)
            {
                Debug.LogWarning($"Camera {camIndex}: JointPrecisions invalid ({body.JointPrecisions?.Length}). Skipping.");
                continue;
            }

            worldJointsPerCam.Add(worldJoints);
            confidencesPerCam.Add(body.JointPrecisions);
        }

        // If no cameras produced valid skeleton data then no fused result
        if (jointCount <= 0 || worldJointsPerCam.Count == 0)
        {
            hasFusedBody = false;
            return;
        }

        // Initialize fused skeleton
        fusedBody = new Body(jointCount);
        fusedBody.Length = jointCount;
        fusedBody.Id = 0;

        // Fuse joints across cameras
        for (int j = 0; j < jointCount; j++)
        {
            bool anyValid = false;
            Vector3 sum = Vector3.zero;
            int count = 0;

            JointConfidenceLevel bestConfidence = JointConfidenceLevel.None;
            Vector3 bestPos = Vector3.zero;

            for (int c = 0; c < worldJointsPerCam.Count; c++)
            {
                // Safety: JointPrecisions length check
                if (j >= confidencesPerCam[c].Length)
                    continue;

                JointConfidenceLevel conf = confidencesPerCam[c][j];
                if (conf == JointConfidenceLevel.None)
                    continue;

                Vector3 pos = worldJointsPerCam[c][j];
                anyValid = true;

                // Pick highest-confidence joint
                if (conf > bestConfidence)
                {
                    bestConfidence = conf;
                    bestPos = pos;
                }

                // Also average positions
                sum += pos;
                count++;
            }

            System.Numerics.Vector3 fusedPos3D = new System.Numerics.Vector3();

            if (anyValid && count > 0)
            {
                Vector3 avg = sum / count;
                fusedPos3D.X = avg.x;
                fusedPos3D.Y = avg.y;
                fusedPos3D.Z = avg.z;
            }

            fusedBody.JointPositions3D[j] = fusedPos3D;
            fusedBody.JointPrecisions[j] = anyValid ? bestConfidence : JointConfidenceLevel.None;
        }

        hasFusedBody = true;
        ComputeFusedSkeletonError(worldJointsPerCam, fusedBody, calibrator.cameraRoots);
    }

    private void ComputeFusedSkeletonError(
    List<Vector3[]> worldJointsPerCam,
    Body fusedBody,
    Transform[] cameraRoots)
{
    int jointCount = fusedBody.Length;
    if (jointCount <= 0) return;

    // Convert fused skeleton to world space (use Camera 0 as reference)
    Vector3[] fusedWorld = BodyTransformUtils.GetWorldJointPositions(
        fusedBody,
        cameraRoots[0]
    );

    float totalError = 0f;
    int sampleCount = 0;

    foreach (Vector3[] cam in worldJointsPerCam)
    {
        for (int j = 0; j < jointCount; j++)
        {
            float e = Vector3.Distance(fusedWorld[j], cam[j]);
            totalError += e;
            sampleCount++;
        }
    }

    if (sampleCount == 0) return;

    float avgError = totalError / sampleCount; // in meters
    Debug.Log($"[FusionError] Avg joint error = {avgError * 1000f:F1} mm");
}


}


