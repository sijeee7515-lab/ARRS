using UnityEngine;

public class DebugJointWorldPositions : MonoBehaviour
{
    public MultiCameraManager manager;
    public MultiCameraCalibrator calibrator;

    void Update()
    {
        var frame = manager.GetLatestFrameForCamera(0);
        if (frame == null || frame.NumOfBodies == 0) return;

        Body b = frame.Bodies[0];
        var worldJoints = BodyTransformUtils.GetWorldJointPositions(b, calibrator.cameraRoots[0]);

        //Debug.Log("Pelvis world pos: " + worldJoints[(int)Microsoft.Azure.Kinect.BodyTracking.JointId.Pelvis]);
    }
}
