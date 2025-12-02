using UnityEngine;
using Microsoft.Azure.Kinect.BodyTracking;

public static class BodyTransformUtils
{
    /// <summary>
    /// Returns world joint positions in Unity coordinates.
    /// Kinect to unity mapping
    /// </summary>
    public static Vector3[] GetWorldJointPositions(Body body, Transform cameraRoot)
    {
        Vector3[] worldJoints = new Vector3[body.Length];

        for (int i = 0; i < body.Length; i++)
        {
            var jp = body.JointPositions3D[i];

            Vector3 localUnity = new Vector3(jp.X, -jp.Y, jp.Z);

            // Convert through camera root
            worldJoints[i] = cameraRoot.TransformPoint(localUnity);
        }

        return worldJoints;
    }

    /// Gets a single joint world position in Unity coordinates.
    public static Vector3 GetWorldJoint(Body body, int jointId, Transform cameraRoot)
    {
        var jp = body.JointPositions3D[jointId];
        Vector3 localUnity = new Vector3(jp.X, -jp.Y, jp.Z);
        return cameraRoot.TransformPoint(localUnity);
    }

    /// Converts Kinect joint directly to Unity without camera transform.
    public static Vector3 KinectToUnity(Vector3 kinect)
    {
        return new Vector3(kinect.x, -kinect.y, kinect.z);
    }
}
