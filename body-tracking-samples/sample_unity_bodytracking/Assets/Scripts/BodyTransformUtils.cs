using UnityEngine;
using Microsoft.Azure.Kinect.BodyTracking;

public static class BodyTransformUtils
{
    /// <summary>
    /// Returns world joint positions in Unity coordinates.
    /// Kinect → Unity mapping: (x, y, z) → (x, -y, z)
    /// </summary>
    public static Vector3[] GetWorldJointPositions(Body body, Transform cameraRoot)
    {
        Vector3[] worldJoints = new Vector3[body.Length];

        for (int i = 0; i < body.Length; i++)
        {
            var jp = body.JointPositions3D[i];

            // Kinect cam space → Unity space
            Vector3 localUnity = new Vector3(jp.X, -jp.Y, jp.Z);

            // Convert through camera root
            worldJoints[i] = cameraRoot.TransformPoint(localUnity);
        }

        return worldJoints;
    }

    /// <summary>
    /// Gets a single joint world position in Unity coordinates.
    /// </summary>
    public static Vector3 GetWorldJoint(Body body, int jointId, Transform cameraRoot)
    {
        var jp = body.JointPositions3D[jointId];
        Vector3 localUnity = new Vector3(jp.X, -jp.Y, jp.Z);
        return cameraRoot.TransformPoint(localUnity);
    }

    /// <summary>
    /// Converts Kinect joint directly to Unity without camera transform.
    /// </summary>
    public static Vector3 KinectToUnity(Vector3 kinect)
    {
        return new Vector3(kinect.x, -kinect.y, kinect.z);
    }
}
