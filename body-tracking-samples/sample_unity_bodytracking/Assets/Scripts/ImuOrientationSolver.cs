using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;

public static class ImuOrientationSolver
{
    public static Quaternion SolveOrientation(ImuSample sample)
    {
        Vector3 gravity = new Vector3(
            sample.AccelerometerSample.X,
            sample.AccelerometerSample.Y,
            sample.AccelerometerSample.Z
        );

        gravity.Normalize();

        // Camera's forward axis is +Z, so compute the rotation that makes its "down" match gravity
        Quaternion rotation = Quaternion.FromToRotation(Vector3.down, gravity);

        return rotation;
    }
}
