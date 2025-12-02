using Microsoft.Azure.Kinect.BodyTracking;
using Microsoft.Azure.Kinect.Sensor;
using System;
using System.IO;
using System.Threading;
using UnityEngine;
using System.Numerics; // For System.Numerics.Vector2, Vector3

public class SkeletalTrackingProvider : BackgroundDataProvider
{
    public Device SensorDevice { get; private set; }
    public Calibration SensorCalibration { get; private set; }
    private bool imuStarted = false;

    bool readFirstFrame = false;
    TimeSpan initialTimestamp;

    public SkeletalTrackingProvider(int id) : base(id)
    {
        Debug.Log("SkeletalTrackingProvider constructor for device " + id);
    }

    System.Runtime.Serialization.Formatters.Binary.BinaryFormatter binaryFormatter { get; set; } =
        new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();

    public Stream RawDataLoggingFile = null;


    public void StartImuIfNeeded()
    {
        if (SensorDevice == null)
        {
            Debug.LogWarning("StartImuIfNeeded() called, but SensorDevice is null.");
            return;
        }

        if (!imuStarted)
        {
            Debug.Log("Starting IMU for this Kinect device...");
            SensorDevice.StartImu();
            imuStarted = true;
        }
    }


    protected override void RunBackgroundThreadAsync(int id, CancellationToken token)
    {
        try
        {
            Debug.Log("Starting body tracker background thread for device " + id);

            BackgroundData currentFrameData = new BackgroundData();

            SensorDevice = Device.Open(id);

            using (Device device = SensorDevice)
            {
                device.StartCameras(new DeviceConfiguration()
                {
                    CameraFPS = FPS.FPS30,
                    ColorResolution = ColorResolution.R720p,
                    DepthMode = DepthMode.NFOV_Unbinned,
                    WiredSyncMode = WiredSyncMode.Standalone,
                });

                Debug.Log($"Open K4A device successful. id {id} sn:{device.SerialNum}");

                SensorCalibration = device.GetCalibration();
                Calibration calibration = SensorCalibration;

                using (Tracker tracker = Tracker.Create(calibration, new TrackerConfiguration()
                {
                    ProcessingMode = TrackerProcessingMode.Gpu,
                    SensorOrientation = SensorOrientation.Default
                }))
                {
                    while (!token.IsCancellationRequested)
                    {
                        using (Capture sensorCapture = device.GetCapture())
                            tracker.EnqueueCapture(sensorCapture);

                        using (Frame frame = tracker.PopResult(TimeSpan.Zero, throwOnTimeout: false))
                        {
                            if (frame == null)
                                continue;

                            IsRunning = true;

                            currentFrameData.SensorId = id;
                            currentFrameData.NumOfBodies = frame.NumberOfBodies;

                            for (uint i = 0; i < currentFrameData.NumOfBodies; i++)
                                currentFrameData.Bodies[i].CopyFromBodyTrackingSdk(frame.GetBody(i), calibration);

                            Capture bodyFrameCapture = frame.Capture;

                            Image depthImage = bodyFrameCapture.Depth;
                            Image colorImage = bodyFrameCapture.Color;

                            // ------------ COLOR ------------
                            if (colorImage != null)
                            {
                                currentFrameData.ColorWidth = colorImage.WidthPixels;
                                currentFrameData.ColorHeight = colorImage.HeightPixels;

                                int width = colorImage.WidthPixels;
                                int height = colorImage.HeightPixels;
                                int stride = colorImage.StrideBytes;     // padded row stride
                                int bpp = 4;                          // BGRA → 4 bytes/pixel

                                // RAW access (stride-safe)
                                ReadOnlySpan<byte> src = colorImage.Memory.Span;  // <-- THIS IS THE FIX

                                // Copy row-by-row into compacted buffer
                                for (int y = 0; y < height; y++)
                                {
                                    int srcIndex = y * stride;       // padded row
                                    int dstIndex = y * width * bpp;  // compact row

                                    // Copy only the VALID pixel region, not the stride padding
                                    src.Slice(srcIndex, width * bpp)
                                       .CopyTo(currentFrameData.ColorImage.AsSpan().Slice(dstIndex));
                                }
                            }



                            // ------------ TIMESTAMP ------------
                            if (!readFirstFrame)
                            {
                                readFirstFrame = true;
                                initialTimestamp = depthImage.DeviceTimestamp;
                            }

                            currentFrameData.TimestampInMs =
                                (float)(depthImage.DeviceTimestamp - initialTimestamp).TotalMilliseconds;

                            // ------------ DEPTH ------------
                            int w = depthImage.WidthPixels;
                            int h = depthImage.HeightPixels;

                            currentFrameData.DepthImageWidth = w;
                            currentFrameData.DepthImageHeight = h;

                            var dspan = depthImage.GetPixels<ushort>().Span;
                            int pcount = dspan.Length;

                            currentFrameData.DepthImageSize = pcount;

                            for (int i = 0; i < pcount; i++)
                                currentFrameData.DepthImageMm[i] = dspan[i];

                            // Legacy grayscale
                            for (int i = 0; i < pcount; i++)
                            {
                                ushort d = currentFrameData.DepthImageMm[i];
                                currentFrameData.DepthImage[i] = (byte)Mathf.Clamp(d / 16, 0, 255);
                            }

                            if (RawDataLoggingFile != null && RawDataLoggingFile.CanWrite)
                                binaryFormatter.Serialize(RawDataLoggingFile, currentFrameData);

                            SetCurrentFrameData(ref currentFrameData);
                        }
                    }
                }
            }

            RawDataLoggingFile?.Close();
        }
        catch (Exception e)
        {
            Debug.Log($"Exception in background thread for device {id}: {e}");
            token.ThrowIfCancellationRequested();
        }
    }


    // -------------------------------------------------------------------------
    // PUBLIC: Depth → Color mapping (used by point cloud renderer)
    // -------------------------------------------------------------------------
    public bool DepthToColor(int px, int py, ushort depthMm, out int cx, out int cy)
    {
        return SensorCalibration.TransformDepthToColor(px, py, depthMm, out cx, out cy);
    }
}


// =======================================================================================================
//      EMBEDDED CALIBRATION EXTENSIONS — FIXED FOR YOUR SDK SIGNATURE
// =======================================================================================================
public static class CalibrationExtensions
{
    public static bool TransformDepthToColor(
        this Calibration calib,
        int depthX, int depthY, ushort depthMm,
        out int colorX, out int colorY)
    {
        // Depth pixel → Vector2 (System.Numerics)
        System.Numerics.Vector2 dp =
            new System.Numerics.Vector2(depthX, depthY);

        // Depth pixel → Depth 3D point
        System.Numerics.Vector3? depthPoint3D =
            calib.TransformTo3D(
                dp,
                depthMm,
                CalibrationDeviceType.Depth,
                CalibrationDeviceType.Depth);

        if (!depthPoint3D.HasValue)
        {
            colorX = colorY = -1;
            return false;
        }

        // Depth 3D → Color pixel using 3-argument overload
        System.Numerics.Vector2? colorPixel =
            calib.TransformTo2D(
                depthPoint3D.Value,
                CalibrationDeviceType.Depth,   // source space
                CalibrationDeviceType.Color);  // target space

        if (!colorPixel.HasValue)
        {
            colorX = colorY = -1;
            return false;
        }

        colorX = (int)colorPixel.Value.X;
        colorY = (int)colorPixel.Value.Y;
        return true;
    }
}

