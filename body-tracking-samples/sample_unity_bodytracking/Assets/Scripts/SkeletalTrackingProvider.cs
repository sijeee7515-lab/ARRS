using Microsoft.Azure.Kinect.BodyTracking;
using Microsoft.Azure.Kinect.Sensor;
using System;
using System.IO;
using System.Threading;
using UnityEngine;

public class SkeletalTrackingProvider : BackgroundDataProvider
{
    public Device SensorDevice { get; private set; }
    public Calibration SensorCalibration { get; private set; }

    private Transformation transformation;
    private bool imuStarted = false;

    bool readFirstFrame = false;
    TimeSpan initialTimestamp;

    public Stream RawDataLoggingFile = null;

    private System.Runtime.Serialization.Formatters.Binary.BinaryFormatter binaryFormatter =
        new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();

    public SkeletalTrackingProvider(int id) : base(id)
    {
        Debug.Log("SkeletalTrackingProvider constructor for device " + id);
    }

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
                    ColorFormat = ImageFormat.ColorBGRA32,
                    ColorResolution = ColorResolution.R720p,
                    DepthMode = DepthMode.NFOV_Unbinned,
                    SynchronizedImagesOnly = true,
                    WiredSyncMode = WiredSyncMode.Standalone,
                });

                Debug.Log($"Open K4A device successful. id {id} sn:{device.SerialNum}");

                SensorCalibration = device.GetCalibration();
                transformation = SensorCalibration.CreateTransformation();

                using (Tracker tracker = Tracker.Create(SensorCalibration, new TrackerConfiguration()
                {
                    ProcessingMode = TrackerProcessingMode.Gpu,
                    SensorOrientation = SensorOrientation.Default
                }))
                {
                    Debug.Log("Body tracker created.");

                    while (!token.IsCancellationRequested)
                    {
                        // Get synchronized capture from device
                        Capture rawCapture = device.GetCapture();
                        tracker.EnqueueCapture(rawCapture);

                        using (Frame frame = tracker.PopResult(TimeSpan.Zero, throwOnTimeout: false))
                        {
                            if (frame == null)
                            {
                                Debug.Log("Pop result from tracker timeout!");
                                rawCapture.Dispose();
                                continue;
                            }

                            IsRunning = true;

                            currentFrameData.SensorId = id;
                            currentFrameData.NumOfBodies = frame.NumberOfBodies;

                            for (uint i = 0; i < currentFrameData.NumOfBodies; i++)
                            {
                                currentFrameData.Bodies[i].CopyFromBodyTrackingSdk(
                                    frame.GetBody(i), SensorCalibration);
                            }

                            Image depthImage = rawCapture.Depth;
                            if (depthImage == null)
                            {
                                Debug.LogWarning($"[Depth] rawCapture.Depth is null for dev {id}");
                                rawCapture.Dispose();
                                continue;
                            }

                            if (!readFirstFrame)
                            {
                                readFirstFrame = true;
                                initialTimestamp = depthImage.DeviceTimestamp;
                            }

                            currentFrameData.TimestampInMs =
                                (float)(depthImage.DeviceTimestamp - initialTimestamp).TotalMilliseconds;

                            int width  = depthImage.WidthPixels;
                            int height = depthImage.HeightPixels;
                            int pixelCount = width * height;

                            currentFrameData.DepthImageWidth  = width;
                            currentFrameData.DepthImageHeight = height;
                            currentFrameData.DepthImageSize   = pixelCount;

                            currentFrameData.EnsureDepthCapacity(pixelCount);

                            //Depth 16-bit (mm)
                            var depthSpan = depthImage.GetPixels<ushort>().Span;
                            for (int i = 0; i < pixelCount; i++)
                                currentFrameData.DepthImageMm[i] = depthSpan[i];

                            //8-bit
                            for (int i = 0; i < pixelCount; i++)
                            {
                                ushort d = currentFrameData.DepthImageMm[i];
                                currentFrameData.DepthImage[i] = (byte)Mathf.Clamp(d / 16, 0, 255);
                            }

                            //Color to Depth (using Capture overload, like sample)
                            try
                            {
                                if (transformation != null)
                                {
                                    using (Image colorOnDepth = transformation.ColorImageToDepthCamera(rawCapture))
                                    {
                                        int cwidth  = colorOnDepth.WidthPixels;
                                        int cheight = colorOnDepth.HeightPixels;
                                        int cpx = cwidth * cheight;

                                        currentFrameData.ColorWidth  = cwidth;
                                        currentFrameData.ColorHeight = cheight;

                                        currentFrameData.EnsureColorCapacity(cpx);

                                        var colorSpan = colorOnDepth.GetPixels<BGRA>().Span;

                                        for (int i = 0; i < cpx; i++)
                                        {
                                            BGRA c = colorSpan[i];
                                            int idx = i * 4;

                                            currentFrameData.ColorImageBgra[idx + 0] = c.B;
                                            currentFrameData.ColorImageBgra[idx + 1] = c.G;
                                            currentFrameData.ColorImageBgra[idx + 2] = c.R;
                                            currentFrameData.ColorImageBgra[idx + 3] = c.A;
                                        }
                                    }
                                }
                            }
                            catch (Exception e)
                            {
                                Debug.LogWarning($"[RGB] Failed to extract color for dev {id}: {e.Message}");
                            }

                            if (RawDataLoggingFile != null && RawDataLoggingFile.CanWrite)
                                binaryFormatter.Serialize(RawDataLoggingFile, currentFrameData);

                            SetCurrentFrameData(ref currentFrameData);
                        }

                        rawCapture.Dispose();
                    }

                    Debug.Log("Disposing of tracker for device " + id);
                }
            }

            if (RawDataLoggingFile != null)
                RawDataLoggingFile.Close();
        }
        catch (Exception e)
        {
            Debug.Log($"Exception in background thread for device {id}: {e.Message}");
            token.ThrowIfCancellationRequested();
        }
    }
}
