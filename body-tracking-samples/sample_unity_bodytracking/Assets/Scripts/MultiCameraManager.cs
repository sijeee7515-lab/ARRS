using UnityEngine;
using System.Collections.Generic;

public class MultiCameraManager : MonoBehaviour
{
    [Header("Azure Kinect Device IDs (index in this array is camera index)")]
    public int[] deviceIds = new int[] { 0, 1 }; // example: 2 cameras

    [Header("TrackerHandlers for each camera (same order as deviceIds)")]
    public TrackerHandler[] trackerHandlers;

    private SkeletalTrackingProvider[] providers;
    private BackgroundData[] latestFrames;

    void Start()
    {
        int n = deviceIds.Length;
        providers = new SkeletalTrackingProvider[n];
        latestFrames = new BackgroundData[n];

        for (int i = 0; i < n; i++)
        {
            providers[i] = new SkeletalTrackingProvider(deviceIds[i]);
            latestFrames[i] = new BackgroundData();
        }
    }

    void Update()
    {
        for (int i = 0; i < providers.Length; i++)
        {
            var provider = providers[i];
            if (provider != null && provider.IsRunning)
            {
                if (provider.GetCurrentFrameData(ref latestFrames[i]))
                {
                    var frame = latestFrames[i];
                    if (frame.NumOfBodies > 0 && trackerHandlers != null && i < trackerHandlers.Length)
                    {
                        trackerHandlers[i].updateTracker(frame);
                    }
                }
            }
        }
    }

    // -----------------------------
    // NEW: EXPOSE PROVIDER BY INDEX
    // -----------------------------
    public SkeletalTrackingProvider GetProvider(int camIndex)
    {
        if (providers == null)
            return null;

        if (camIndex < 0 || camIndex >= providers.Length)
            return null;

        return providers[camIndex];
    }

    public BackgroundData GetLatestFrameForCamera(int camIndex)
    {
        if (camIndex < 0 || camIndex >= latestFrames.Length)
            return null;

        return latestFrames[camIndex];
    }

    void OnApplicationQuit()
    {
        if (providers != null)
        {
            foreach (var p in providers)
            {
                p?.Dispose();
            }
        }
    }
}
