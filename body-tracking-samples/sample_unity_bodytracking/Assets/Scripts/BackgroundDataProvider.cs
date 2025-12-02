using System;
using System.Threading;
using System.Threading.Tasks;

/// <summary>
/// Base class for all background data providers.
/// This class owns the double-buffer swap used to pass BackgroundData from
/// the background capture thread to the Unity main thread safely.
/// 
/// IMPORTANT:
///  - Depth extraction is NOT done here.
///  - Concrete subclasses (e.g., SkeletalTrackingProvider) must write real
///    Kinect depth into BackgroundData.DepthImageMm before calling SetCurrentFrameData().
/// </summary>
public abstract class BackgroundDataProvider : IDisposable
{
    // Double-buffer: the background thread writes into one buffer,
    // the main thread reads from the other.
    private BackgroundData m_frameBackgroundData = new BackgroundData();

    // Indicates if a new frame was produced since the last GetCurrentFrameData()
    private bool m_latest = false;

    private readonly object m_lockObj = new object();

    // Whether the provider is actively running
    public bool IsRunning { get; set; } = false;

    private CancellationTokenSource _cancellationTokenSource;
    private CancellationToken _token;

    /// <summary>
    /// Constructor: starts the background capture thread for this provider.
    /// </summary>
    public BackgroundDataProvider(int id)
    {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.quitting += OnEditorClose;
#endif
        _cancellationTokenSource = new CancellationTokenSource();
        _token = _cancellationTokenSource.Token;

        // Launch the background run loop for this provider
        Task.Run(() => RunBackgroundThreadAsync(id, _token));
    }

    private void OnEditorClose()
    {
        Dispose();
    }

    /// <summary>
    /// Implement this in subclasses.
    /// This runs on a background thread and must:
    ///   - Capture Kinect frames
    ///   - Fill out a BackgroundData instance (including DepthImageMm)
    ///   - Call SetCurrentFrameData(ref data)
    /// </summary>
    protected abstract void RunBackgroundThreadAsync(int id, CancellationToken token);

    /// <summary>
    /// Called from the background thread to publish a new frame.
    /// Swaps the reference so Unity main thread sees it on next Update().
    /// </summary>
    public void SetCurrentFrameData(ref BackgroundData currentFrameData)
    {
        lock (m_lockObj)
        {
            // Swap buffers
            var temp = currentFrameData;
            currentFrameData = m_frameBackgroundData;
            m_frameBackgroundData = temp;

            m_latest = true;
        }
    }

    /// <summary>
    /// Called from Unity main thread to fetch the latest frame.
    /// Returns true if new data is available.
    /// </summary>
    public bool GetCurrentFrameData(ref BackgroundData dataBuffer)
    {
        lock (m_lockObj)
        {
            // Swap buffers
            var temp = dataBuffer;
            dataBuffer = m_frameBackgroundData;
            m_frameBackgroundData = temp;

            bool result = m_latest;
            m_latest = false;

            return result;
        }
    }

    /// <summary>
    /// Stops the background thread on dispose or Editor exit.
    /// </summary>
    public void Dispose()
    {
#if UNITY_EDITOR
        UnityEditor.EditorApplication.quitting -= OnEditorClose;
#endif
        _cancellationTokenSource?.Cancel();
        _cancellationTokenSource?.Dispose();
        _cancellationTokenSource = null;
    }
}
