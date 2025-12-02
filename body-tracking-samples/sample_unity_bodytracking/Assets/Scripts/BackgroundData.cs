using System;
using System.Runtime.Serialization;

// Class which contains all data sent from background thread to main thread.
[Serializable]
public class BackgroundData : ISerializable
{
    // Identify which Azure Kinect this frame came from
    public int SensorId { get; set; }

    // Timestamp of current data
    public float TimestampInMs { get; set; }

    // LEGACY 8-bit depth image (Kept only for backward compatibility)
    public byte[] DepthImage { get; set; }

    // TRUE 16-bit depth image in millimeters
    public ushort[] DepthImageMm { get; set; }

    public int DepthImageWidth { get; set; }
    public int DepthImageHeight { get; set; }
    public int DepthImageSize { get; set; }

    // ----------- NEW: COLOR IMAGE SUPPORT -----------------
    public byte[] ColorImage { get; set; }      // BGRA (K4A format)
    public int ColorWidth { get; set; }
    public int ColorHeight { get; set; }
    // -------------------------------------------------------

    // Number of detected bodies
    public ulong NumOfBodies { get; set; }

    public Body[] Bodies { get; set; }

    public BackgroundData(
        int maxDepthImageSize = 1024 * 1024 * 3,
        int maxBodiesCount = 20,
        int maxJointsSize = 100)
    {
        DepthImage = new byte[maxDepthImageSize];
        DepthImageMm = new ushort[maxDepthImageSize];

        // Allocate color buffer (RGBA per pixel worst case)
        ColorImage = new byte[maxDepthImageSize * 4];

        Bodies = new Body[maxBodiesCount];
        for (int i = 0; i < maxBodiesCount; i++)
            Bodies[i] = new Body(maxJointsSize);
    }

    public BackgroundData(SerializationInfo info, StreamingContext context)
    {
        SensorId = info.GetInt32("SensorId");
        TimestampInMs = info.GetSingle("TimestampInMs");

        DepthImageWidth = info.GetInt32("DepthImageWidth");
        DepthImageHeight = info.GetInt32("DepthImageHeight");
        DepthImageSize = info.GetInt32("DepthImageSize");

        NumOfBodies = (ulong)info.GetValue("NumOfBodies", typeof(ulong));

        Bodies = (Body[])info.GetValue("Bodies", typeof(Body[]));

        DepthImage = (byte[])info.GetValue("DepthImage", typeof(byte[]));
        DepthImageMm = (ushort[])info.GetValue("DepthImageMm", typeof(ushort[]));

        // NEW
        ColorWidth = info.GetInt32("ColorWidth");
        ColorHeight = info.GetInt32("ColorHeight");
        ColorImage = (byte[])info.GetValue("ColorImage", typeof(byte[]));
    }

    public void GetObjectData(SerializationInfo info, StreamingContext context)
    {
        info.AddValue("SensorId", SensorId);
        info.AddValue("TimestampInMs", TimestampInMs);

        info.AddValue("DepthImageWidth", DepthImageWidth);
        info.AddValue("DepthImageHeight", DepthImageHeight);
        info.AddValue("DepthImageSize", DepthImageSize);
        info.AddValue("NumOfBodies", NumOfBodies);

        // Bodies
        Body[] validBodies = new Body[NumOfBodies];
        for (int i = 0; i < (int)NumOfBodies; i++)
            validBodies[i] = Bodies[i];
        info.AddValue("Bodies", validBodies);

        // Depth images
        info.AddValue("DepthImage", DepthImage);
        info.AddValue("DepthImageMm", DepthImageMm);

        // -------- NEW: SERIALIZE COLOR -----------
        info.AddValue("ColorImage", ColorImage);
        info.AddValue("ColorWidth", ColorWidth);
        info.AddValue("ColorHeight", ColorHeight);
        //-------------------------------------------
    }
}
