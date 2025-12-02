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

    //Color Image (BGRA Aligned to Depth)
    // BGRA — 4 bytes per pixel, aligned to depth resolution
    public byte[] ColorImageBgra { get; set; }
    public int ColorWidth { get; set; }
    public int ColorHeight { get; set; }

    // Number of detected bodies
    public ulong NumOfBodies { get; set; }

    public Body[] Bodies { get; set; }


    public BackgroundData(
        int initialDepthCapacity = 1024 * 1024,
        int maxBodiesCount = 20,
        int maxJointsSize = 100)
    {
        // Depth buffers
        DepthImage = new byte[initialDepthCapacity];
        DepthImageMm = new ushort[initialDepthCapacity];

        // Color buffer (BGRA: dynamically resized later if needed)
        ColorImageBgra = new byte[initialDepthCapacity * 4];

        // Body array
        Bodies = new Body[maxBodiesCount];
        for (int i = 0; i < maxBodiesCount; i++)
            Bodies[i] = new Body(maxJointsSize);
    }


    // Deserialization
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

        // Color
        ColorWidth = info.GetInt32("ColorWidth");
        ColorHeight = info.GetInt32("ColorHeight");
        ColorImageBgra = (byte[])info.GetValue("ColorImageBgra", typeof(byte[]));
    }

    //Serialization
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

        // Color image (BGRA)
        info.AddValue("ColorImageBgra", ColorImageBgra);
        info.AddValue("ColorWidth", ColorWidth);
        info.AddValue("ColorHeight", ColorHeight);
    }


    //Helpers
    public void EnsureDepthCapacity(int pixelCount)
    {
        if (DepthImage == null || DepthImage.Length < pixelCount)
            DepthImage = new byte[pixelCount];

        if (DepthImageMm == null || DepthImageMm.Length < pixelCount)
            DepthImageMm = new ushort[pixelCount];
    }

    public void EnsureColorCapacity(int pixelCount)
    {
        int needed = pixelCount * 4;
        if (ColorImageBgra == null || ColorImageBgra.Length < needed)
            ColorImageBgra = new byte[needed];
    }
}
