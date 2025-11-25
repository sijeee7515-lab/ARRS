## Pipeline

performs multi-Azure Kinect extrinsic calibration using a single 5 cm ArUco marker, then records synchronized point clouds from all cameras, 
transforms them into a shared coordinate system, and outputs fused PLY frames suitable for visualization in MeshLab.

### Phase A — Calibration (marker present, static scene)

1. Start all Kinects with proper sync (master/subordinate).
2. Capture one frame per camera while the marker is visible and the scene is static.
3. For each camera:
   - Get color + depth frame.
   - Detect marker pose → get `rvec`, `tvec`.
   - Build a colored point cloud in that camera’s coordinate frame. 
4. Convert those poses into relative extrinsics to camera 0.
5. Refine with ICP.  
6. Save extrinsics to `extrinsics.txt`.  

### Phase B — Live capture (marker removed, moving objects)

1. On startup:
   - If extrinsics file exists → load extrinsics, skip calibration.  
   - Else → run calibration phase, then save extrinsics.
2. User passes `N` on command line: number of frames to fuse.
3. For `frame = 0..N-1`:
   - Grab synced frames from all cameras.
   - Build a colored point cloud for each camera.
   - Transform each cloud with that camera’s fixed extrinsic matrix into camera-0 (world) space.
   - Append them into one global cloud.
4. At the end, save global cloud as `output.ply`.

## Instructions

1. build the project using `Visual Studio`

2. the `.exe` will be in `marker/out/build/x64-Debug/bin/marker_calibration.exe`

3. `cd` to the `.exe` folder

4. run calibration first: 
	- for board: `./marker_calibration.exe calib_board <master device serial number>`

	- for cube: `./marker_calibration.exe calib <master device serial number>`

5. this will generates `extrinsics.txt` inside `/marker` folder

6. run capture then: `./marker_calibration.exe capture <number of frame to be captured>`

## Realtime

1. follow build and calib steps from Instructions

2. run realtime capture with: `./marker_calibration.exe realtime`

## Misc
- remove `extrinsics.txt` if you want to recalibrate the cameras
- `extrinsics.txt` is a great way to debug 
- calibration target should be placed in roughly 1 meter from the cameras and make sure all cameras can clearly see the marker in the middle without major angle distortion.
