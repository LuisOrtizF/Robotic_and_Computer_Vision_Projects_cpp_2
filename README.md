# Robotic and Computer Vision Projects  with C++
---

This repository contains projects of computer vision tasks implemented with __*C++*__ libraries.

<div align="center">
<table> 
  <tr>
    <td> <img src="/Images/opencv_logo.png" width="125" height="125" alt="opencv_logo" /> </td>
    <td> <img src="/Images/pcl_logo.png" width="150" height="150" alt="pcl_logo" /> </td>
    <td> <img src="/Images/cmake_logo.png" width="200" height="80" alt="cmake_logo" /></td>
  </tr>
</table>
</div>

<div align="center">
<table> 
<tr> 
<td> 

+ **Kitti_Clouds_Viewer_and_Writer (leftCamRef)**: Visualize in 3D and write Kitti sequence point clouds in __*PCL*__ format (.pcd). The output point clouds are in the left camera coordinate system.
  + **inputs**: 
    + __*times_file_path*__: path of the directory where the times (*.txt*) file of the kitti sequences is saved
    + __*clouds_sequence_path*__: path of the directory where the kitti point clouds (.bin) are saved
    + __*max_frame*__: max mumber of frame in the sequence that we need visualize or write
    + __*write_flag (optional)*__: use only if you need save the point clouds into (.pcd) __*PCL*__ format.
  + **how use**: `./clouds_viewer_writer <times_file_path> <clouds_sequence_path> <max_frame> <write_flag>`
  + **example**: `./clouds_viewer_writer ~/data_odometry_gray/sequences/04/ ~/clouds/sequences/04/ 270 -w`
  + **output**: folder `~/build/sequence_clouds/` with the saved point cloud, only if you set __*write_flag*__ with `-w`

<tr>
<td> 

+ **Kitti_Clouds_Viewer_and_Writer (lidarRef)**: Visualize in 3D and write Kitti sequence point clouds in __*PCL*__ format (.pcd). The output point clouds are in the Velodyne lidar coordinate system.
  + **inputs**: 
    + __*times_file_path*__: path of the directory where the times (*.txt*) file of the kitti sequences is saved
    + __*clouds_sequence_path*__: path of the directory where the kitti point clouds (.bin) are saved
    + __*max_frame*__: max mumber of frame in the sequence that we need visualize or write
    + __*write_flag (optional)*__: use only if you need save the point clouds into (.pcd) __*PCL*__ format.
  + **how use**: `./clouds_viewer_writer <times_file_path> <clouds_sequence_path> <max_frame> <write_flag>`
  + **example**: `./clouds_viewer_writer ~/data_odometry_gray/sequences/04/ ~/clouds/sequences/04/ 270 -w`
  + **output**: folder `~/build/sequence_clouds/` with the saved point cloud, only if you set __*write_flag*__ with `-w`

<div align="center">
<table> 
  <tr>
    <td> <img src="/Images/kitti_clouds_viewer.gif" width="600" height="400" alt="kitti_clouds_viewer" /> </td>
  </tr>
</table>
</div>

<tr>
<td> 

+ **Kitti_Convert_StereoImages_to_PointClouds**: Convert kitti sequence gray images into __*PCL*__ point clouds (*.pcd*).
  + **inputs**: 
    + __*sequence_path*__: path of the directory where the kitti images are saved
    + __*num_frames*__: number of frames in the sequence that we need convert
    + __*write_flag (optional)*__: use only if you need save the point clouds into __*PCL*__ format.
  + **how use**: `./stereo2cloud <sequence_path> <num_frames> <write_flag (optional)>`
  + **example**: `./stereo2cloud ~/data_odometry_gray/sequences/04/ 270 -w`
  + **output**: folder `~/build/sequence_clouds/` with the saved point cloud, only if you set __*write_flag*__ with `-w`

<div align="center">
<table> 
  <tr>
    <td> <img src="/Images/stereo2cloud.gif" width="800" height="400" alt="stereo2cloud" /> </td>
  </tr>
</table>
</div>

<tr>
<td> 

+ **Kitti_Odometry_Viewer**: Visualization 3D of the Kitti cameras odometry
  + **inputs**: 
    + __*kitti_poses_file*__: poses file (*.txt*) of the kitti sequence
  + **how use**: `./kitti_odometry_viewer <kitti_poses_file>`
  + **example**: `./kitti_odometry_viewer ~/data_odometry_poses/poses/04.txt`
  + **output**: file `~/build/poses_quaternion.txt` with the kitti sequence poses (`tx` `ty` `tz` `qw` `qx` `qy` `qz`)

<tr>
<td> 

+ **Kitti_Save_Scales**: Compute the scales for a Kitti sequence. The scale is computed using the ground truth poses.
  + **inputs**: 
    + __*kitti_poses_file*__: poses file (*.txt*) of the kitti sequence to compute the scales
    + __*out_file_name*__: name of the output file (*.txt*) where the code save the kitti sequence scales
  + **how use**: `./kitti_scales <kitti_poses_file> <out_file_name>`
  + **example**: `./kitti_scales ~/data_odometry_poses/poses/04.txt ~/Desktop/scales_kitti_seq_04.txt`
  + **output**: (*.txt*) file with the saved scales

<tr>
<td> 

+ **Kitti_Sttiching**: Demonstrates how to use the Stitching module of __*OpenCV*__ to create a panorama using left and right images of the Kitti sequence.
  + **inputs**: 
    + __*sequence_path*__: path of the directory where the kitti images are saved
  + **how use**: `./panorama <sequence_path>`
  + **example**: `./panorama ~/data_odometry_gray/sequences/04/`
  + **output**: file `~/build/panorama.png` with the result of the sttiching beeteen left and right images.

<tr>
<td> 

+ **Plot_3D_Arrow**: Test __*PCL*__ visualizer plotting 3D arrows.
  + **how use**: `./plot_3D`

<tr>
<td> 

+ **Real_Time_Chessboard_Corners_Detection**: Detect and plot the corners of a chessboard in real-time using __*OpenCV*__ and a webcam.
  + **how use**: `./livecorners`

<tr>
<td> 

+ **Real_Time_Chessboard_Corners_Detection_SVO**: Detect and plot the corners of a chessboard in *(*.svo*) video using __*OpenCV*__.
  + **how use**: `./zedcorners`

<tr>
<td> 

+ **Record_SVO_from_ZED**: Record an (*.svo*) video using __*ZED*__ camera.
  + **inputs**: 
    + __*cam_resolution*__: __*ZED*__ camera resolutions `VGA` or `HD720` or `HD1080` or `HD2K`
    + __*output_dir*__: directory to save the (*.svo*) video and the camera intrinsic parameters
  + **how use**:`./record_svo_zed <cam_resolution> <output_dir>`
  + **example**: `./record_svo_zed HD720 ~/Desktop` 
  + **output**: a folder with the (*.svo*) video and a (*.txt*) file with the camera intrinsic parameters  

<tr>
<td> 

+ **Record_SVO_ZED+P3AT_Automatic**: Record a video using a __*ZED*__ camera mounted on a robot __*Pioneer 3AT*__. The robot moves into a straight line for some distance. When the robot starts, use the `arrow keys to drive it and the `spacebar` to stop. Assume you are ready to record a video, then press key `a` to switch to automatic mode. Press `Esc` to exit.
  + **inputs**: 
    + __*cam_resolution*__: __*ZED*__ camera resolutions `VGA` or `HD720` or `HD1080` or `HD2K`
    + __*output_dir*__: directory to save the (*.svo*) video
    + __*distance_in_mm*__: distance that the robot moves fordward in straight line
  + **how use**: `./record_svo_automatic <cam_resolution> <output_dir> <distance_in_mm> -rp /dev/ttyUSB0`
  + **example**: `./record_svo_automatic HD720 ~/Desktop/ 17000 -rp /dev/ttyUSB0`
  + **output**: a folder with the (*.svo*) video.

<tr>
<td> 

+ **Record_SVO_ZED+P3AT_Manual**: Record a video using a __*ZED*__ camera mounted on a robot __*Pioneer 3AT*__. You can drive the robot with the keyboard (arrows). Then when you are ready to record, press `s` and press `q` to exit when you finish.
  + **inputs**: 
    + __*cam_resolution*__: __*ZED*__ camera resolutions `VGA` or `HD720` or `HD1080` or `HD2K`
    + __*output_dir*__: directory to save the (*.svo*) video and camera parameters file.
  + **how use**: `./record_svo_manual <cam_resolution> <output_dir> -rp /dev/ttyUSB0`
  + **example**: `./record_svo_manual HD720 ~/Desktop/ -rp /dev/ttyUSB0`
  + **output**: a folder with the (*.svo*) video and camera parameters file *.yml*

<tr>
<td> 

+ **Registration_3D_using_Features**: Demonstrates how use the registration __*PCL*__ module. This code computes the relative transform between two point clouds.
  + **inputs**: 
    + __*source_cloud*__ : (*.pcd*) __*PCL*__ cloud
    + __*target_cloud*__ : (*.pcd*) __*PCL*__ cloud
  + **how use**:`./registration3D <source_cloud> <target_cloud>`

<tr>
<td> 

+ **Registration_Point_Clouds_using_SVD**: Compute the transformation between two point clouds using the SVD algorithm. Use this transformation to register the two-point clouds. Finally, compute the RMSE between the two-point clouds.
  + **how use**:`./svd`

<tr>
<td> 

+ **Save_ZED_All_Data**: Capture left, right images, depth, and a point cloud from the __*ZED*__.
  + **how use**:`./save_all <VGA or HD720 or HD1080 or HD2K>`
  + **outputs**:
    + __*/left/*__: folder with (*.png*) images
    + __*/right/*__: folder with (*.png*) images
    + __*/depth/*__: folder with (*.png*) depth map (in milieters)
    + __*/cloud/*__: folder with (*.pcd*) clouds
    + __*/poses.txt*__: camera poses referenced to the world coordinate system in Kitti format
    + __*/confidence.txt*__: depth map confidence
    + __*/times.txt*__: timestamps in milliseconds
    + __*/calibration.txt*__: intrinsics parameters of the camera

<tr>
<td> 

+ **Save_ZED_Data_Manual(keyboard)**: Capture left, right images, depth, and a point cloud from the __*ZED*__ using the keyboard. When you press `s` or  `S` the code saves data only of the current frame.
  + **how use**:`./save_all <VGA or HD720 or HD1080 or HD2K>`
  + **outputs**:
    + __*/left/*__: folder with (*.png*) images
    + __*/right/*__: folder with (*.png*) images
    + __*/depth/*__: folder with (*.png*) depth map (in milieters)
    + __*/cloud/*__: folder with (*.pcd*) clouds
    + __*/poses.txt*__: camera poses referenced to the world coordinate system in Kitti format
    + __*/confidence.txt*__: depth map confidence
    + __*/times.txt*__: timestamps in milliseconds
    + __*/calibration.txt*__: intrinsics parameters of the camera

<tr>
<td> 

+ **Save_ZED_Processing_Times(Images_Depth_Clouds)**: Carried out and analyzed of processing times of each capture step in the __*ZED*__ camera, i.e., the time that takes in capture an image, a depth map, and a point cloud.
  + **inputs**: 
    + __*zed_resolution*__ : `VGA` or `HD720` or `HD1080` or `HD2K`
    + __*frames_per_second*__ : Remember that VGA mode have 15, 30, 60 and 100 fps. HD720 mode have 15, 30 and 60 fps. HD1080 mode have 15 and 30 fps and HD2K mode have only 15 fps.
  + **how use**: `./save_processing_times <zed_resolution> <frames_per_second>`
  + **outputs**:
    + __*depth_var_xx*__: (*.png*) image that reflects the variance in each pixel of the all taked images
    + __*depth_mean_xx*__: (*.png*) image that reflects the mean in each pixel of all taked images for the analisys
    + __*depth_var_xx*__: (*.txt*) file with the variance in each pixel of all taked images for the analisys
    + __*depth_mean_xx*__: (*.txt*) file with the mean in each pixel of all taked images for the analisys

<tr>
<td> 

+ **Save_ZED_Undistorted_and_Distorted_Images**: 
  + **inputs**: 
    + __*zed_resolution*__ : `VGA` or `HD720` or `HD1080` or `HD2K`
  + **how use**: `./save_zed <zed_resolution>`
  + **outputs**:
    + __*/left/*__: folder with (*.png*) left rectifiqued (undistorted) images 
    + __*/right/*__: folder with (*.png*) right rectifiqued (undistorted) images 
    + __*/left_unrec/*__: folder with (*.png*) left unrectifiqued (distorted images 
    + __*/right_unrec/*__: folder with (*.png*) right unrectifiqued (distorted) images
    + __*/calibration.txt*__: (*.txt*) file with intrinsic parameters of left and right cameras
    
<tr>
<td> 

+ **Save_ZED(SVO)_Images_and_Poses_in_Kitti_Format**: Save left and right images form (*.svo*) video. Also, save camera poses in Kitti format. 
  + **inputs**: 
    + __*svo_video*__ : (*.svo*) file path
  + **how use**: `./save_zed_data <svo_video>`
  + **example**: `./save_zed_data ../Data/test.svo`
  + **outputs**:
    + `../Data/left/`: folder with saved left rgb images 
    + `../Data/right/`: folder with saved right rgb images 
    + `../Data/poses_world.txt`: file with __*ZED*__ poses referenced to world coordinate system
    + `../Data/poses_camera.txt`: file with __*ZED*__ poses referenced to camera coordinate system

<tr>
<td> 

+ **Save_ZED(SVO)_Poses_in_TUM_Format**: Save __*ZED*__ camera poses form (*.svo*) video. The poses are in TUM format. 
  + **inputs**: 
    + __*svo_video*__ : (*.svo*) file path
  + **how use**:`./save_zed_data <svo_video>`
  + **example**: `./save_zed_data ../Data/test.svo`
  + **outputs**:
    + `../Data/poses_zed.txt`: file with __*ZED*__ poses referenced to camera coordinate system

<tr>
<td> 

+ **Test_Kv1_using_OpenNI**: Test the Microsoft Kinect version 1 using OpenNI. 
  + **how use**:`./test_kv1`

<tr>
<td> 

+ **Test_Kv2_using_Libfreenect2**: Test the Microsoft Kinect version 2 using Libfreenect2. 
  + **how use**:`./test_kv1`

<tr>
<td> 

+ **Test_Kv2_using_OpenNI**: Test the Microsoft Kinect version 2 using OpenNI. 
  + **how use**:`./test_kv1`

<tr>
<td> 

+ **Visual_Monocular_SLAM_using_UcoSlam**: Compute visual monocular SLAM using UcoSlam (MODE_SLAM). 
  + **inputs**: 
    + __*video*__ : (*.avi*) video recorded with a monocular camera
    + __*camera_parameters*__ : *.yml* file with the intrinsics parameters of the camera
    + __*vocabulary*__ : *.fbow* vocabulary for the ORB keypoint descriptor
    + __*output_map*__ : *.map* result map with all data
  + **how use**: `./ucoslam_monocular <video> <camera_parameters> <vocabulary> <output_map>`
  + **example**: `./ucoslam_monocular ../Data/test.avi ../Data/mono_params.yml ../Data/orb.fbow ../Data/out.map`
  + **outputs**:
    + `../Data/poses_Kitti.txt`: file with camera poses referenced to camera coordinate system in Kitti format.

<div align="center">
<table> 
  <tr>
    <td> <img src="/Images/ucoslam_monocular_slam.gif" width="800" height="400" alt="ucoslam_monocular_slam" /> </td>
  </tr>
</table>
</div>

<tr>
<td> 

<tr>
<td> 

+ **Visual_Stereo_SLAM_using_UcoSlam**: Compute visual stereo SLAM using UcoSlam (MODE_SLAM) in a Kitti sequence. MODE_SLAM  is used to create and update maps. In that mode, the MapManager thread is activated to decide when keyframes are added, etc. 
  + **inputs**: 
    + __*video_left*__ : *.mp4* video recorded with left camera
    + __*video_right*__ : *.mp4* video recorded with right camera
    + __*stereo_calibration_file*__ : *.yml* file with the intrinsics and extrinsics parameters of the left and right cameras
    + __*kitti_times_file*__ : (*.txt*) kitti times file of the sequence
    + __*output_poses*__ : (*.txt*) file with the saved camera poses in TUM format
    + __*voc_path*__ : *.fbow* vocabulary for the ORB keypoint descriptor
    + __*ucoslam_params*__ : *.yml* parameters to setup UcoSlam
  + **how use**:`./slam_stereo <video_left> <video_right> <stereo_calibration_file> <kitti_times_file> <output_poses> <voc_path> <ucoslam_params>`
  + **example**: `./slam_stereo ../Data/cam0.mp4 ../Data/cam1.mp4 ../Data/stereo.yml ../Data/times.txt ../Data/poses_TUM.txt ../Data/orb.fbow ../Data/myparams.yml`
  + **outputs**:
    + `../Data/poses_TUM.txt`: file with camera poses referenced to camera coordinate system

<div align="center">
<table> 
  <tr>
    <td> <img src="/Images/ucoslam_stereo_slam.gif" width="800" height="400" alt="ucoslam_stereo_slam" /> </td>
  </tr>
</table>
</div>

</table>
</div>

## Installation:

+ Dependences (mandatory):
    + __*ZED*__ SDK 3.0
    + __*CUDA*__ 10.0
    + __*OpenCV*__ 3.4.1
    + __*Aruco*__ 3.0.12
    + __*PCL*__ 1.8
    + __*ARIA*__ or __*ARIACODA*__
    + __*UcoSLAM*__ 
    + __*Boost*__
    + __*Freenect2*__
    + __*Libviso2*__
    + __*Open-NI*__
    + __*Libusb*__ 1.0

+ Download any project and open a terminal _`ctrl+t`_:
    ```
    $ cd path 
    $ mkdir build & cd build 
    $ cmake .. 
    $ make
    ```

## NOTE:

| If you find any of these codes helpful, please share my __[GitHub](https://github.com/LuisOrtizF)__ and __*STAR*__ :star: this repository to help other enthusiasts to find these tools. Remember, the knowledge must be shared. Otherwise, it is useless and lost in time.|
| :----------- |
