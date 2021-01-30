# t265_depth

This ROS node can perform the following operations:

1. Rectify fisheye images coming from the realsense t265 ros driver
2. Compute and publish disparity image based on the step 1
3. Compute and publish a pointcloud2 based on the step 2

## Note
- Assuming you correclty installed librealsense, get your specific camera parameters using **rs-enumerate-devices -c**
- If using low-power cpu, use lower resolution (see full, half, third in cfg folder). Remeber to adjust your parameters accordingly.

## Make it work
- Launch the t265 ros driver, be sure to publish images
- roslaunch t265_depth t265_depth.launch

# Parameters
- input_topic_left: topic name of the left camera
- input_topic_right: topic name of the right camera
- output_frame_id: name of the frame with respect to where publish the cloud/disparity image
- process_every_nth_frame: to save computation, skip frames
- param_file_path: path of your camera parameters
- scale: full, half of a third of the resolution (1/4 not usefull, image details lost)



