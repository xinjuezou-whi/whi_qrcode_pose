# whi_qrcode_pose
Determine the QR code position and orientation. Advertise service for retrieving the encoded info and the offsets to the camera frame

![qrcode](https://github.com/xinjuezou-whi/whi_qrcode_pose/assets/72239958/bdfe4f2f-de9b-4512-8ce1-144df485ca33)

## Camera intrinsics
The accuracy of the QR code pose is mainly affected by the camera's intrinsics. Follow the [OpenCV's calibration guidance](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) to refer to the fundamental logic, or there are lots of [great tools](https://github.com/natowi/CameraCalibTools?tab=readme-ov-file) that can do the job

Once you get your camera's intrinsics, edit the "config.yaml" to record them:
```
intrinsic_projection: [385.75, 385.75, 323.12, 236.74] # focal length x, y, and optical center x, y
intrinsic_distortion: [0.0, 0.0, 0.0, 0.0]
```

## Dependency
```
cd /<your_workspace>/src
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Advertised service
**qrcode**(whi_interfaces::WhiSrvQrcode)

The argument: timeout in the request specifies the maximum duration to wait for the response from the server. The response from the server is filled with the offset(geometry_msgs/PoseStamped) to the camera frame and the encoded contents of the QR code

## Image source
Currently, three image sources are supported: USB CAM stream, the message of sensor_msgs::Image, and images stored in a local folder

Refer to the parameters to specify the image source

## Parameters
```
whi_qrcode_pose:
  frame_id: camera
  image_topic: image
  camera_device: /dev/video0
  image_path: debug_images
  source: path # topic/device/path
  loop_hz: 20 # hz
  show_source_image: true
  show_detected_image: true
  service: qrcode_pose
  frame_unit: meter # millimeter
  intrinsic_projection: [385.75, 385.75, 323.12, 236.74]
  intrinsic_distortion: [0.0, 0.0, 0.0, 0.0]
```

## Build
```
cd /<your_workspace>/src
git clone https://github.com/xinjuezou-whi/whi_qrcode_pose.git
cd ..
catkin build whi_qrcode_pose
source /<your_workspace>/devel/setup.bash
```

## Demo with local stored images
For a quick demo, change the parameter "source" to "path", edit the "image_path" with your workspace like "/home/<your_workspace>/src/whi_qrcode_pose/debug_images/". And make sure parameters "show_source_image" and "show_detected_image" are both enabled, then run the following command:
```
roslaunch whi_qrcode_pose whi_qrcode_pose.launch
```
