# whi_qrcode_pose
Detect the QR code and estimate its position and orientation, code ArUco is supported. Advertise service for retrieving the encoded info and the offsets to the camera frame

![qrcode](https://github.com/xinjuezou-whi/whi_qrcode_pose/assets/72239958/bdfe4f2f-de9b-4512-8ce1-144df485ca33)

![aruco](https://github.com/user-attachments/assets/e9813b4f-dd35-4015-97ad-c20bf85222d2)

This package can be applied in visual guidance and pose alignment:

![align](https://github.com/xinjuezou-whi/whi_qrcode_pose/assets/72239958/f216d89c-f583-4cc9-b3e9-d8c0862a666f)

## Camera intrinsics
The accuracy of the QR code pose is mainly affected by the camera's intrinsics. Follow the [OpenCV's calibration guidance](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) to refer to the fundamental logic, or there are lots of [great tools](https://github.com/natowi/CameraCalibTools?tab=readme-ov-file) that can do the job

Once you get your camera's intrinsics, edit the "config.yaml" to record them:
```
intrinsic_projection: [385.75, 385.75, 323.12, 236.74] # focal length x, y, and optical center x, y
intrinsic_distortion: [0.0, 0.0, 0.0, 0.0]
```

## Dependency
### gcc9
```
sudo apt install gcc-9 g++-9
sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
```

### whi_interfaces
```
cd /<your_workspace>/src
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Advertised service
**qrcode_pose**(whi_interfaces::WhiSrvQrcode)

The argument: count in the request specifies the maximum number of estimated poses for average. The response from the server is filled with the offset(geometry_msgs/PoseStamped) to the camera frame and the encoded contents of the QR code

**qrcode_activate**(std_msgs::Bool)

To toggle the activity of the detection

## Image source
Currently, three image sources are supported: USB CAM stream, the message of sensor_msgs::Image, and images stored in a local folder

Refer to the parameters to specify the image source

## Parameters
```
whi_qrcode_pose:
  frame_id: camera
  source: device # topic/device/path
  topic:
    img_topic: /whi_pgnd_inspection/color_view
  device:
    cam_device: /dev/video0
  path:
    img_path: debug_images
  loop_hz: 20 # hz
  show_source_image: false
  show_detected_image: true
  activated_default: true
  frame_unit: meter # millimeter
  intrinsic_projection: [386.157, 386.157, 323.237, 239.697] # focal length x, y, and optical center x, y
  intrinsic_distortion: [0.0, 0.0, 0.0, 0.0]
  type: aruco # qr or aruco
  # ArUco
  aruco:
    dictionary: DICT_4X4_50
    marker_side_length: 0.165 # in meter
```

## Build
```
cd /<your_workspace>/src
git clone https://github.com/xinjuezou-whi/whi_qrcode_pose.git
cd ..
catkin build whi_qrcode_pose
source /<your_workspace>/devel/setup.bash
```

## Demo with locally stored images
For a quick demo, change the parameter "source" to "path", edit the "image_path" with your workspace like "/home/<your_workspace>/src/whi_qrcode_pose/debug_images/". And make sure parameters "show_source_image" and "show_detected_image" are both enabled, then run the following command:
```
roslaunch whi_qrcode_pose whi_qrcode_pose.launch
```
