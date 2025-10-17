# HP60C YOLO Object Detection & Obstacle Avoidance

Real-time object detection and obstacle avoidance system using YDLidar HP60C RGB-D camera with YOLO and ROS2 Humble.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-green)
![YOLO](https://img.shields.io/badge/YOLO-v8-red)

## 🎯 Features

- **Real-time Object Detection**: YOLOv8-powered detection with 80+ object classes
- **3D Distance Measurement**: Precise distance calculation using HP60C depth data
- **Obstacle Avoidance**: Autonomous navigation with velocity commands
- **Visual Feedback**: Annotated video stream with bounding boxes and distance labels
- **Color-coded Warnings**: Green (safe) / Red (danger) based on distance thresholds

## 📹 Demo

The system:
- Detects objects using YOLO
- Measures distance using HP60C depth image
- Displays distance on video feed (e.g., "person 0.85 1.2m")
- Publishes velocity commands for robot control

## 🛠️ Hardware Requirements

- **Camera**: YDLidar HP60C RGB-D Camera
- **Computer**: Jetson Orin Nano (or any system running Ubuntu 22.04)

## 📦 Software Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

## 🚀 Prerequisites

- Ubuntu 22.04 with ROS2 Humble installed
- YDLidar HP60C camera connected via USB
- HP60C SDK installed and configured
- Required dependencies installed

## 🎮 Usage

### Quick Start

**Terminal 1: Start HP60C Camera**
```bash
cd ~/EaiCameraSDK/demo/linux_ros/ros2
source /opt/ros/humble/setup.bash
./run_ascamera_node.sh
```

**Terminal 2: Run YOLO Detection Node**
```bash
cd ~/robot_vision_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hp60c_vision yolo_obstacle_node
```

**Terminal 3: View Annotated Video**
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select topic: `/detections/image`

### Parameters

Configure the node with custom parameters:

```bash
ros2 run hp60c_vision yolo_obstacle_node --ros-args \
  -p yolo_model:=yolov8n.pt \
  -p confidence_threshold:=0.5 \
  -p obstacle_distance_threshold:=1.0 \
  -p safe_distance:=0.5
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `yolo_model` | `yolov8n.pt` | YOLO model file (n/s/m/l/x) |
| `confidence_threshold` | `0.5` | Detection confidence threshold |
| `obstacle_distance_threshold` | `1.0` | Max distance to consider as obstacle (meters) |
| `safe_distance` | `0.5` | Minimum safe distance (meters) |

## 📊 ROS2 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ascamera/camera_publisher/rgb0/image` | `sensor_msgs/Image` | RGB camera feed |
| `/ascamera/camera_publisher/depth0/image_raw` | `sensor_msgs/Image` | Depth image |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detections` | `vision_msgs/Detection2DArray` | Detected objects with bounding boxes |
| `/detections/image` | `sensor_msgs/Image` | Annotated RGB image with distances |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for navigation |

## 🤖 Robot Integration

The node publishes velocity commands on `/cmd_vel` for autonomous navigation:

- **No obstacles**: Move forward (0.2 m/s)
- **Obstacle detected**: Turn away from obstacle
- **Too close (< 0.5m)**: Back up (-0.1 m/s)

Integrate with your robot by subscribing to `/cmd_vel` or remapping the topic:

```bash
ros2 run hp60c_vision yolo_obstacle_node --ros-args \
  -r cmd_vel:=/my_robot/cmd_vel
```

## 📁 Project Structure

```
robot_vision_ws/
└── src/
    └── hp60c_vision/
        ├── hp60c_vision/
        │   ├── __init__.py
        │   └── yolo_obstacle_node.py
        ├── package.xml
        ├── setup.py
        └── README.md
```

## 🐛 Troubleshooting

### Camera not detected
```bash
# Check USB connection
lsusb | grep -i YD

# Verify camera topics
ros2 topic list | grep ascamera
```

### No distance shown in video
```bash
# Check depth image publishing
ros2 topic hz /ascamera/camera_publisher/depth0/image_raw

# View depth image directly
ros2 run rqt_image_view rqt_image_view
# Select: /ascamera/camera_publisher/depth0/image_raw
```

### Build errors with setuptools
```bash
# Fix setuptools version (required for ROS2 Humble)
pip3 install setuptools==58.2.0

# Clean and rebuild
cd ~/robot_vision_ws
rm -rf build install log
colcon build --packages-select hp60c_vision
```

### Point cloud issues
The node uses **depth image** (not point cloud) for distance calculations, ensuring proper pixel-to-distance mapping.

## 🎓 How It Works

1. **RGB Image Processing**: YOLO detects objects in the RGB camera stream
2. **Depth Mapping**: For each detected object, the bounding box center pixel is mapped to the depth image
3. **Distance Calculation**: The depth value at that pixel gives the distance in meters
4. **Visual Annotation**: 
   - Green box: Object at safe distance (> 0.5m)
   - Red box: Object too close (< 0.5m)
   - Label format: "class confidence distance" (e.g., "person 0.85 1.2m")
5. **Obstacle Classification**: Objects within threshold distance are marked as obstacles
6. **Navigation Logic**: Velocity commands generated based on closest obstacle position and distance

## 📊 Performance

- **Detection Speed**: ~30 FPS (YOLOv8n on Jetson Orin Nano)
- **Distance Accuracy**: ±2cm (HP60C specification)
- **Detection Range**: 0.2m - 4.0m
- **Field of View**: 73.8° (HP60C)

## 📝 Citation

If you use this project in your research, please cite:

```bibtex
@software{hp60c_yolo_vision,
  title={HP60C YOLO Object Detection & Obstacle Avoidance},
  author={Edwin C Daison},
  year={2025},
  url={https://github.com/infinity1350/robot_vision_ws
}
```

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📧 Contact

Your Name - Edwin C Daison

Project Link: [https://github.com/infinity1350/robot_vision_ws](https://github.com/infinity1350/robot_vision_ws)

## 🙏 Acknowledgments

- [YDLidar](https://www.ydlidar.com/) for HP60C RGB-D camera
- [Ultralytics](https://github.com/ultralytics/ultralytics) for YOLOv8
- [ROS2](https://docs.ros.org/en/humble/) community

## 🔗 Resources

- [YDLidar HP60C Documentation](https://www.ydlidar.com/service_support.html)
- [YDLidar HP60C Datasheet](https://www.ydlidar.com/dowfile.html?cid=29&type=5)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [CV Bridge Documentation](http://wiki.ros.org/cv_bridge)

## 🆕 Future Enhancements

- [ ] Multi-camera support
- [ ] Object tracking across frames
- [ ] 3D point cloud visualization
- [ ] Custom object training
- [ ] ROS2 bag recording/playback
- [ ] Real-time parameter tuning via rqt_reconfigure