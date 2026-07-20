# ROS 2 C++ Real-Time Image Analytics & Camera Publisher

A simple, educational ROS 2 workspace built to practice C++ component nodes, multi-threaded execution, and real-time image processing with OpenCV. This project was created as a hands-on learning workspace to master C++ robotics software engineering principles.

The workspace consists of three packages:
1. **`pi_edge_detector`**: A multi-threaded C++ component node that receives image frames, applies a configurable edge detection filter (Canny, Sobel, or Laplacian), and publishes the processed output.
2. **`pi_yolov26_detector`**: A high-performance real-time object detection C++ component node that runs YOLO inference on CPU using OpenCV's DNN module, parsing class scores and bounding box coordinates, applying Non-Maximum Suppression (NMS), and publishing the annotated frames.
3. **`mac_camera_publisher`**: A simple utility node that captures raw webcam frames on macOS using OpenCV and publishes them to the ROS 2 graph for local testing.

---

## Key Concepts Practiced
*   **ROS 2 Composable Components**: Designing nodes as shared libraries loaded into a single `component_container_mt` process to allow zero-copy (pointer-based) frame transfer.
*   **Multi-Threaded Execution**: Running callbacks concurrently across a thread pool using the `MultiThreadedExecutor`.
*   **Zero-Copy cv_bridge Sharing**: Utilizing `cv_bridge::toCvShare` to borrow image message buffers without unnecessary memory copying.
*   **Contiguous Memory Transpositions**: Handling row-major OpenCV DNN outputs by transposing and formatting contiguous matrices (via `cv::transpose`) for correct `minMaxLoc` and bounding box coordinate mapping.
*   **Dynamic Parameter Reconfiguration**: Tuning thresholds and switching models or algorithms at runtime without restarting nodes.
*   **Chrono & Performance Tracking**: Custom $O(1)$ sliding window tracker (`PerfMonitor`) to measure throughput (FPS) and latency.

---

## Workspace Structure
```text
pi_video_analytics/
├── src/
│   ├── pi_edge_detector/       # Core edge detection C++ component package
│   ├── pi_yolov26_detector/    # Real-time YOLO object detection C++ component package
│   └── mac_camera_publisher/  # Webcam publisher utility for local testing
├── view_edges.py               # Lightweight Python viewer for edge detection
├── view_detections.py          # Lightweight Python viewer for YOLO detections
└── README.md
```

---

## Getting Started

### Prerequisites
*   ROS 2 (Humble or newer)
*   OpenCV 4
*   `cv_bridge` and `image_transport` ROS 2 packages
*   *Recommended for macOS:* Conda environment configured via RoboStack (e.g. `ros_env`, `openpark_env`).

### Building the Workspace
1. Clone or download the workspace.
2. Navigate to the root directory and source your ROS 2 environment (e.g. your Conda environment).
3. Build the workspace:
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

---

## Running the Nodes

### 1. Start the Camera Publisher
Launch the webcam publisher node (customize the camera index or FPS if needed):
```bash
source install/setup.zsh
ros2 run mac_camera_publisher camera_publisher_standalone --ros-args -p camera_index:=0 -p fps:=15.0
```

### 2. Run the Analytics Nodes

#### Option A: Edge Detector
```bash
source install/setup.zsh
ros2 launch pi_edge_detector edge_detector.launch.py
```
*Visualize with:*
```bash
python3 view_edges.py
```

#### Option B: YOLO Object Detector
If you haven't exported your model yet, activate the environment containing `ultralytics` (e.g. `openpark_env`) and run the export script to download and convert the weights to ONNX:
```bash
python3 src/pi_yolov26_detector/scripts/export_yolov26.py --model yolov8n.pt
```
Then, launch the YOLO detector node:
```bash
source install/setup.zsh
ros2 launch pi_yolov26_detector yolov26_detector.launch.py
```
*Visualize with:*
```bash
python3 view_detections.py
```

---

## Dynamic Parameter Tuning
Use the ROS 2 command-line interface to update parameters on the fly:

**Edge Detector Tuning:**
```bash
# Switch to Sobel filter
ros2 param set /edge_detector algorithm sobel

# Tune Canny thresholds
ros2 param set /edge_detector canny_low_threshold 80
ros2 param set /edge_detector canny_high_threshold 180
```

**YOLO Detector Tuning:**
```bash
# Adjust confidence threshold
ros2 param set /yolov26_detector confidence_threshold 0.35

# Throttle maximum inference frame rate
ros2 param set /yolov26_detector max_fps 5.0
```
