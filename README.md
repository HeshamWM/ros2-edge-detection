# ROS 2 C++ Edge Detection & Camera Publisher

A simple, educational ROS 2 workspace built to practice C++ component nodes, multi-threaded execution, and real-time image processing with OpenCV. This was created as a hands-on learning project to prepare for C++ robotics software engineering interviews.

The workspace consists of two packages:
1. **`pi_edge_detector`**: A multi-threaded component node that receives image frames, applies a configurable edge detection filter (Canny, Sobel, or Laplacian), and publishes the processed output.
2. **`mac_camera_publisher`**: A simple node that captures raw webcam frames on macOS using OpenCV and publishes them to the ROS 2 graph for local testing.

---

## Key Concepts Practiced
*   **ROS 2 Composable Components**: Designing nodes as shared libraries loaded into a single `component_container_mt` process to allow zero-copy (pointer-based) frame transfer.
*   **Multi-Threaded Execution**: Running callbacks concurrently across a thread pool using the `MultiThreadedExecutor`.
*   **Zero-Copy cv_bridge Sharing**: Utilizing `cv_bridge::toCvShare` to borrow image message buffers without memory allocation.
*   **Dynamic Parameter Reconfiguration**: Tuning edge detection thresholds and switching algorithms at runtime without restarting nodes.
*   **Chrono & Performance Tracking**: Custom $O(1)$ sliding window tracker (`PerfMonitor`) to measure throughput (FPS) and latency.

---

## Workspace Structure
```text
pi_video_analytics/
├── src/
│   ├── pi_edge_detector/       # Core edge detection C++ component package
│   └── mac_camera_publisher/  # Webcam publisher utility for local testing
├── view_edges.py               # Lightweight Python viewer using cv2.imshow
└── README.md
```

---

## Getting Started

### Prerequisites
*   ROS 2 (Humble or newer)
*   OpenCV 4
*   `cv_bridge` and `image_transport` ROS 2 packages
*   *Recommended for macOS:* Conda environment configured via RoboStack.

### Building the Workspace
1. Clone or download the workspace.
2. Navigate to the root directory and source your ROS 2 environment.
3. Build the workspace:
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

### Running the Nodes

1. **Start the Camera Publisher** (Terminal 1):
   ```bash
   source install/setup.zsh
   ros2 run mac_camera_publisher camera_publisher_standalone --ros-args -p camera_index:=0 -p fps:=15.0
   ```

2. **Launch the Edge Detector** (Terminal 2):
   ```bash
   source install/setup.zsh
   ros2 launch pi_edge_detector edge_detector.launch.py
   ```

3. **Visualize the Output**:
   *   Option A: Start the lightweight python script in the workspace root:
       ```bash
       python3 view_edges.py
       ```
   *   Option B: Open RViz 2 and subscribe to `/edge_detector/edges`:
       ```bash
       rviz2
       ```

### Dynamic Tuning
Use the ROS 2 command-line interface to update parameters on the fly:
```bash
# Switch to Sobel filter
ros2 param set /edge_detector algorithm sobel

# Tune Canny thresholds
ros2 param set /edge_detector canny_low_threshold 80
ros2 param set /edge_detector canny_high_threshold 180
```
