# 2D image processing in ROS for object orientation, features and DMC code reading

## Setup the environment
### Download Docker Container

Use Git to clone the docker container included in this repository.



Navigate into the cloned repository and use the convenience script provided to build the image of the docker container.

```
cd 2DImageprocessing/
```
```
./BUILD-DOCKER-CONTAINER.sh
```
Run the container via another convenience script in the current terminal.
```
./RUN-DOCKER-CONTAINER.sh
```


### 📺 **VideoPublisher2D.py**

📌 Overview:
This ROS 2 node publishes a static or live 2D image stream as a 
`sensor_msgs/Image` message. It serves as a virtual camera source for other 
modules such as visualization, monitoring, or image registration pipelines.

⚙️ Key Features:
- Publishes an image on a configurable topic (`camera_stream2D` by default).
- Embeds ROS message headers with timestamps and frame IDs.
- Configurable frequency and camera device index.
- Currently uses a static image source for proof-of-concept.

🧩 Parameters:
| Parameter    | Type   | Default              | Description |
|---------------|--------|----------------------|--------------|
| name          | str    | "VideoPublisher2D_node" | Node name |
| topic         | str    | "camera_stream2D"    | Output topic for images |
| frame_id      | str    | "2DCamera"           | Frame identifier |
| frequency     | float  | 20.0                 | Publishing frequency (Hz) |
| device_name   | int    | 2                    | Camera device index (reserved for live streaming) |

📤 Published Topic:
| Topic | Message Type | Description |
|--------|---------------|-------------|
| /camera_stream2D | sensor_msgs/msg/Image | Published grayscale image frames |

🧰 Dependencies:
- rclpy (ROS 2 client library for Python)
- sensor_msgs (Image message definitions)
- cv_bridge (ROS ↔ OpenCV conversion)
- OpenCV (Image loading and manipulation)

▶️ Usage:

    # Option 1: Run as standalone ROS node
    ros2 run ImgPro VideoPublisher2D.py

    # Option 2: Direct Python execution (for testing)
    python3 VideoPublisher2D.py

🧠 Notes:
- The node currently loads a static image from disk.
- Replace `cv2.imread()` with `cv2.VideoCapture()` for live camera input.
- Ideal as a mock camera node for testing downstream ROS image pipelines.

------------------------------------------------------------------------------

### 📺 **VideoMonitor2D.py**

 📌 Overview:
`VideoMonitor2D.py` is a **ROS 2 node** that subscribes to a 2D image topic (`sensor_msgs/Image`) and displays incoming frames in real time using OpenCV.  
It is designed to work seamlessly with `VideoPublisher2D.py`, serving as a **visualization and monitoring tool** to verify that image data is being published correctly in the ROS environment.



⚙️ Key Features:

- Subscribes to an image topic (default: `camera_stream2D`).
- Converts incoming ROS image messages to OpenCV format using `CvBridge`.
- Displays the live image stream in a resizable OpenCV window.
- Logs message reception to the ROS 2 console.
- Ready for adding text overlays (timestamp, frame ID, etc.) on each frame.



🧩 Parameters:

| Parameter | Type | Default | Description |
|------------|------|----------|-------------|
| `name` | `str` | `"VideoMonitor2D_node"` | Node name |
| `topic` | `str` | `"camera_stream2D"` | Topic name for subscribed image stream |



🧰 Dependencies:

- **rclpy** — ROS 2 Python client library  
- **sensor_msgs** — ROS 2 message definitions for image topics  
- **cv_bridge** — Conversion between ROS `Image` messages and OpenCV images  
- **opencv-python** — For visualization and frame processing  


▶️ Usage:

    # Option 1: Run as standalone ROS node
    ros2 run ImgPro VideoMonitor2D.py

    # Option 2: Direct Python execution (for testing)
    python3 VideoMonitor2D.py


------------------------------------------------------------------------------

### 🧮 **ImageRegistration.py**


📌 Overview:  
This ROS 2 node performs **feature-based image registration** between a reference image and a live 2D camera stream.  
It uses **ORB feature matching** and **RANSAC-based homography estimation** to determine object orientation and compute grip points for robotic manipulation tasks.

⚙️ Key Features:
- Subscribes to a live image topic (`camera_stream2D` by default).  
- Detects and matches ORB features between reference and sensed images.  
- Estimates homography transformation using RANSAC for robust alignment.  
- Draws bounding boxes, grip points, and origin markers on the registered image.  
- Displays the resulting alignment in real time using OpenCV visualization.  

🧩 Parameters:
| Parameter | Type | Default | Description |
|------------|------|----------|-------------|
| `name` | `str` | `"ImageRegistration_node"` | Node name |
| `topic` | `str` | `"camera_stream2D"` | Subscribed image topic for registration |

📤 Subscribed Topic:
| Topic | Message Type | Description |
|--------|---------------|-------------|
| `/camera_stream2D` | `sensor_msgs/msg/Image` | Incoming 2D image stream for registration and feature matching |

🧰 Dependencies:
- rclpy (ROS 2 client library for Python)  
- sensor_msgs (Image message definitions)  
- cv_bridge (ROS ↔ OpenCV conversion)  
- OpenCV (feature detection, matching, and visualization)  
- NumPy (matrix and geometric transformations)  
- Matplotlib (optional debugging or plotting)  

**Installation:**
```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge
pip install opencv-python numpy matplotlib



▶️ Usage:

## Option 1: Run as a ROS 2 node
ros2 run ImgPro ImageRegistration.py

# Option 2: Run directly for debugging
python3 ImageRegistration.py



## Results

### Matched Image with Annotations
<img src="https://raw.githubusercontent.com/vinayakabhatkale/2DImageprocessing/main/Results/tm.png" alt="Matched Image" width="600"/>

### Final Blurred Result
<img src="https://raw.githubusercontent.com/vinayakabhatkale/2DImageprocessing/main/Results/Result_blurred.png" alt="Result Blurred" width="600"/>

🟩 The white quadrilateral indicates the projected object boundary.
🟢 The green dots mark Grip_point1 and Grip_point2, used for potential robotic grasping.
🔴 The red text labels are dynamically drawn annotations added using OpenCV.



