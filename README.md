# 2DImageoprocessing pipeline for object orientation,DMC reader, ORB features and bounding box

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
**VideoPublisher2D.py**

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
"""
