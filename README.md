Instructions for running image stitcher service (ros2 humble)

1. Run Camera Driver
(If you don't have usb_cam installed: sudo apt install ros-humble-usb-cam)
Bash

ros2 run usb_cam usb_cam_node_exe

2. Run This Node
(Note: Replace /image_raw with /camera/image_raw if your usb_cam uses that topic)
Bash

ros2 run final_panorama panorama_service_node --ros-args -p image_topic:="/image_raw"

3. Initial Setup
In a new terminal, publish the start angle 0.0:
Bash

ros2 topic pub --once /servo_angle std_msgs/msg/Float32 "{data: 0.0}"

4. Trigger the Service
In another terminal:
Bash

ros2 service call /trigger_panorama std_srvs/srv/Trigger {}

The node will now say "Captured Image @ 0.00 deg" and then "Waiting for angle 20.00...".

5. The Loop (Manual)

    Move Laptop ~20 degrees.

    Publish 20: ros2 topic pub --once /servo_angle std_msgs/msg/Float32 "{data: 20.0}"

    Node captures image, now waits for 40...

    Move Laptop ~20 degrees.

    Publish 40: ros2 topic pub --once /servo_angle std_msgs/msg/Float32 "{data: 40.0}"

    (Repeat until you publish 180.0)

Once 180 is reached, it will automatically stitch and save the result.
