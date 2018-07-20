# Nodes
- apriltags_locator.py
..- Node Name: apriltags_locator
..- Subscribes to:
....- tag_detections - > published by apriltags2_ros gives the pose of the tag
....- image_raw -> published by bebop_driver gives the video feed
..- Publishes to:
....- tag_location -> gives the position of the apriltag relative to the odom
....- tag_pixel_location -> gives the pixel coordinates of the apriltag
....- tag_found -> boolean revealing if the tag can be seen
....- image_overlay -> video feed with red circle over the apriltag
- controller.py
..- node name: controller_command
..- Subscribes to:
....- location_controller -> published by state machine. States which controller should be used for 3d space control(String)
....- rotation_controller -> published by state machine. States which controller should be used for rotation control (String)
....- tag_pixel_location -> published by apriltags_locator. Gives location of tag relative to camera viewport
....- tag_location -> published by apriltags_locator. Gives the location of the tag relative to odom
..- Publishes to:
....- cmd_vel_set -> flight commands for the bebop
states.py
..- node name: state_machine
..- subscribes to:
....- tag_found -> published by apriltags_locator; boolean if apriltag is found
....- operation_mode -> published by operator_command; ‘manual’ or ‘follow’
....- bebop/states/ardrone3/PilotingState/AlertStateCanged -> published by default; low battery or critical battery warnings
....- bebop/states/ARDrone3/PilotingState/FlyingStateChanged -> published by default; Used for transitions between landing and takeoff
..- publishes to:
....- rotation_controller -> controller type for rotation control
....- location_controller -> controller type for flight in 3d space
- operator.py

