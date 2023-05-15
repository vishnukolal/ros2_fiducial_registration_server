# ROS2 Fiducial Registration Server

##Dependencies

This package made using ROS2 Galactic and depends on TF2 and Eigen.
Ensure that Eigen is installed https://eigen.tuxfamily.org/index.php?title=Main_Page

##Install

1. Clone the repo

```
git clone https://github.com/vishnukolal/rsp.git
```

2. Source the directory

```
source install/setup.bash
```

3. Run rosdep

```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

4. Build the remaining packages

```
colcon build
```

##Usage

To run the server:

```
ros2 run registration_service reg_svr
```

To make a call to the server use the Registration.srv message in registration_msgs package 
OR 
Create an object of registration_client in your source code and use the `result = registration_client.send_request(fixed_frames, recorded_frames, marker_offset)` method to send the two sets of points and an optional marker offset to the server. Use `spin_until_future_complete(result)` to wait for the server to send the result. The result transformation can be accessed through TF or directly by using the `registration_client.get_registration(pose, error, status)` method.


To test the server:

```
ros2 run registration_service reg_cli
```
