
```bash
gz topic --list | grep gimbal
/gazebo/default/iris_demo/gimbal_roll_cmd
/gazebo/default/iris_demo/gimbal_tilt_cmd
/gazebo/default/iris_demo/gimbal_yaw_cmd


gz topic -p /gazebo/default/iris_demo/gimbal_roll_cmd  "gazebo.msgs.GzString" -m 'data: "0.0"'

gz topic -p /gazebo/default/iris_demo/gimbal_tilt_cmd  "gazebo.msgs.GzString" -m 'data: "0.0"'
gz topic -p /gazebo/default/iris_demo/gimbal_tilt_cmd  "gazebo.msgs.GzString" -m 'data: "1.575"'

gz topic -p /gazebo/default/iris_demo/gimbal_yaw_cmd  "gazebo.msgs.GzString" -m 'data: "0.0"'
gz topic -p /gazebo/default/iris_demo/gimbal_yaw_cmd  "gazebo.msgs.GzString" -m 'data: "0.707"'
gz topic -p /gazebo/default/iris_demo/gimbal_yaw_cmd  "gazebo.msgs.GzString" -m 'data: "-0.707"'

gz topic -p /gazebo/default/iris_demo/gimbal_roll_cmd  "gazebo.msgs.GzString" -m 'data: "0.0"'
gz topic -p /gazebo/default/iris_demo/gimbal_roll_cmd  "gazebo.msgs.GzString" -m 'data: "0.707"'
gz topic -p /gazebo/default/iris_demo/gimbal_roll_cmd  "gazebo.msgs.GzString" -m 'data: "-0.707"'

```