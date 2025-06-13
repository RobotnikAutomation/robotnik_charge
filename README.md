# robotnik_charge

This packages creates an orchestor for automatic charging to the docking station of Robotnik robots.

> [!IMPORTANT]
> It depends on [charge_manager](https://github.com/RobotnikAutomation/battery_manager/tree/jazzy-devel) and [robotnik_navigation_msgs](https://github.com/RobotnikAutomation/robotnik_interfaces).

---

## Usage

```
ros2 launch robotnik_charge robotnik_charge_launch.py
```

---

### Charge

```
ros2 actions send_goal /robot/charge robotnik_navigation_msgs/action/Charge "dock_frame='' robot_dock_frame='' dock_offset= retries= "
```

The charge action depends on 4 arguments:

- *dock_frame*: Frame reference to where the robot should dock, it's usually the docking station marker frame.
- *robot_dock_frame*: Base frame of the robot to align to *dock_frame*, it's usually the docking contact frame.
- *dock_offset*: The distance that the docking action should stop before the docking station.
- *retries*: The number of maximum retries that the charge action must do.

The charge process follows the next steps:

#### Before accepting the process it is check if:
1. If the robotnik_charge is in the correct state (not doing any other action)
2. There is battery information
3. The robot is not already charging
4. The frame exists
5. The dock and move actions are correct

If there is any issue with this, the action will be rejected.

#### Charge action
1. If the robot has safety laser, change to the mode during action**(Still pending to be implemented)**
2. Send dock goal with offset.
3. Once dock is compleated, send move goal.
4. Once move is compleated, activate relay.
5. Wait for charge detected.
6. If not charging, move backwards and repeat.
7. If all tries are compleated, wait forced to 10s.
8. If not charging, move backwards and abort.

If during the process there is any failure, the action is aborted.


### Uncharge

```
ros2 actions send_goal /robot/uncharge robotnik_navigation_msgs/action/Uncharge {}
```

The uncharge action doesn't need any argument.

The charge process follows the next steps:

#### Before accepting the process it is check if:
1. If the robotnik_charge is in the correct state (not doing any other action)
2. There is battery information
3. The robot is already charging

If there is any issue with this, the action will be rejected.

#### Uncharge action

1. Deactivate relay.
2. If the robot has safety laser, change to the mode during action **(Still pending to be implemented)**
3. Move backwards.
4. Rotate.
5. If the robot has safety laser, change to the mode after action **(Still pending to be implemented)**

If during the process there is any failure, the action is aborted.

### Parameters

There is a [config file](/config/robotnik_charge_params.yaml) where the parameters are configured before launching the node.

These are the parameters:

  * **rate**:
    * type: int
    * default_value: 10
    * read_only: true
    * description: "Frequency to run the action. default: 10"

  * **step_timeout**:
    * type: double
    * default_value: 30.0
    * read_only: true
    * description: "Seconds of timeout that each step of the charging or uncharging method must last. default: 30.0"

  * **dock_action**:
    * type: string
    * default_value: "/smooth_drive/dock"
    * description: "Name of dock action. default: /smooth_drive/dock"

  * **move_action**:
    * type: string
    * default_value: "/move"
    * description: "Name of move action. default: /move"

  * **charge_contact_distance_from_marker**:
    * type: double
    * default_value: 0.14
    * read_only: true
    * description: "Distance from marker to charge contact in docking station. default: 0.14"

  * **timeout_charging_detection**:
    * type: int
    * default_value: 1
    * read_only: true
    * description: "Seconds to wait to detect charge. default: 1"

  * **has_safety_lasers**:
    * type: bool
    * default_value: false
    * description: "If the robot has safety lasers, the safety mode must change. default: false"

  * **laser_mode_during_action**:
    * type: string
    * default_value: "docking"
    * description: "Laser mode during the charge or uncharge action. default: docking"

  * **laser_mode_after_action**:
    * type: string
    * default_value: "standard"
    * description: "Laser mode at the end of the uncharge action. default: standard"

  * **step_back_distance**:
    * type: double
    * default_value: 0.5
    * description: "Distance to step back during uncharge. default: 0.5"
 
  * **rotation**:
    * type: double
    * default_value: 0.0
    * description: "Rotation to apply after uncharge step back. default: 0.0"

  * **max_velocity_x**:
    * type: double
    * default_value: 1.0
    * description: "Maximum velocity in x axis. default: 1.0"

  * **max_velocity_y**:
    * type: double
    * default_value: 0.0
    * description: "Maximum velocity in y axis. default: 0.0"

  * **max_velocity_yaw**:
    * type: double
    * default_value: 1.0
    * description: "Maximum velocity in yaw. default: 1.0"
