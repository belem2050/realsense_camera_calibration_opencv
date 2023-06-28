## Behavior trees trigger
Test designed to to test behavior trees.

### Installation
Dependencies are part of the serena_setup_playbook so they should be present and built by default in the robot's docker.
If after sourcing the company software, packages are not detected by ros2, build the package:
```bash
colcon build --symlink-install --packages-up-to serena_end_to_end_tests
```
### Execution
   ##
1. Run dependencies on the robot in 3 separate terminals:
```bash
ros2 launch serena_bringup bringup_launch.py is_gazebo_simulation:=False
```

2. Run the test on the developer computer with docker and the company software:
```bash
ros2 run serena_end_to_end_tests behavior_tree_trigger_test --cycles <cycles [int]> --timeout_bt <[s]>  --behavior_tree_name <bt name [string]> 
```
#### Parameters:
| command              | description                                     |
|:--------------------:|:-----------------------------------------------:|
|-h, --help            | show help and exit                              |
|--timeout_bt          | the timeout                                     |
|--behavior_tree_name  | behavior tree name                              |
|--cycles              | the number of cycles                            |
### Output:
All results are stored in /bt_reports




-----------------------------------------------

## Square route Test

Test designed to test differents features of the robot. It groups :
   *  ***docking and undocking with electromagnets*** : tests docking and undocking by powering on electromagnets
   *  ***test_area_square_route*** : collects data such as odometries, velocity, battteries levels by following a square route by reaching out waypoints
   *  ***square_route_all_features*** : combining above tests plus trolley slippage test and collecting all data in one report
   


## Docking and undocking with electromagnets
Test designed to test electromagnets powering off and on. Until 29th of June, onl serena-13 has electromagnets and the right arduino code for this test.
For further tests, make sure that arduino code is upgraded. [Arduino code](https://github.com/cmrobotics/led_strips_serial_arduino)
   
https://github.com/cmrobotics/led_strips_serial_arduino Installation
Dependencies are part of the serena_setup_playbook so they should be present and built by default in the robot's docker.
If after sourcing the company software, packages are not detected by ros2, build the package:
```bash
colcon build --symlink-install --packages-up-to serena_end_to_end_tests
```
### Execution
   ##
1. Run dependencies on the robot in 2 separate terminals:
```bash
ros2 launch serena_bringup bringup_launch.py is_gazebo_simulation:=False
```

2. Run the test on the developer computer with docker and the company software:
```bash
ros2 run serena_end_to_end_tests docking_and_undocking_with_electromagnets_test --cycles <cycles [int]>
```
#### Parameters:
| command               | description                                      
|:---------------------:|:--------------------:|
| -h, --help            |show help and exit    |
| --cycles              |the number of cycles  |

 ##### Output:
   All results are stored in /ElectroMagnets_reports.


## License
Coalescent Mobile Robotics

   [Robot Framework]: <https://robotframework.org/>
