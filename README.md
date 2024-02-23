# Adaptive Cartesian Compliance Controller

Extension of the [cartesian compliance controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_compliance_controller) to work in an adaptive scenario.

## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration. Also see [the simulation config](../cartesian_controller_simulation/config/controller_manager.yaml) for further information.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    cartesian_adaptive_compliance_controller:
      type: cartesian_adaptive_compliance_controller/CartesianAdaptiveComplianceController

    # More controller instances here
    # ...

cartesian_adaptive_compliance_controller:
  ros__parameters:
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "sensor_link"
    compliance_ref_link: "tool0"
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

    # Choose: position or velocity.
    command_interfaces:
      - position
        #- velocity

    stiffness:  # w.r.t. compliance_ref_link
        trans_x: 500
        trans_y: 500
        trans_z: 500
        rot_x: 20
        rot_y: 20
        rot_z: 20

    solver:
        error_scale: 0.5
        iterations: 1

    pd_gains:
        trans_x: {p: 0.05, d: 0.005}
        trans_y: {p: 0.05, d: 0.005}
        trans_z: {p: 0.05, d: 0.005}
        rot_x: {p: 1.5}
        rot_y: {p: 1.5}
        rot_z: {p: 1.5}


# More controller specifications here
# ...

```
