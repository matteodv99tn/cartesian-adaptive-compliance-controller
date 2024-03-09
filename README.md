# Adaptive Cartesian Compliance Controller

Extension of the [cartesian compliance controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_compliance_controller) to work in an adaptive scenario.

## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration. Also see [the simulation config](../cartesian_controller_simulation/config/controller_manager.yaml) for further information.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    cartesian_adaptive_compliance_controller:
      type: cartesian_adaptive_compliance_controller/CartesianAdaptiveComplianceController

    # More controller instances here
    # ...

cartesian_adaptive_compliance_controller:
  ros__parameters:

    # Cartesian controller parameters
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

    command_interfaces:
      - position

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

    # Topics
    target_velocity_topic: "/target_velocity"
    target_wrench_topic: "/target_wrench"

    # Sampling period of the interface
    sampling_period: 0.002

    # Tank parameters
    tank:
        initial_state: 1.0
        minimum_energy: 0.4
        eta: -0.1

    # QP parameters
    Qp:
        F_min:  # Force lower bound
            - -15.0
            - -15.0
            - -15.0
            - -1.5
            - -1.5
            - -1.5
        F_max:  # Force upper bound
            - 15.0
            - 15.0
            - 15.0
            - 1.5
            - 1.5
            - 1.5
        K_min: # Minimum stiffness (diagonal)
            - 300.0
            - 300.0
            - 100.0
            - 30.0
            - 30.0
            - 10.0
        K_max: # Maximum stiffness (diagonal)
            - 1000.0
            - 1000.0
            - 1000.0
            - 100.0
            - 100.0
            - 100.0
        Q_weights: # Q weights
            - 3200.0
            - 3200.0
            - 3200.0
            - 3200.0
            - 3200.0
            - 3200.0
        R_weights: # R weights
            - 0.01
            - 0.01
            - 0.01
            - 0.01
            - 0.01
            - 0.01

# More controller specifications here
# ...
```
