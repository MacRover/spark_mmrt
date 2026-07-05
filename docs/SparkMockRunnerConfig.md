# Spark Mock Runner Configuration

This document describes the configurable parameters used by `spark_mock_runner.py` and the structure shown in `config/default_runner.yaml`.

## Top-level settings

| Key | Type to set | Description |
| --- | --- | --- |
| `log_level` | String | Controls runner logging verbosity. Supported values in the current runner are `debug`, `info`, `warning`, `error`, and `critical`. |
| `mock_runners` | Array of objects | List of mock CAN runner instances to start. Each entry creates one `SparkMockRunner`. |

## `mock_runners[]`

Each item in `mock_runners` configures one CAN bus runner.

| Key | Type to set | Description |
| --- | --- | --- |
| `name` | String | Human-readable name to distinguish between log messages of different runners. |
| `can_interface` | String | SocketCAN interface name to bind to, such as `vcan0` or `can0`. |
| `can_bitrate` | Integer | CAN bus bitrate in bits per second. |
| `filters` | Array of objects | CAN receive filters applied to the bus. Each filter entry must provide `can_id` and `can_mask`. |
| `can_nodes` | Array of objects | List of simulated SparkMAX nodes attached to this runner. |

## `mock_runners[].filters[]`

Each filter is passed directly to the CAN backend.

| Key | Type to set | Description |
| --- | --- | --- |
| `can_id` | Integer | Arbitration ID value to match. YAML can use decimal or hex values, such as `0x02050000`. |
| `can_mask` | Integer | Bit mask used with `can_id` to select the CAN frames that should be received. |

## `mock_runners[].can_nodes[]`

Each entry defines one simulated SparkMAX device.

| Key | Type to set | Description |
| --- | --- | --- |
| `name` | String | Human-readable node name to distinguish between log messages of different nodes. |
| `device_id` | Integer | CAN device ID for the simulated node. |
| `max_velocity` | Number | Maximum simulated velocity used by the control model. |
| `max_acceleration` | Number | Maximum simulated acceleration used by the position control model. |
| `status_rates` | Array of numbers | Status frame update rates, one value per status frame index. Assigning a value of `0` effectively disables that status frame. |
| `listen_for_heartbeat` | Boolean | Enables heartbeat monitoring for the node. When `true`, the node expects primary heartbeat traffic before continuing motion. |
| `flash` | Object | Persistent per-node parameter values used to initialize the simulated device state. |

## `mock_runners[].can_nodes[].flash`

These values seed the simulated flash-backed parameters for each node.

| Key | Type to set | Description |
| --- | --- | --- |
| `position_factor` | Number | Scale factor applied to position values. |
| `velocity_factor` | Number | Scale factor applied to velocity values. |
| `abs_position_factor` | Number | Scale factor for the absolute encoder position. |
| `abs_velocity_factor` | Number | Scale factor for the absolute encoder velocity. |
| `control_type` | ControlType | Default control mode for the node. |
| `sensor_type` | SensorType | Default sensor source used by the node. |

## `ControlType` Enum Values

| Value | Meaning |
| --- | --- |
| `0` | Duty cycle |
| `1` | Velocity |
| `2` | Voltage |
| `3` | Position |
| `4` | Smart motion |
| `5` | Smart velocity |
| `6` | Max motion position |
| `7` | Max motion velocity |

## `SensorType` Enum Values

| Value | Meaning |
| --- | --- |
| `0` | None |
| `1` | Main encoder |
| `2` | Analog |
| `3` | Alternate encoder |
| `4` | Duty cycle |
