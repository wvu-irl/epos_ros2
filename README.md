# epos_ros2
ROS2 interface for the EPOS controllers/Maxon motors

## Notes
Currently, the functionality is contained in the EPOSWrapper header and folder. Still need to develop and test interfaces

## Install 
Expects EposCmdLib 6.6.1.0, this can be modified in CMakeLists
EposCmdLib 6.6.1.0 can be installed by running `bash install_files/install_epos_cmd_lib.bash`

## TODO:
Add EPOS Interface to do fault handling this will likely involve integration with the new system manager package (private for now)

Make some more example code

Link with Read the Docs

Verify Functionality of the Following code (to be done the week of Aug 1):

```
EPOSWrapper();

EPOSWrapper(rclcpp::Node *_node_ptr, EPOSParams _params);

~EPOSWrapper();
```


**INITIALIZATION**

```
int open_devices();

int close_devices();
```


**CONFIGURATION**

```
int set_modes(const std::vector<std::string> &_motors,const std::vector<OperationMode> &_modes);

int set_modes(const std::vector<std::string> &_motors, OperationMode _mode);

int set_mode(const std::string &_motor, OperationMode _mode);

int reset_devices(const std::vector<std::string> &_motors);

int reset_device(const std::string &_motor);

int set_states(const std::vector<std::string> &_motors, const std::vector<DeviceState> &_states);

int set_states(const std::vector<std::string> &_motors, DeviceState _state);

int set_state(const std::string &_motor, DeviceState _state);

int get_states(const std::vector<std::string> &_motors, std::vector<DeviceState> &_states);

int get_state(const std::string &_motor, DeviceState &_state);

int clear_faults(const std::vector<std::string> &_motors);

int clear_fault(const std::string &_motor);
```


**OPERATION**

```
int go_to_velocities_profile(const std::vector<std::string> &_motors, const std::vector<double> &_velocities, bool _rpm);

int go_to_velocities_profile(const std::vector<std::string> &_motors, double _velocity, bool _rpm);

int go_to_velocity_profile(const std::string &_motor, double _velocity, bool _rpm);

int get_velocities(const std::vector<std::string> &_motors, std::vector<double> &_velocities, bool _rpm);

int get_velocity(const std::string &_motor, double &_velocity, bool _rpm);

int go_to_positions_profile(const std::vector<std::string> &_motors, const std::vector<double> &_positions, bool _counts, bool _absolute, bool _immediate);

int go_to_positions_profile(const std::vector<std::string> &_motors, double _position, bool _counts, bool _absolute, bool _immediate);

int go_to_position_profile(const std::string &_motor, double _position, bool _counts, bool _absolute, bool _immediate);

int get_positions(const std::vector<std::string> &_motors, std::vector<double> &_positions, bool _counts);

int get_position(const std::string &_motor, double &_position, bool _counts);

int go_to_torques(const std::vector<std::string> &_motors, const std::vector<double> &_torques);

int go_to_torques(const std::vector<std::string> &_motors, double _torque);

int go_to_torque(const std::string &_motor, double _torque);

int get_torques(const std::vector<std::string> &_motors, std::vector<double> &_torques);

int get_torque(const std::string &_motor, double &_torque);

int get_currents(const std::vector<std::string> &_motors, std::vector<double> &_currents);

int get_current(const std::string &_motor, double &_current);

int halt_velocity(const std::string &_motor);

int halt_position(const std::string &_motor);
```


**ERRORS and LOGGING**

```
static std::string get_error(DWORD _error_code);

std::string get_error_vcs(DWORD _error_code);

std::string get_state_string(DeviceState _state);

std::string get_mode_string(OperationMode _mode);
```


**UTILS**

```
double count_2_m(MaxonMotor &_motor, double _distance);

double m_2_count(MaxonMotor &_motor, double _distance);

double mNm_2_ma(MaxonMotor &_motor, double _torque);

double ma_2_mNm(MaxonMotor &_motor, double _current);
```
