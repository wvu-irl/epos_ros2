#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
    /////////////////////////////////////////////////////////////////////
    /**************************ERRORS and LOGGING***********************/
    /////////////////////////////////////////////////////////////////////

    // /**
    //     Displays Error info for an executed function

    //     @param error_code_Value Error code number
    //     @return Success(0)/Failure(1) of command
    //  */
    // int EPOSWrapper::getError(unsigned short _error_code_value)
    // {
    // 		int result = RETURN_FAILED;
    // 		if(VCS_GetErrorInfo(_error_code_value, error_code_char_, MMC_MAX_LOG_MSG_SIZE))
    // 		{
    // 				ROS_ERROR("ERROR %u: %u\n", _error_code_value, *error_code_char_);
    // 				result = RETURN_SUCCESS;
    // 		}

    // 		return result;
    // }

    // void EPOSWrapper::logError(std::string _function_name)
    // {
    // 		std::cerr << "EPOS COMMAND: " << _function_name << " failed (error_code_=0x" << std::hex << error_code_ << ")"<< std::endl;
    // }

    // int EPOSWrapper::checkNodeID(int _id)
    // {
    // 		int result = RETURN_FAILED;

    // 		for (int i = 0; i < node_id_list_.size(); ++i)
    // 		{
    // 				if (_id == node_id_list_[i]) {
    // 						result == RETURN_SUCCESS;
    // 				}
    // 		}
    // 		return result;
    // }

    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------

    
    void EPOSWrapper::log_msg(std::string _msg, loggingVerbosity _verbosity = LOG_OFF, loggingGroup _group = GROUP_ERROR)
    {
        _msg = "[EPOS WRAPPER] " + _msg;

        bool is_in_group = std::find(epos_params_.groups.begin(), epos_params_.groups.end(), _group) != epos_params_.groups.end();
        bool log_expression = epos_params_.is_on && (is_in_group || _group == GROUP_ERROR);

        switch (_verbosity)
        {
        case LOG_OFF:
            break;
        case LOG_DEBUG:
            RCLCPP_DEBUG_EXPRESSION(node_ptr_->get_logger(), log_expression, _msg.c_str());
            break;
        case LOG_INFO:
            RCLCPP_INFO_EXPRESSION(node_ptr_->get_logger(), log_expression, _msg.c_str());
            break;
        case LOG_WARN:
            RCLCPP_WARN_EXPRESSION(node_ptr_->get_logger(), log_expression, _msg.c_str());
            break;
        case LOG_ERROR:
            RCLCPP_ERROR_EXPRESSION(node_ptr_->get_logger(), log_expression, _msg.c_str());
            break;
        case LOG_FATAL:
            RCLCPP_FATAL_EXPRESSION(node_ptr_->get_logger(), log_expression, _msg.c_str());
            break;
        default:
            RCLCPP_FATAL(node_ptr_->get_logger(), "[EPOS WRAPPER] INVALID ERROR VERBOSITY");
            break;
        }
    }

    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------

    std::string EPOSWrapper::get_error_code(DWORD _error_code)
    {
        switch (_error_code)
        {
        case 0x00000000:
            return "[" + std::to_string(_error_code) + "] No error: communication was successful";
        case 0x10000001:
            return "[" + std::to_string(_error_code) + "] Internal error: ¯\\_(ツ)_/¯";
        case 0x10000002:
            return "[" + std::to_string(_error_code) + "] Null pointer: Null pointer passed to function";
        case 0x10000003:
            return "[" + std::to_string(_error_code) + "] Handle not valid: Handle passed to function is not valid";
        case 0x10000004:
            return "[" + std::to_string(_error_code) + "] Bad virtual device name: Virtual device name is not valid";
        case 0x10000005:
            return "[" + std::to_string(_error_code) + "] Bad device name: Device name is not valid";
        case 0x10000006:
            return "[" + std::to_string(_error_code) + "] Bad protocal stack name: Protocol stack name is not valid";
        case 0x10000007:
            return "[" + std::to_string(_error_code) + "] Bad interface name: Interface name is not valid";
        case 0x10000008:
            return "[" + std::to_string(_error_code) + "] Bad port name: Port name is not valid";
        case 0x10000009:
            return "[" + std::to_string(_error_code) + "] Library not loaded: Could not load external library";
        case 0x1000000A:
            return "[" + std::to_string(_error_code) + "] Command failed: Error while executing command";
        case 0x1000000B:
            return "[" + std::to_string(_error_code) + "] Timeout: Timeout occurred during execution";
        case 0x1000000C:
            return "[" + std::to_string(_error_code) + "] Bad parameter: Bad parameter passed to function";
        case 0x1000000D:
            return "[" + std::to_string(_error_code) + "] Command aborted by user: ¯\\_(ツ)_/¯";
        case 0x1000000E:
            return "[" + std::to_string(_error_code) + "] Buffer too small: ¯\\_(ツ)_/¯";
        case 0x1000000F:
            return "[" + std::to_string(_error_code) + "] No communication found: No communication settings found";
        case 0x10000010:
            return "[" + std::to_string(_error_code) + "] Function not supported: ¯\\_(ツ)_/¯";
        case 0x100000011:
            return "[" + std::to_string(_error_code) + "] Parameter already used: ¯\\_(ツ)_/¯";
        case 0x10000013:
            return "[" + std::to_string(_error_code) + "] Bad device handle: ¯\\_(ツ)_/¯";
        case 0x10000014:
            return "[" + std::to_string(_error_code) + "] Bad protocol stack: ¯\\_(ツ)_/¯";
        case 0x10000015:
            return "[" + std::to_string(_error_code) + "] Bad interface handle: ¯\\_(ツ)_/¯";
        case 0x10000016:
            return "[" + std::to_string(_error_code) + "] Bad port handle: ¯\\_(ツ)_/¯";
        case 0x10000017:
            return "[" + std::to_string(_error_code) + "] Address parameters are not correct: ¯\\_(ツ)_/¯";
        case 0x10000020:
            return "[" + std::to_string(_error_code) + "] Bad device state: ¯\\_(ツ)_/¯";
        case 0x10000021:
            return "[" + std::to_string(_error_code) + "] Bad file content: ¯\\_(ツ)_/¯";
        case 0x10000022:
            return "[" + std::to_string(_error_code) + "] Path does not exist: System cannot find specified path";
        case 0x10000024:
            return "[" + std::to_string(_error_code) + "] Cross thread error: (.NET only) Open device and close device called from different threads";
        case 0x10000026:
            return "[" + std::to_string(_error_code) + "] Gateway support error: Gateway is not supported";
        case 0x10000027:
            return "[" + std::to_string(_error_code) + "] Serial number update error: Serial number update failed";
        case 0x10000028:
            return "[" + std::to_string(_error_code) + "] Communication interface error: Communication interface is not supported";
        case 0x10000029:
            return "[" + std::to_string(_error_code) + "] Firmware support error: Firmware version does not support functionality";
        case 0x1000002A:
            return "[" + std::to_string(_error_code) + "] Firmware file hardware error: Firmware file does not match hardware version";
        case 0x1000002B:
            return "[" + std::to_string(_error_code) + "] Firmware file error: Firmware file does not match or is corrupt";
        case 0x20000001:
            return "[" + std::to_string(_error_code) + "] Opening interface error: Error while opening interface";
        case 0x20000002:
            return "[" + std::to_string(_error_code) + "] Closing interface error: Error while closing interface";
        case 0x20000003:
            return "[" + std::to_string(_error_code) + "] Interface is not open: ¯\\_(ツ)_/¯";
        case 0x20000004:
            return "[" + std::to_string(_error_code) + "] Opening port error: Error while opening port";
        case 0x20000005:
            return "[" + std::to_string(_error_code) + "] Closing port error: Error while closing port";
        case 0x20000006:
            return "[" + std::to_string(_error_code) + "] Port is not open: ¯\\_(ツ)_/¯";
        case 0x20000007:
            return "[" + std::to_string(_error_code) + "] Resetting port error: Error while resetting port";
        case 0x20000008:
            return "[" + std::to_string(_error_code) + "] Configuring port settings error: Error while configuring port settings";
        case 0x20000009:
            return "[" + std::to_string(_error_code) + "] Configuring port mode error: Error while configuring port mode";
        case 0x21000001:
            return "[" + std::to_string(_error_code) + "] RS232 write data error: Error while writing RS232 data";
        case 0x21000002:
            return "[" + std::to_string(_error_code) + "] RS232 read data error: Error while reading RS232 data";
        case 0x22000001:
            return "[" + std::to_string(_error_code) + "] CAN receive frame error: Error while receiving CAN frame";
        case 0x22000002:
            return "[" + std::to_string(_error_code) + "] CAN transmit frame error: Error while transmitting CAN frame";
        case 0x23000001:
            return "[" + std::to_string(_error_code) + "] USB write data error: Error while writing USB data";
        case 0x23000002:
            return "[" + std::to_string(_error_code) + "] USB read data error: Error while writing USB data";
        case 0x24000001:
            return "[" + std::to_string(_error_code) + "] HID write data error: Error while writing USB data to HID device";
        case 0x24000002:
            return "[" + std::to_string(_error_code) + "] HID read data error: Error while reading USB data from HID device";
        case 0x31000001:
            return "[" + std::to_string(_error_code) + "] Negative acknowledge received: ¯\\_(ツ)_/¯";
        case 0x31000002:
            return "[" + std::to_string(_error_code) + "] Bad CRC received: Bad checksum received";
        case 0x31000003:
            return "[" + std::to_string(_error_code) + "] Bad data received: Bad data size received";
        case 0x32000001:
            return "[" + std::to_string(_error_code) + "] SDO response not received: CAN frame of SDO protocol not received";
        case 0x32000002:
            return "[" + std::to_string(_error_code) + "] Requested CAN frame not received: ¯\\_(ツ)_/¯";
        case 0x32000003:
            return "[" + std::to_string(_error_code) + "] CAN frame not received: ¯\\_(ツ)_/¯";
        case 0x34000001:
            return "[" + std::to_string(_error_code) + "] Stuffing error: Failure while stuffing data";
        case 0x34000002:
            return "[" + std::to_string(_error_code) + "] Destuffing error: Failuer while destuffing data";
        case 0x34000003:
            return "[" + std::to_string(_error_code) + "] Bad CRC received: ¯\\_(ツ)_/¯";
        case 0x34000004:
            return "[" + std::to_string(_error_code) + "] Bad data size received: ¯\\_(ツ)_/¯";
        case 0x34000005:
            return "[" + std::to_string(_error_code) + "] Bad data size written: ¯\\_(ツ)_/¯";
        case 0x34000006:
            return "[" + std::to_string(_error_code) + "] Serial data frame not writter: Failuer occurred while writing data";
        case 0x34000007:
            return "[" + std::to_string(_error_code) + "] Serial data frame not received: Failure occurred while reading data";
        case 0x50300000:
            return "[" + std::to_string(_error_code) + "] Toggle error: Toggle bit not alternated";
        case 0x50400000:
            return "[" + std::to_string(_error_code) + "] SDO timeout: SDO protocol timed out";
        case 0x50400001:
            return "[" + std::to_string(_error_code) + "] Client/server specific error: Client/server command specifier not valid or unknown";
        case 0x50400002:
            return "[" + std::to_string(_error_code) + "] Invalid block size: (block mode only)";
        case 0x50400003:
            return "[" + std::to_string(_error_code) + "] Invalid sequence: Invalid sequence number (block mode only)";
        case 0x50400004:
            return "[" + std::to_string(_error_code) + "] CRC error: (block mode only)";
        case 0x5040005:
            return "[" + std::to_string(_error_code) + "] Out of memory error: ¯\\_(ツ)_/¯";
        case 0x51000001:
            return "[" + std::to_string(_error_code) + "] Bad data size received: Object data size does not correspond to requested data size";
        case 0x51000007:
            return "[" + std::to_string(_error_code) + "] Sensor configuration not supported: Sensor configuration cannot be writted to controller";
        case 0x51000008:
            return "[" + std::to_string(_error_code) + "] Sensor configuration unknown: Sensor configuration read from controller is not supported by library";
        case 0x51000009:
            return "[" + std::to_string(_error_code) + "] Configuration not supported: ¯\\_(ツ)_/¯";
        case 0x5100000A:
            return "[" + std::to_string(_error_code) + "] Digital input mask not supported: ¯\\_(ツ)_/¯";
        case 0x60100000:
            return "[" + std::to_string(_error_code) + "] Access error: Unsupported access to an object (e.g., write command to a read-only object";
        case 0x60100001:
            return "[" + std::to_string(_error_code) + "] Write only: Read command to a write only object";
        case 0x60100002:
            return "[" + std::to_string(_error_code) + "] Read only: Write command to a read only object";
        case 0x60200000:
            return "[" + std::to_string(_error_code) + "] Object does not exist: Last read/write command had an invalid object index or subindex";
        case 0x60400041:
            return "[" + std::to_string(_error_code) + "] PDO mapping error: Object cannot be mapped to PDO";
        case 0x60400042:
            return "[" + std::to_string(_error_code) + "] PDO length error: Number and length of objects to be mapped would exceed PDO length";
        case 0x60400043:
            return "[" + std::to_string(_error_code) + "] General parameter error: General parameter incompatibility";
        case 0x60400047:
            return "[" + std::to_string(_error_code) + "] General internal incompatibility error: General internal incompatibility in device";
        case 0x60600000:
            return "[" + std::to_string(_error_code) + "] Hardware error: Access failed due to a hardware error";
        case 0x60700010:
            return "[" + std::to_string(_error_code) + "] Service parameter error: Data type does not match, length or service parameter does not match";
        case 0x60700012:
            return "[" + std::to_string(_error_code) + "] Service parameter too high: Data typ edoes not match, length or service parameter too high";
        case 0x60700013:
            return "[" + std::to_string(_error_code) + "] Service parameter too low: Data type does not match, length or service parameter too low";
        case 0x60900011:
            return "[" + std::to_string(_error_code) + "] Object subindex error: Last read/write command had invalid subindex";
        case 0x60900030:
            return "[" + std::to_string(_error_code) + "] Value range error: Value range of parameter exceeded";
        case 0x60900031:
            return "[" + std::to_string(_error_code) + "] Value too high: Value of parameter written too high";
        case 0x60900032:
            return "[" + std::to_string(_error_code) + "] Value too low: Value of parameter written too low";
        case 0x60900036:
            return "[" + std::to_string(_error_code) + "] Maximum less minimum error: Maximum value is less than minimum value";
        case 0x80000000:
            return "[" + std::to_string(_error_code) + "] General error: ¯\\_(ツ)_/¯ this sucks I guess";
        case 0x80000020:
            return "[" + std::to_string(_error_code) + "] Transfer/store error: Data cannot be transferred/stored";
        case 0x80000021:
            return "[" + std::to_string(_error_code) + "] Local control error: Data cannot transferred/stored to application because of local control";
        case 0x80000022:
            return "[" + std::to_string(_error_code) + "] Wrong device state: Data cannot be transferred/stored to application because of present device state";
        case 0x0F00FFB9:
            return "[" + std::to_string(_error_code) + "] CAN ID error: Wrong CAN ID";
        case 0x0F00FFBC:
            return "[" + std::to_string(_error_code) + "] Service mode error: Device is not in service";
        case 0x0F00FFBE:
            return "[" + std::to_string(_error_code) + "] Password error: Password is wrong ¯\\_(ツ)_/¯";
        case 0x0F00FFBF:
            return "[" + std::to_string(_error_code) + "] Illegal command: RS232 command is illegal (does not exist)";
        case 0x0F00FFC0:
            return "[" + std::to_string(_error_code) + "] Wrong NMT state: Device is in wrong NMT state";
        default:
            return "INVALID ERROR CODE: " + std::to_string(_error_code);
        }
    }
}
