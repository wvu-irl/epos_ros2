#include <epos_ros2/EPOSWrapper.hpp>

namespace epos2
{
   /////////////////////////////////////////////////////////////////////
   /***************************CONFIGURATION***************************/
   /////////////////////////////////////////////////////////////////////
   
   // /**
   //     Sets mode for select devices

   //     @param _ids IDs for devices to set mode
   //     @param _mode operation mode to be set
   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::setMode(std::vector<int> _ids, OpMode _mode)
   // {
   // 		current_mode_ = _mode;
   // 		for (int i = 0; i < _ids.size(); ++i)
   // 		{
   // 				//std::cout << IDs.size() << std::endl;
   // 				if(VCS_SetOperationMode(key_handle_, _ids[i], _mode, &error_code_))
   // 				{
   // 						ROS_INFO("Operation mode set");
   // 				} else
   // 				{
   // 						logError("SetOperationMode");
   // 						return RETURN_FAILED;
   // 				}
   // 		}
   // 		return RETURN_SUCCESS;
   // }

   // /**
   //     ROS callback for setting operational mode

   //     @param msg ROS msg of custom type:
   //  */
   // /**void EPOSWrapper::setModeCallback(const )
   //    {
   //    for(int i = 0; i < msg.nodeID.size(); ++i)
   //    {
   //     if (!setMode(msg.nodeID[i],msg.mode, error_code_))
   //     {
   //       ROS_ERROR("FAILED TO SET MODE OF NODE %d", msg.nodeID[i]);
   //       break;
   //     }
   //    }
   //    }

   //    /**
   //     Resets device state machine

   //     @param _node_id node to have operation mode modified

   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::resetDevice(unsigned short _node_id)
   // {
   // 		int result = RETURN_FAILED;

   // 		if (VCS_ResetDevice(key_handle_, _node_id, &error_code_))
   // 		{
   // 				result = RETURN_SUCCESS;
   // 		}

   // 		return result;
   // }

   // /**
   //     Sets Device State in state machine

   //     @param _node_id node to have operation mode modified
   //     @param _state desried state of state machine
   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::setState(unsigned short _node_id, DevState _state)
   // {
   // 		if( current_state_ == _state || VCS_SetState(key_handle_,_node_id,_state,&error_code_))
   // 		{
   // 				return RETURN_SUCCESS;
   // 		} else
   // 		{
   // 				return RETURN_FAILED;
   // 		}
   // }

   // /**
   //     Gets Device State in state machine

   //     @param _node_id node to have operation mode modified
   //     @param _state state of state machine
   //     @return Success(0)/Failure(1) of command
   //  */
   // int EPOSWrapper::getState(unsigned short _node_id, DevState &_state)
   // {
   // 		short unsigned int state_value; // = getDevStateValue(state);
   // 		ROS_DEBUG("Retrieving State");
   // 		if( VCS_GetState(key_handle_,_node_id,&state_value,&error_code_))
   // 		{
   // 				ROS_DEBUG("State Retrieved");
   // 				_state = getDevState(state_value);
   // 				//std::cout << "Motor "<< nodeID << " is in state " << state << std::endl;
   // 				return RETURN_SUCCESS;

   // 		} else {
   // 				ROS_WARN("State Failed");
   // 				return RETURN_FAILED;
   // 		}
   // }
}