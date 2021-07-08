/*
   Maxon Motor Controller EPOSWrappers ROS Parameters
   ROSParams.hpp
   Purpose: Link together various parameters for the EPOSWrapper

   @author Jared Beard
   @version 1.0 11/13/18
 */

#ifndef ROS_NODE_PARAMS_HPP
#define ROS_NODE_PARAMS_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace epos2
{

  enum loggingPriority
  {
    LOG_OFF = 0,
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL
  };
  enum loggingThrottle
  {
    THROTTLE_OFF = 0,
    THROTTLE_FREQUENCY,
    THROTTLE_SKIP_N,
    THROTTLE_DYNAMIC
  };

  //typedef std::pair<loggingThrottle, int>

  class ROSNodeParams
  {

  public:
    shared_ptr<rclcpp::Node> node_;
    int logging_priority_;
    double logging_throttle_;
    std::vector<std::tuple<int, int, rclcpp::Time>> throttled_loggers;

    ROSNodeParams(shared_ptr<rclcpp::Node> _node, int _logging_priority, double _logging_throttle): 
                    node_(_node), _logging_priority(logging_priority_, _logging_throttle(logging_throttle_);

    virtual bool loggingCondition() {
      return true;}

    void ros2Log(std::string _msg, int _priority = 0, int _throttle = 0, double _throttle_val = 0, int _throttle_id = 0)
    {
      bool throttle_released = true;
      if (_throttle)
      {
        bool in_list = false;
        for (auto &logger : throttled_loggers_)
        {
          if (!(_throttle_ind == logger.first))
          {
            switch (_throttle)
            {
            case THROTTLE_OFF:
              break;
            case THROTTLE_FREQUENCY:
              if ((rclcpp::Time::now().seconds() - logger.third.seconds()) <= 1.0 / _throttle_val)
              {
                throttle_released = false;
              }
              else
              {
                logger.third = rclcpp::Time::now();
              }
              break;
            case THROTTLE_SKIP_N:
              if (logger.second % != 0)
                throttle_released = false;
              ++logger.second;
              break;
            default:
              std::string header = "[" this->get_namespace() + "/" + this->get_node + "] ";
              RCLCPP_FATAL(this->get_logger(), header + "INVALID LOGGING THROTTLE: logger %d", _throttle_id);
             	assert(("KILLED LOGGER", false));	
              break;
            }
          }
          in_list = true;
          break;
        }
      }

      if (!in_list)
      {
        throttled_loggers.push_back(std::make_tyuple(_msg, 1, rclcpp::Time::now()));
      }
        
      }

      if (throttle_released)
      {
      std::string header = "[" this->get_namespace() + "/" + this->get_node + "] ";
      switch (_priority)
      {
      case LOG_OFF:
        break;
      case LOG_DEBUG:
        break;
      case LOG_INFO:
        break;
      case LOG_WARN:
        break;
      case LOG_ERROR:
        break;
      case LOG_FATAL:
        break;
      default:
        RCLCPP_FATAL(this->get_logger(), header + "INVALID PRIORITY" );
        assert(("KILLED LOGGER", false));	
        break;
      }
      }
    //check if throttle condition is met. If so, we can log
    //log at priority
    //check throttle
    //check priority
    //log
  }

  private:
};
}
#endif