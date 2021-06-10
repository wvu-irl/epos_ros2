/*
   Parameter Server for EPOS
   epos_param_server.hpp
   Purpose: Node to EPOS parameters 

   @author Jared Beard
   @version 1.0 6/4/21
 */

#include <chrono>
#include <memory>

#include <string>

#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

class EPOSParams : public rclcpp::Node
{
	public:
        EPOSParams(std::string node_name) : Node (node_name)
		{
			timer_ = this->create_wall_timer(2000ms, std::bind(&EPOSParams::timer_callback, this));
    	}
  	private:
		rclcpp::TimerBase::SharedPtr timer_;

    	void timer_callback()
    	{
            RCLCPP_INFO(this->get_logger(),"Yeet");
    	}
    	

};

int main(int argc, char** argv)
{
	// ROS INITILIZATION ------------------------------------------------------------------------
	rclcpp::init(argc,argv);
	//EPOSParams params("epos_parameters");
    rclcpp::spin(std::make_shared<EPOSParams>("epos_parameters"));

	//ROS SPIN	
	rclcpp::shutdown();
    return 0;
}