#include <ros/ros.h>
#include <osr_control/control_loop.h>
#include <ros/console.h>


int main(int argc, char **argv) {
  using namespace roboclaw_hardware_interface;

  ros::init(argc, argv, "osr_control");
  ros::NodeHandle nh, nh_private("~");

  // set the log level to debug programmatically (should be done trough rqt_logger_level instead)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("Initializing hardware interface for OSR...");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  try {
      // Create the hardware interface
      std::shared_ptr<RoboclawHardwareInterface> osr_hw_interface = std::make_shared<RoboclawHardwareInterface>();
      // Initialize the hardware interface:
      osr_hw_interface->init(nh, nh_private);
      ROS_DEBUG("Initialized hardware interface.");

      // Start the control loop
      osr_control::OSRControlLoop osr_control_loop(nh, osr_hw_interface);

      ros::waitForShutdown();
  }
  catch (const ros::Exception& e) {
      ROS_FATAL_STREAM("Error in the hardware interface:\n"
          << "\t" << e.what());
      return 1;
  }

  return 0;
}
