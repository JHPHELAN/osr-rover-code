#ifndef OSR_CONTROL_LOOP_H
#define OSR_CONTROL_LOOP_H

#include <ros/node_handle.h>

#include <chrono>

#include <controller_manager/controller_manager.h>
#include <roboclaw-hardware-interface/hardware_interface.h>


namespace osr_control {
  using namespace std::chrono;
  /**
   * Control loop for the OSR drive and corner motors
   */
  class OSRControlLoop
  {
  public:
    /*
     * Initialize the controller manager loop
     */
    OSRControlLoop(
        ros::NodeHandle& nh, 
        std::shared_ptr<roboclaw_hardware_interface::RoboclawHardwareInterface> hw_interface
    );

    ~OSRControlLoop() {};

    /*
     * Perform an update step, triggered by the _non_realtime_loop timer with _loop_frequency
     */
    void update_loop(const ros::TimerEvent&);

  private:
    ros::NodeHandle _nh;

    std::shared_ptr<controller_manager::ControllerManager> _controller_manager;
    std::shared_ptr<roboclaw_hardware_interface::RoboclawHardwareInterface> _hardware_interface;

    // tracking timing, frequency of execution
    steady_clock::time_point _current_time;
    steady_clock::time_point _last_time;
    ros::Duration _elapsed_time;
    double _loop_frequency;
    ros::Duration _desired_update_interval;
    ros::Timer _non_realtime_loop;
  };
}

#endif // OSR_CONTROL_LOOP_H