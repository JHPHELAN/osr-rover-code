#include <osr_control/control_loop.h>

using namespace osr_control;
using namespace roboclaw_hardware_interface;

OSRControlLoop::OSRControlLoop(ros::NodeHandle& nh, std::shared_ptr<RoboclawHardwareInterface> hw_interface)
: _nh(nh),
  _hardware_interface(hw_interface)
{
  // create the controller manager
  _controller_manager.reset(new controller_manager::ControllerManager(_hardware_interface.get(), _nh));

  // Get current time for use with first update
  _last_time = steady_clock::now();

  // set the variables TODO load from param server
  _loop_frequency = 100;

  // Start timer that will periodically call OSRControlLoop::update_loop
  _desired_update_interval = ros::Duration(1 / _loop_frequency);
  _non_realtime_loop = _nh.createTimer(_desired_update_interval, &OSRControlLoop::update_loop, this);
}

void OSRControlLoop::update_loop(const ros::TimerEvent&) {
  _current_time = steady_clock::now();
  duration<double> time_span =
    duration_cast<duration<double>>(_current_time - _last_time);
  _elapsed_time = ros::Duration(time_span.count());
  _last_time = _current_time;

  // read update write
  _hardware_interface->read();
  _controller_manager->update(ros::Time::now(), _elapsed_time);
  _hardware_interface->write();
}