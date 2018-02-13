/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Blaise Gassend
#ifndef __DRIVER_BASE__DRIVER_NODE_H__
#define __DRIVER_BASE__DRIVER_NODE_H__

#include <diagnostic_updater/diagnostic_updater.h>
#include <self_test/self_test.h>
#include <ros/node_handle.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <driver_base/SensorLevels.h>
#include <signal.h>
#include <dynamic_reconfigure/server.h>

namespace driver_base
{

class AbstractDriverNode
{
public:
  static int ctrl_c_hit_count_; 
  
  static void hupCalled(int sig)
  {
    ROS_WARN("Unexpected SIGHUP caught. Ignoring it.");
  }
  
  static void sigCalled(int sig)
  {
    ctrl_c_hit_count_++;
  }
};
  
template <class DriverType>
int main(int argc, char **argv, std::string name)
{
  /// @todo put SIGINT handling back in at some point. Out for now because
  /// it prevents proper handling of SIGINT when there is no master.
  ros::init(argc, argv, name);
  //ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  //signal(SIGINT, &AbstractDriverNode::sigCalled);
  //signal(SIGTERM, &AbstractDriverNode::sigCalled);
  signal(SIGHUP, &AbstractDriverNode::hupCalled);
  ros::NodeHandle nh;
  DriverType driver(nh);
  return driver.spin();
  /// @todo Add check for leaked file descriptors and such things here.
}
  
template <class Driver>
class DriverNode : public AbstractDriverNode
{
public:
  //typedef char state_t;
  typedef typename Driver::Config Config;
 
protected:
  // Hooks
  virtual void addDiagnostics() = 0; 
  virtual void addStoppedTests() = 0;
  virtual void addOpenedTests() = 0;
  virtual void addRunningTests() = 0;
  virtual void reconfigureHook(int level) = 0; 

  // Helper classes
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;
  
  Driver driver_;
  dynamic_reconfigure::Server<Config> reconfigure_server_; 
  diagnostic_updater::CompositeDiagnosticTask driver_status_diagnostic_;
  // driver_ declaration must precede reconfigure_server_ for constructor
  // calling order reasons.

private:
  // Subscriber tracking
  int num_subscribed_topics_; // Number of topics that have subscribers.
  
  //static const state_t DISABLED = 0;
  //static const state_t LAZY_ON = 1;
  //static const state_t ALWAYS_ON = 2;
  //static const state_t SELF_TEST = 3;
  //static const state_t EXITING = 4;

  //state_t state_;
                          
  boost::shared_ptr<boost::thread> ros_thread_;
  
  int exit_status_;

  // The device
  typedef typename Driver::state_t drv_state_t;

  drv_state_t pre_self_test_driver_state_;
    
  diagnostic_updater::FunctionDiagnosticTask driver_status_standard_diagnostic_;

  void reconfigure(Config &config, uint32_t level)
  {
    /// @todo Move this into the Driver class?
    ROS_DEBUG("Reconfigure called at level %x.", level);
    boost::recursive_mutex::scoped_lock lock(driver_.mutex_);
    
    drv_state_t orig_state = driver_.getState();
  
    if ((level | driver_base::SensorLevels::RECONFIGURE_STOP) == level)
    {
      driver_.stop();
      if (!driver_.isStopped())
        ROS_ERROR("Failed to stop streaming before reconfiguring. Reconfiguration may fail.");
    }
  
    if ((level | driver_base::SensorLevels::RECONFIGURE_CLOSE) == level)
    {
      driver_.close();
      if (!driver_.isClosed())
        ROS_ERROR("Failed to close device before reconfiguring. Reconfiguration may fail.");
    }
  
    driver_.config_update(config, level);
    config = driver_.config_;
    reconfigureHook(level);
  
    driver_.goState(orig_state);

    /* Other errors will show up when this happens. No need to have it
     * here.
    if (driver_.getState() != orig_state)
    {
      ROS_ERROR("Failed to resume original device state after reconfiguring. The requested configuration may contain errors.");
    }
    */
    
    ROS_DEBUG("Reconfigure completed.");
  }

private:
/*  void connectCallback(const ros::PublisherPtr &pub)
  {
    if (pub.numSubscribers == 1)
      num_subscribed_topics_++;

    if (num_subscribed_topics_ == 1)
      start();
  }

  void disconnectCallback(const ros::PublisherPtr &pub)
  {
    if (pub.numSubscribers == 0)
      num_subscribed_topics_++;

    if (num_subscribed_topics_ == 0)
      stop();
  }*/

  void prepareDiagnostics()
  {
    driver_status_diagnostic_.addTask(&driver_status_standard_diagnostic_);
    diagnostic_.add(driver_status_diagnostic_);
    addDiagnostics();
  }

  void statusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (driver_.isRunning())
      stat.summary(0, "OK");
    else
      stat.summary(2, "not running");
    
    stat.add("Driver state:", driver_.getStateName());
    stat.add("Latest status message:", driver_.getStatusMessage());
    /// @fixme need to put something more useful here.
  }

  void prepareSelfTests()
  {
    self_test_.add( "Interruption Test", this, &DriverNode::interruptionTest );
    addStoppedTests();
    self_test_.add( "Connection Test", this, &DriverNode::openTest );
    self_test_.add( "ID Test", this, &DriverNode::idTest );
    addOpenedTests();
    self_test_.add( "Start Streaming Test", this, &DriverNode::runTest );
    addRunningTests();
    self_test_.add( "Stop Streaming Test", this, &DriverNode::stopTest );
    self_test_.add( "Disconnection Test", this, &DriverNode::closeTest );
    self_test_.add( "Resume Activity", this, &DriverNode::resumeTest );
  } 

  void interruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    pre_self_test_driver_state_ = driver_.getState();
    driver_.goClosed();
    
    drv_state_t s = driver_.getState();

    if (num_subscribed_topics_ > 0)  /// @todo need to set num_subscribed_topics_ somewhere.
      status.summary(1, "There were active subscribers.  Running of self test interrupted operations.");
    else if (s != Driver::CLOSED)
      status.summaryf(2, "Could not close device, currently in state %s.", Driver::getStateName(s).c_str());
    else
      status.summary(0, "No operation interrupted.");
  } 
  
  void reliableGoStateTest(diagnostic_updater::DiagnosticStatusWrapper &status, drv_state_t target_state)
  {
    const int max_tries = 5;
    ROS_ASSERT(max_tries > 0);
    int retries;
    drv_state_t current_state;
    for (retries = 0; retries < max_tries; retries++)
    {
      sleep(1);
      driver_.goState(target_state);

      current_state = driver_.getState();
      if (current_state == target_state)
        break;
    }

    std::string target_state_name = Driver::getStateName(target_state);
    std::string current_state_name = Driver::getStateName(current_state);
    
    // We carefully avoid rechecking the state here, as it may have
    // changed since we last checked. We aren't here to check that we
    // stayed in the target state, just want to check that we made it
    // there...
    if (retries >= max_tries) 
      status.summaryf(2, "Failed to go to %s state despite %u retries, now in state %s.", target_state_name.c_str(), retries, current_state_name.c_str());
    else if (retries)
      status.summaryf(1, "Successfully went to state %s after %u retries.", target_state_name.c_str(), retries);
    else
      status.summaryf(0, "Successfully went to state %s.", target_state_name.c_str());
  }

  void openTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    reliableGoStateTest(status, Driver::OPENED);
  }
  
  void idTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string ID = driver_.getID();
    if (ID.compare(""))
    {
      status.summaryf(0, "Device ID is %s", ID.c_str());
      self_test_.setID(ID);
    }
    else
      status.summaryf(2, "Error reading Device ID");
  }

  void runTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    reliableGoStateTest(status, Driver::RUNNING);
  } 

  void stopTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    reliableGoStateTest(status, Driver::OPENED);
  } 
  
  void closeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    reliableGoStateTest(status, Driver::CLOSED);
  } 
  
  void resumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    reliableGoStateTest(status, pre_self_test_driver_state_);
  }
  
public:
  virtual ~DriverNode() {}

  int spin()
  {
    prepareDiagnostics();
    prepareSelfTests();
    typename dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&DriverNode::reconfigure, this, _1, _2);
    reconfigure_server_.setCallback(f);
    
    ros_thread_.reset(new boost::thread(boost::bind(&ros::spin)));
    /// @todo What happens if thread creation fails?
    assert(ros_thread_);
               
    driver_.goRunning();

    /// @todo Do something about exit status?
    std::string last_status_message;
    while (node_handle_.ok() && /*state_ != EXITING &&*/ !ctrl_c_hit_count_)
    {
      {
        // Must lock here, otherwise operations like reconfigure will cause
        // the test to pass, and an unnecessary restart will happen when the 
        // reconfigure is done.
        boost::recursive_mutex::scoped_lock lock_(driver_.mutex_);

        if (!driver_.isRunning())
        {             
          std::string new_status_message = driver_.getStatusMessage();
          if ((last_status_message != new_status_message || driver_.getRecoveryComplete()) 
              && !driver_.getStatusOk())
          {
            ROS_ERROR("%s", new_status_message.c_str()); 
            driver_.clearRecoveryComplete();
            last_status_message = new_status_message;
          }
          ros::WallDuration(1).sleep();
          driver_.goClosed(); 
          driver_.goRunning();
        }

        diagnostic_.update();
        self_test_.checkTest();
      }
      ros::WallDuration(0.1).sleep();
    }

    driver_.goClosed();

    ros::shutdown();
    
    if (ros_thread_ && !ros_thread_->timed_join((boost::posix_time::milliseconds) 2000))
    {
      ROS_ERROR("ROS thread did not die after two seconds. Pretending that it did. This is probably a bad sign.");
    }
    ros_thread_.reset();

    return 0; /// @todo Work on return type here.
  }
  
  DriverNode(ros::NodeHandle &nh) : 
    node_handle_(nh), 
    private_node_handle_("~"), 
    self_test_(node_handle_), 
    diagnostic_(), 
    reconfigure_server_(driver_.mutex_, ros::NodeHandle("~")),
    driver_status_diagnostic_("Driver Status"),
    driver_status_standard_diagnostic_("Driver Status", boost::bind(&DriverNode::statusDiagnostic, this, _1))
  {
    num_subscribed_topics_ = 0; /// @fixme this variable is hokey.
    exit_status_ = 0;

    // Defer most initialization to spin so that virtual calls in derived
    // classes will succeed.
  }
};

int AbstractDriverNode::ctrl_c_hit_count_ = 0;
  
// @todo exit status.
// @todo take over ctrl_c.

};

#endif
