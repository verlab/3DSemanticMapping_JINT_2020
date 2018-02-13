/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009-2010, Willow Garage, Inc.
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

#ifndef __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__
#define __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__

#include <ros/ros.h>
#include <queue>
#include <utility>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <std_msgs/Header.h>
#include <boost/thread/thread_time.hpp>

namespace timestamp_tools
{

class TriggerMatcherBase
{
protected:
  boost::mutex mutex_;
  virtual void gotTrigger() = 0;

  ros::Time getTimestampNoblockPrelocked(const ros::Time &data_time)
  {
    while (!trig_queue_.empty())
    {
      // If the head trigger is after the head data then we don't have a
      // timestamp for the data and drop it.
      if (nonCausalHeads(data_time))
      {
        if (verbose_)
          ROS_WARN("TriggerMatcherBase: data arrived before trigger. Discarding data sample.");
        return DropData;
      }
      
      ros::Time trig_stamp = trig_queue_.front();
      
      // If we are not synchronized, then we need to check that the next trigger
      // time stamp would not be satisfactory. (If no late counts are
      // allowed then we are always unsynchronized.)
      
      //ROS_DEBUG("getTimestampNoblock t=%f, qt=%f, ql=%i, sync = %i, last = %f", t.toSec(), trig_stamp.toSec(), trig_queue_.size(), synchronized_, last_data_stamp_.toSec());

      if (!synchronized_ || !late_data_count_allowed_)
      {
        //ROS_DEBUG("Not synchronized...");
        while (true)
        {
          if (trig_queue_.size() < 2)
            return RetryLater; // Can't do the check now.

          trig_queue_.pop();
          if (nonCausalHeads(data_time)) 
            break; // Head is non-causal, we have arrived.
          trig_stamp = trig_queue_.front(); // Skip that trigger event.
        }
        synchronized_ = true;
        late_data_count_ = 0;
      }
      else 
      {
        // If the data is later than the next trigger timestamp, we might
        // have missed a timestamp, or the data may just have been delayed a
        // lot. We count it and if it happens too often we assume that a
        // timestamp was missed and redo the locking process.
        //ROS_DEBUG("Packet check %f >? %f", last_data_stamp_.toSec(), trig_stamp.toSec());
        if (last_data_stamp_ > trig_stamp)
        {
          //ROS_DEBUG("Late packet...");
          if (++late_data_count_ >= late_data_count_allowed_)
          {
            if (verbose_)
              ROS_WARN("TriggerMatcherBase: too many late data packets. Assuming missed trigger. Relocking...");
            synchronized_ = false;
            continue;
          }
        }
        else
          late_data_count_ = 0;

        trig_queue_.pop();
      }
      
      last_data_stamp_ = data_time;
      return trig_stamp;
    }

    return RetryLater;
  }
  
  virtual void baseReset()
  {
    synchronized_ = false;
    late_data_count_ = 0;
    trig_queue_ = std::queue<ros::Time>();
  }

private:
  std::queue<ros::Time> trig_queue_;
  
  ros::Duration trig_delay_;

  ros::Time last_data_stamp_;

  unsigned int late_data_count_allowed_;
  unsigned int late_data_count_;
  unsigned int max_trig_queue_length_;
  bool synchronized_;

  bool nonCausalHeads(const ros::Time &data_stamp)
  { // Are the heads non-causal? (Assumes the heads are present.)
    return trig_queue_.front() > data_stamp;
  }
  
public:
  bool verbose_;
  
  static const ros::Time DropData;
  static const ros::Time RetryLater;

  bool hasTimestamp()
  {
    return !trig_queue_.empty();
  }
  
  virtual ~TriggerMatcherBase()
  {
  }

  void setLateDataCountAllowed(unsigned int v)
  {
    late_data_count_allowed_ = v;
  }

  void setTrigDelay(double delay)
  {
    setTrigDelay(ros::Duration(delay));
  }

  void setTrigDelay(const ros::Duration &delay)
  {
    trig_delay_ = delay;
  }

  TriggerMatcherBase(unsigned int late_data_count_allowed, unsigned int max_trig_queue_length) : 
    trig_delay_(0), 
    last_data_stamp_(ros::TIME_MIN),
    late_data_count_allowed_(late_data_count_allowed),
    late_data_count_(0),
    max_trig_queue_length_(max_trig_queue_length),
    synchronized_(false),
    verbose_(false)
  {
  }

  void triggerCallback(double stamp)
  {
    triggerCallback(ros::Time(stamp));
  }

  void triggerCallback(const std_msgs::HeaderConstPtr &msg)
  {
    triggerCallback(msg->stamp);
  }

  void triggerCallback(const ros::Time &stamp)
  {
    boost::mutex::scoped_lock(mutex_);
    
    trig_queue_.push(stamp + trig_delay_);
    if (trig_queue_.size() > max_trig_queue_length_)
    {
      if (verbose_)
        ROS_WARN("TriggerMatcherBase: trig_queue_ overflow dropping from front.");
      trig_queue_.pop();
    }
    gotTrigger();
  }
};
  
const ros::Time TriggerMatcherBase::DropData = ros::TIME_MIN;
const ros::Time TriggerMatcherBase::RetryLater = ros::TIME_MAX;
 
class TriggerMatcher : public TriggerMatcherBase
{
private:
  boost::mutex data_source_mutex_;
  boost::condition_variable got_trigger_condition_;

protected:
  virtual void gotTrigger()
  {
    got_trigger_condition_.notify_one();
  }

public:
  virtual ~TriggerMatcher()
  {}

  TriggerMatcher(unsigned int late_data_count_allowed, unsigned int max_trig_queue_length) :
    TriggerMatcherBase(late_data_count_allowed, max_trig_queue_length)
  {}

  void reset()
  {
    boost::mutex::scoped_lock lock(mutex_);

    got_trigger_condition_.notify_all();
    baseReset();
  }

  ros::Time getTimestampBlocking(const ros::Time &t)
  {
    boost::mutex::scoped_lock data_lock(data_source_mutex_);
    boost::mutex::scoped_lock lock(mutex_);

    ros::Time stamp = getTimestampNoblockPrelocked(t);

    if (stamp != RetryLater)
      return stamp;
    
    got_trigger_condition_.wait(lock);
    
    return getTimestampNoblockPrelocked(t);
  }

  ros::Time getTimestampBlocking(const ros::Time &t, double timeout)
  {
    boost::mutex::scoped_lock data_lock(data_source_mutex_);
    boost::mutex::scoped_lock lock(mutex_);

    ros::Time stamp = getTimestampNoblockPrelocked(t);

    if (stamp != RetryLater)
      return stamp;

    got_trigger_condition_.timed_wait(lock, boost::posix_time::microseconds(timeout * 1e6));
    
    return getTimestampNoblockPrelocked(t);
  }

  ros::Time getTimestampNoblock(const ros::Time &data_time)
  {
    boost::mutex::scoped_lock data_lock(data_source_mutex_);
    boost::mutex::scoped_lock lock(mutex_);

    return getTimestampNoblockPrelocked(data_time);
  }
};

template <class C>
class QueuedTriggerMatcher : public TriggerMatcherBase
{
public:
  typedef std::pair<ros::Time, boost::shared_ptr<C const> > DataPair;
  typedef boost::function<void(const ros::Time &, boost::shared_ptr<C const> &)> MatchCallback;

private:
  MatchCallback match_callback_;
  std::queue<DataPair> data_queue_;
  unsigned int max_data_queue_length_;

  static void DefaultCallback(const ros::Time &t, boost::shared_ptr<C const> &d)
  {
    ROS_ERROR("QueuedTriggerMatcher triggered with no callback set. This is a bug in the code that uses QueuedTriggerMatcher.");
  }

protected:
  virtual void gotTrigger()
  {
    while (!data_queue_.empty())
    {
      DataPair &data = data_queue_.front();
      ros::Time stamp = getTimestampNoblockPrelocked(data.first);

      if (stamp == DropData)
        data_queue_.pop();
      else if (stamp == RetryLater)
        return;
      else
      {
        match_callback_(stamp, data.second);
        data_queue_.pop();
      }
    }
  }

public:
  virtual ~QueuedTriggerMatcher()
  {
  }
  
  void setMatchCallback(MatchCallback &cb)
  {
    match_callback_ = cb;
  }
  
  QueuedTriggerMatcher(unsigned int late_data_count_allowed, unsigned int max_trig_queue_length, unsigned int max_data_queue_length) :
    TriggerMatcherBase(late_data_count_allowed, max_trig_queue_length),
    max_data_queue_length_(max_data_queue_length)
  {
    match_callback_ = &QueuedTriggerMatcher::DefaultCallback;
  }
  
  void dataCallback(double stamp, const C &data)
  {
    dataCallback(ros::Time(stamp), data);
  }

  void dataCallback(const ros::Time &stamp, const C &data)
  {
    boost::shared_ptr<C const> ptr = boost::shared_ptr<C const>(new C(data));
    dataCallback(stamp, ptr);
  }

  void dataCallback(double stamp, const boost::shared_ptr<C const> &data)
  {
    dataCallback(ros::Time(stamp), data);
  }

  void dataCallback(const ros::Time &stamp, const boost::shared_ptr<C const> &data)
  {
    dataCallback(DataPair(stamp, data));
  }

  void reset()
  {
    boost::mutex::scoped_lock lock(mutex_);

    data_queue_ = std::queue<DataPair>();
    baseReset();
  }

  void dataCallback(const DataPair &pair)
  {
    boost::mutex::scoped_lock lock(mutex_);

    data_queue_.push(pair);
    if (data_queue_.size() > max_data_queue_length_)
    {
      if (verbose_)
        ROS_WARN("QueuedTriggerMatcher: trig_queue_ overflow dropping from front.");
      data_queue_.pop();
    }
    gotTrigger();
  }
};

}

#endif // __TIMESTAMP_TOOLS__TRIGGER_MATCHER_H__
