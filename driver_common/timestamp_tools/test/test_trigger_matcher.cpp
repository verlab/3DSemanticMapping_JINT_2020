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

#include <timestamp_tools/trigger_matcher.h>
#include <gtest/gtest.h>
#include <queue>

/// @todo need to add a check for the hasTimestamp method.

class QueuedChecker
{
  std::queue< std::pair<double, int> > incoming_;

public:  
  timestamp_tools::QueuedTriggerMatcher<int> tm;
  
  QueuedChecker() : tm(1, 10, 5)
  {
    tm.setTrigDelay(0.2);
    timestamp_tools::QueuedTriggerMatcher<int>::MatchCallback bound_cb = boost::bind(&QueuedChecker::callback, this, _1, _2);
    tm.setMatchCallback(bound_cb);
                  
    // Get it into the permanent regime with an empty queue.
    tm.triggerCallback(0);
    expectEmpty(0);
    tm.dataCallback(1, 1);
    expectEmpty(1);
    tm.triggerCallback(2);
    expectHead(0.2, 1);
    expectEmpty(2);
    tm.dataCallback(3, 2);
    expectHead(2.2, 2);
    expectEmpty(3);
  }

  void expectEmpty(double time)
  {
    EXPECT_TRUE(incoming_.empty()) << "QueuedChecker queue not empty at time " << time 
      << " contains " << incoming_.front().first << ", " << incoming_.front().second;
  }

  void expectHead(double time, int data)
  {
    if (incoming_.empty())
    {
      ADD_FAILURE() << "QueuedChecker queue empty when checking " << time << ", " << data;
      return;
    }

    std::pair<double, int> &head = incoming_.front();

    EXPECT_EQ(time, head.first) << "Timestamp mismatch when checking " << time << ", " << data;
    EXPECT_EQ(data, head.second) << "Data mismatch when checking " << time << ", " << data;

    incoming_.pop();
  }

  void callback(const ros::Time &time, const boost::shared_ptr<int const> &data)
  {
    incoming_.push(std::pair<double, int>(time.toSec(), *data));
  }
};

TEST(QueuedTriggerMatcher, BasicFunctionality)
{
  QueuedChecker c;
  
// Data gets delayed...
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.dataCallback(5, 3);
  c.expectHead(4.2, 3);
  c.expectEmpty(5);
  c.tm.dataCallback(7, 4);
  c.expectHead(6.2, 4);
  c.expectEmpty(7);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5);
  c.expectEmpty(9);

// Timestamp gets delayed...
  c.tm.dataCallback(11, 6);
  c.expectEmpty(11);
  c.tm.dataCallback(13, 7);
  c.expectEmpty(13);
  c.tm.dataCallback(15, 8);
  c.expectEmpty(15);
  c.tm.triggerCallback(10);
  c.expectHead(10.2, 6);
  c.expectEmpty(10);
  c.tm.triggerCallback(12);
  c.expectHead(12.2, 7);
  c.expectEmpty(12);
  c.tm.triggerCallback(14);
  c.expectHead(14.2, 8);
  c.expectEmpty(14);
}

TEST(QueuedTriggerMatcher, TestReset)
{
  QueuedChecker c;

  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.reset();
  c.tm.dataCallback(5, 3);
  c.expectEmpty(5);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.dataCallback(7, 4);
  c.expectEmpty(7);
  c.tm.triggerCallback(8);
  c.expectHead(6.2, 4);
  c.expectEmpty(8);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5);
  c.expectEmpty(9);

  c.tm.dataCallback(5, 3);
  c.expectEmpty(5);
  c.tm.reset();
  c.expectEmpty(5.1);
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.dataCallback(7, 4);
  c.expectEmpty(7);
  c.tm.triggerCallback(8);
  c.expectHead(6.2, 4);
  c.expectEmpty(8);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5);
  c.expectEmpty(9);
}

TEST(QueuedTriggerMatcher, MissingTrigger)
{
  QueuedChecker c;
  
  // Miss a trigger at time 4...
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.dataCallback(5, 3);
  c.expectEmpty(5);
  c.tm.dataCallback(7, 4);
  c.expectHead(6.2, 4);
  c.expectEmpty(7);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5);
  c.expectEmpty(9);
}

TEST(QueuedTriggerMatcher, MissingData)
{
  QueuedChecker c;
  
  // Miss data at time 5...
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.triggerCallback(10);
  c.expectEmpty(10);
  c.tm.triggerCallback(12);
  c.expectEmpty(12);
  c.tm.dataCallback(7, 4);
  c.expectHead(4.2, 4); // Bad
  c.expectEmpty(7);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5); // Recovered
  c.expectEmpty(9);
  c.tm.dataCallback(11, 6);
  c.expectHead(10.2, 6); 
  c.expectEmpty(11);
  c.tm.dataCallback(13, 7);
  c.expectHead(12.2, 7); 
  c.expectEmpty(13);
}

TEST(QueuedTriggerMatcher, MissingDataZeroLateTolerance)
{
  QueuedChecker c;
  
  c.tm.setLateDataCountAllowed(0);

  // Miss data at time 5...
  c.tm.triggerCallback(4);
  c.expectEmpty(4);
  c.tm.triggerCallback(6);
  c.expectEmpty(6);
  c.tm.triggerCallback(8);
  c.expectEmpty(8);
  c.tm.triggerCallback(10);
  c.expectEmpty(10);
  c.tm.triggerCallback(12);
  c.expectEmpty(12);
  c.tm.triggerCallback(14);
  c.expectEmpty(14);
  c.tm.dataCallback(5, 4);
  c.expectHead(4.2, 4); 
  c.expectEmpty(5);
  c.tm.dataCallback(9, 5);
  c.expectHead(8.2, 5); // Recovered
  c.expectEmpty(9);
  c.tm.dataCallback(11, 6);
  c.expectHead(10.2, 6); 
  c.expectEmpty(11);
  c.tm.dataCallback(13, 7);
  c.expectHead(12.2, 7); 
  c.expectEmpty(13);
}

TEST(QueuedTriggerMatcher, TriggerQueueOverflow)
{
  QueuedChecker c;

  double trig_time = 4;
  for (int i = 0; i < 15; i++)
  {
    c.tm.triggerCallback(trig_time);
    c.expectEmpty(trig_time);
    trig_time += 2;
  }
  c.tm.dataCallback(15, 15);
  c.expectHead(14.2, 15); // Previous triggers were dropped
  c.expectEmpty(15);
}

TEST(QueuedTriggerMatcher, DataQueueOverflow)
{
  QueuedChecker c;

  double data_time = 5;
  for (int i = 0; i < 10; i++)
  {
    c.tm.dataCallback(data_time, data_time);
    c.expectEmpty(data_time);
    data_time += 2;
  }
  c.tm.triggerCallback(14);
  c.expectHead(14.2, 15); // Previous triggers were dropped
  c.expectEmpty(14);
}

TEST(TriggerMatcher, TimeoutCheck)
{
  timestamp_tools::TriggerMatcher tm(1, 10);

  // If this does not return because the timestamp
  EXPECT_EQ(timestamp_tools::TriggerMatcher::RetryLater, tm.getTimestampBlocking(ros::Time(0), 0.5));
}

TEST(TriggerMatcher, TriggerFirstCheck)
{
  timestamp_tools::TriggerMatcher tm(1, 10);

  tm.triggerCallback(1);
  tm.triggerCallback(2);
  tm.triggerCallback(3);

  EXPECT_EQ(ros::Time(1), tm.getTimestampBlocking(ros::Time(1.5), 1)) << "Testing getTimestampBlocking without timeout";
  EXPECT_EQ(ros::Time(2), tm.getTimestampBlocking(ros::Time(2.5))) << "Testing getTimestampBlocking with timeout";
}

TEST(TriggerMatcher, TestReset)
{
  timestamp_tools::TriggerMatcher tm(1, 10);

  tm.triggerCallback(1);
  tm.triggerCallback(2);
  tm.reset();
  tm.triggerCallback(3);
  tm.triggerCallback(4);

  EXPECT_EQ(timestamp_tools::TriggerMatcher::DropData, tm.getTimestampBlocking(ros::Time(1.5), 1)) << "Testing getTimestampBlocking without timeout";
  EXPECT_EQ(ros::Time(3), tm.getTimestampBlocking(ros::Time(3.5))) << "Testing getTimestampBlocking with timeout";
}


void AsyncGenTrigger(timestamp_tools::TriggerMatcher *tm, double time, int delay)
{
  sleep(delay);
  tm->triggerCallback(time);
}

TEST(TriggerMatcher, DataFirstCheck)
{
  timestamp_tools::TriggerMatcher tm(1, 10);

  tm.triggerCallback(5);
  boost::function<void(void)> agt = boost::bind(&AsyncGenTrigger, &tm, 7.0, 2);
  boost::thread trigger_thread(agt);

  EXPECT_EQ(timestamp_tools::TriggerMatcher::RetryLater, tm.getTimestampBlocking(ros::Time(5.5), 0.5)) << "getTimestampBlocking should have timed out or test computer is VERY slow";
  EXPECT_EQ(ros::Time(5), tm.getTimestampBlocking(ros::Time(6.0))) << "getTimestampBlocking should have received timestamp";

  trigger_thread.join();
}


int main(int argc, char **argv){
  for (int i = 0; i < argc; i++)
    printf("%s\n", argv[i]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
