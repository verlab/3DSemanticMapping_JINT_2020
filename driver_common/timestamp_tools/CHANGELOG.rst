^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package timestamp_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.8 (2014-03-30)
------------------

1.6.7 (2013-08-21)
------------------
* No more Willow Garage email.
* Contributors: Chad Rockey

1.6.6 (2013-05-06)
------------------

1.6.5 (2013-01-22)
------------------
* Removed duplicated dependancies
  test_depend tags should only include additional dependencies required for testing. 
* Contributors: Aaron Blasdel

1.6.4 (2012-12-14)
------------------
* add missing dep to catkin
* Contributors: Dirk Thomas

1.6.3 (2012-12-13)
------------------

1.6.2 (2012-12-10)
------------------
* Version 1.6.2
* Removed non-existant library from timestamp_tools
* Contributors: William Woodall

1.6.1 (2012-12-07 16:02)
------------------------

1.6.0 (2012-12-07 11:00)
------------------------
* Wrong version number for timestamp tools.
* Version bump for release.
* Catkinized
* remove roslib::Header usage, bump to 1.2.5
* rosbuild2 taking shape
* Added Ubuntu platform tags to manifest
* driver_common: adding in brief descriptions
* Added license notices.
* Set reviewed tags on all packages.
* Used boost::posix_time::microseconds instead of boost::posix_time::seconds to avoid rounding problems.
* Rearranged condition variable in trigger matcher. No longer needs to use absolute times, and eliminated a race condition related to resetting the class.
* hasTimestamp was inverted in previous checkin.
* Added hasTimestamp method to the trigger matcher to determine if there is a timestamp waiting.
* Marked timestamp_tools as internal only.
* Marked timestamp_tools as internal only.
* Remove use of deprecated rosbuild macros
* Made trigger matcher less verbose by default.
* added XHTML to description
* Corrected Ptr to ConstPtr to make trigger_matcher callback compatible with
  subscribe.
* Updated some debug messages in trigger_matcher.
* Split TriggerMatcher into TriggerMatcher in which data is not queued and
  sent to a callback, and a QueuedTriggerMatcher with the previous
  functionality of TriggerMatcher.
* Added a reset method to the TriggerMatcher
* Wrote skeleton for PeriodicTimestampMinFilter.
* Wrote TriggerMatcher class to synchronize exact trigger time stamps with
  approximate timestamps incoming from hardware. Wrote self tests for it.
* First pass at writing the TriggerMatcher class. Duplicated the wge100 node
  in view of redoing the time stamping code.
* Fixed typo in timestamp_tools manifest.
* Renamed timestamp-tools to timestamp_tools.
* Contributors: Brian Gerkey, Chad Rockey, Gassend Blaise, Ken Conley, Rob Wheeler, Troy Straszheim
