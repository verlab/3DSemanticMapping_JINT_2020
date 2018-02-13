^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package driver_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.8 (2014-03-30)
------------------

1.6.7 (2013-08-21)
------------------
* No more Willow Garage email.
* Contributors: Chad Rockey

1.6.6 (2013-05-06)
------------------
* Fix a compile problem that occurs when using clang
* Contributors: William Woodall

1.6.5 (2013-01-22)
------------------
* Removed duplicated dependancies
  test_depend tags should only not duplicate run/build depends tags.
* Contributors: Aaron Blasdel

1.6.4 (2012-12-14)
------------------
* add missing dep to catkin
* Contributors: Dirk Thomas

1.6.3 (2012-12-13)
------------------
* fix missing dependency
* add missing downstream depend
* switched from langs to message_* packages
* Contributors: Dirk Thomas

1.6.2 (2012-12-10)
------------------
* Version 1.6.2
* Contributors: William Woodall

1.6.1 (2012-12-07 16:02)
------------------------

1.6.0 (2012-12-07 11:00)
------------------------
* Fixing driver_base package.xml and CMakelist
* Version bump for release.
* Catkinized
* rosbuild2 taking shape
* Made all the tests that change the driver state more reliable by doing multiple retries.
* Added Ubuntu platform tags to manifest
* Took out state_ from driver_node.h. It was a vestigial remain from an early version that was causing occasional shutdowns at node startup.
* driver_common: adding in brief descriptions
* Set reviewed tags on all packages.
* Fixed SIGUHP to SIGHUP.
* Added SIGHUP handler to driver_base. Prints a warning and continues.
* Took out message when reconfigure was unable to return to previous state, as it should be covered by other error messages.
* Changed diagnostic summary messages
* Switched over to ROS's default CTRL+C handler to better support CTRL+C when there is no master (and probably other cases too).
* Now holds the lock while doing diagnostics and self_test. Avoids some nasty problems when doing dynamic_reconfigure during self_test.
* Added ability to add diagnostic items to the Driver Status diagnostic. Better control over when status messages get printed.
* Fixed warning.
* Took node name out of driver status diagnostic heading since it is added by the diagnostic_updater.
* Got rid of a deadlock case when external reconfiguration happens at the same time as the device is trying to reconfigure itself.
* Removed obsolete services (seem to have been an early attempte at dynamic_reconfigure).
* cstdio include required for vsnprintf on ubuntu karmic (gcc 4.4)
* Changed sleep(0.1) in main loop into a ros::Duration(0.1).sleep. Doesn't eat CPU now.
* Renamed self_test::Sequencer to self_test::TestRunner as per API review.
* Added missing header
* Got rid of non-constant format string.
* Changed locking on status message to avoid deadlocks when exiting streaming thread.
* Added missing include of stdarg.h
* Added support for debugging messages that only get displayed when the driver state changes.
* Changed sleep behavior in main loop. Now sleeps for a second if there is a crash, but normally only sleeps for 100ms.
* Marked driver_base as internal only for now.
* Remove use of deprecated rosbuild macros
* Updated to new dynamic_reconfigure transport. Added setparam command to dynparam.
* Removed race condition from DriverNode that was causing a spurious device restart after reconfigure.
* Fixed include files, added exception handling, added node handle for private namespace.
* Continued debugging wge100_multi_reconfigurator. Fixed a locking bug in driver_common.
* added missing signal.h include
* updated self_test, diagnostic_updater, dynamic_reconfigure and wge100_camera to use new ~ namespace access method
* Updated package URLs.
* Typo in previous commit.
* Updated manifest.
* Split apart the open and ID self-test steps.
* Updated adds into add as part of the diagnostic message update.
* Added a postOpenHook.
* Bug slipped into previous commit.
* If self-test retries it now issues a warning.
* Increased led_test tolerance to 2. Allowed self test to retry open and start commands before failing.
* Forgotten changes to driver_base that were breaking the build.
* Changed nomenclature in driver_base. Renamed method names to use camelCase.
  Got forearm_node working with driver_base: now appears to be working well.
* Continuing to move driver_base.
* Moving driver_base.
* Contributors: Brian Gerkey, Chad Rockey, Gassend Blaise, Ken Conley, Morgan Quigley, Rob Wheeler, Troy Straszheim
