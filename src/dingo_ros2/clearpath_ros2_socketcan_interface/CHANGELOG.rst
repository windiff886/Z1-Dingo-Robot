^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_ros2_socketcan_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2025-10-20)
------------------
* Added UNKNOWN to can enabling condition. (`#17 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/17>`_)
* Contributors: Tony Baltovski

1.0.3 (2025-07-23)
------------------
* Backports (`#16 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/16>`_)
  * Prefix node names with can interface
  * Wait for interface to be up before launching node (`#7 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/7>`_)
  * Wait for interface to be UP before starting node
  * Fix: Multiple IncludeLaunchDescription (`#9 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/9>`_)
  * Use opaque function
  * Add import and context
  * Type performed context
  * Reattempt transitions on failure (`#10 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/10>`_)
  * Reattempt lifecycle transitions on failure
  * Unique activator node names
  * Removed namespace from node name
  * Linting
  * Use arguments instead of perform(context) (`#11 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/11>`_)
  * Fix: Use script instead of OpaqueFunction (`#12 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/12>`_)
  * Use script instead of OpaqueFunction
  * Add missing line
  * Add license header
  * Add EOF line
  * Add retry in the event lifecycle service is not up yet
  * Fix: Spin Timeout (`#13 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/13>`_)
  * Add timeout to spin to prevent script from waiting when node has failed
  * Update log to match appropriate action
  * Fixed using a global namespace.
  ---------
  Co-authored-by: Roni Kreinin <rkreinin@clearpathrobotics.com>
  Co-authored-by: Roni Kreinin <59886299+roni-kreinin@users.noreply.github.com>
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
* Contributors: luis-camero

1.0.2 (2025-04-09)
------------------
* Increased timeout to 1 second (`#8 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/8>`_)
* Contributors: Roni Kreinin

1.0.1 (2024-12-17)
------------------
* Add our own launch files that allow us to change namespaces
* Contributors: Luis Camero

1.0.0 (2024-11-21)
------------------
* Change CI to Humble.
* Added README.
* Disabled copyright tests.
* Fixed linting.
* Added CI
* Added issue templates.
* Added codeowners.
* Updated package.xml
* Merge pull request `#2 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/2>`_ from clearpathrobotics/rkreinin/callback
  Constructor with callback
* Tx queue and wall timer
* Constructor with callback
  License
* Merge pull request `#1 <https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface/issues/1>`_ from clearpathrobotics/cib/jazzy-fixes
  Small code styling to make default tests pass
* Small code styling to make default tests pass
* Add delay before publishing messages
* Initial add of clearpath_ros2_socketcan_interface
* Initial commit
* Contributors: Chris Iverach-Brereton, Luis Camero, Roni Kreinin, Tony Baltovski, luis-camero
