^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_joint_trajectory_plot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2020-03-07)
------------------
* Add melodic release(`#13 <https://github.com/tork-a/rqt_joint_trajectory_plot/issues/13>`_)
  - Set CI python version to 3.6
  - Merge pull request 
* added ability to plot planned_path from moveit and made rqt launch file(`#10 <https://github.com/tork-a/rqt_joint_trajectory_plot/issues/10>`_)
  - updated package.xml to include moveit_msgs for plotting display_trajectory messages
* fixed example PVA image size
* added empty display trajectory check and updated readme with instrucitons to launch PVA perspective
* added ability to plot planned_path from moveit and made rqt launch file with custom perspective
* Fix workspace name for CI (`#11 <https://github.com/tork-a/rqt_joint_trajectory_plot/issues/11>`_)
  - Fix workspace name as https://github.com/tork-a/jog_control/pull/37/files
* added empty display trajectory check and updated readme with instrucitons to launch PVA perspective
* added ability to plot planned_path from moveit and made rqt launch file with custom perspective
* Add travis.yml
* Contributors: Ryosuke Tajima, marqrazz

0.0.4 (2018-06-14)
------------------
* Fix lint problem
* Contributors: Ryosuke Tajima

0.0.3 (2018-06-14)
------------------
* Check the qt version in plot_widget.py for indigo release(`#9 <https://github.com/tork-a/rqt_joint_trajectory_plot/issues/9>`_)
* Contributors: Ryosuke Tajima

0.0.2 (2018-06-08)
------------------
* Add script/rqt_joint_trajectory_plot(`#7 <https://github.com/tork-a/rqt_joint_trajectory_plot/issues/7>`_)
* Update to visualize FollowJointTrajectoryActionGoal(`#6 <https://github.com/tork-a/rqt_joint_trajectory_plot/issues/6>`_)
* Contributors: Kei Okada, Ryosuke Tajima

0.0.1 (2018-01-20)
------------------
* Organize and format package files
  - roslint check
  - autopep8 for python scripts
* Port to PyQt5 as used in kinetic(`#1 <https://github.com/7675t/rqt_joint_trajectory_plot/issues/1>`_)
* Port to PyQt5
  no toolbar so far, apparently this is not yet(?!) available in matplotlibs Qt5Agg backend.
* add screenshot to README.md
* add a screenshot file.
* Initial commit
* Contributors: Ryosuke Tajima, Simon Schmeisser
