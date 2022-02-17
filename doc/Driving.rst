Driving Ridgeback
=================

There are four ways to drive Ridgeback, and each way will work on a physical Ridgeback robot as well as on a simulated Ridgeback.

The first two ways are teleoperation using a remote controller, and manually through publishing ROS messages. These two ways are covered in this section.

The third way is using the interactive controller in rviz. This is covered in the :doc:`Simulation <Simulating>` section.

The fourth way is through autonomous navigation. This is covered in the :doc:`Navigation <Navigating>` section.

Safety Precautions
-------------------

.. Warning::

	Ridgeback is capable of reaching high speeds. Careless driving can cause harm to the operator, bystanders, the robot, or other property. Always remain vigilant, ensure you have a clear line of sight to the robot, and operate the robot at safe speeds. We strongly recommend driving in normal (slow) mode first, and only enabling turbo in large, open areas that are free of people and obstacles.

Driving with Remote Controller
---------------------------------

.. note::

	For instructions on controller pairing, see :doc:`Pairing Controller <PairingController>`.

To drive the Ridgeback, Axis 0 controls the robot's steering, Axis 1 controls the robot's left/right translation, and Axis 2 controls the robot's steering. Buttons 4 and 5 act as enable and enable-turbo respectively. On common controllers these correspond to the following physical controls:

============= ==================================== ===== ===== ========= =======================
Axis/Button   Physical Input                       PS4   F710  Xbox One  Action
============= ==================================== ===== ===== ========= =======================
Axis 0        Left thumb stick horizontal          LJ    LJ    LJ        Drive forward/backward
Axis 1        Left thumb stick vertical            LJ    LJ    LJ        Drive left/right
Axis 2        Right thumb stick horizontal         RJ    RJ    RJ        Rotate
Button 4      Left shoulder button or trigger      L1    LB    LB        Enable normal speed
Button 5      Right shoulder button or trigger     R1    RB    RB        Enable turbo
============= ==================================== ===== ===== ========= =======================

You must hold either Button 4 or Button 5 at all times while driving the robot.

Driving with ROS Messages
--------------------------

You can manually publish ``geometry_msgs/Twist`` ROS messages to either the ``/ridgeback_velocity_controller/cmd_vel`` or the ``/cmd_vel`` ROS topics to drive Ridgeback. 

For example, in terminal, run:

.. code-block:: bash

	rostopic pub /ridgeback_velocity_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

The command above makes Ridgeback drive forward momentarily at 0.5 m/s without any rotation. 

Emergency Stop
---------------

Ridgeback has four E-Stop buttons located on the four corners of the robot. Pressing any of these
buttons will cut power to the motors. To disengage the E-Stop, simply twist the button in the direction indicated by the arrows, and press the reset button on the rear of the robot.

Whenever you need to perform maintenance on Ridgeback, we recommend engaging the E-Stop if the robot cannot be fully powered down.