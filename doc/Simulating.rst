Simulating Ridgeback
====================

Whether you actually have a Ridgeback robot or not, the Ridgeback simulator is a great way to get started with ROS robot development. In this tutorial, we will go through the basics of starting Gazebo and rviz and how to drive your Ridgeback around.

Installation
------------

To get started with the Ridgeback simulation, make sure you have a :roswiki:`working ROS installation <ROS/Installation>`
set up on your Ubuntu desktop, and install the Ridgeback-specific metapackages for desktop and simulation:

.. parsed-literal::

    sudo apt-get install ros-melodic-ridgeback-simulator ros-melodic-ridgeback-desktop


Launch Gazebo
-------------

Gazebo is the most common simulation tool used in ROS. Ridgeback's model in Gazebo include reasonable
approximations of its dynamics, including wheel slippage, skidding, and inertia. To launch simulated
Ridgeback in a simple example world, run the following command:

.. code-block:: bash

    roslaunch ridgeback_gazebo ridgeback_world.launch

You should see the following window appear, or something like it. You can adjust the camera angle by
clicking and dragging while holding CTRL, ALT, or the shift key:

.. image:: images/Simulation1.png
    :alt: Simulated Ridgeback in the Race World.

The window which you are looking at is the Gazebo Client. This window shows you the "true" state of the
simulated world which the robot exists in. It communicates on the backend with the Gazebo Server, which
is doing the heavy lifting of actually maintaining the simulated world. At the moment, you're running
both the client and server locally on your own machine, but some advanced users may choose to run heavy
duty simulations on separate hardware and connect to them over the network.


Launch rviz
-----------

The next tool we will encounter is :roswiki:`rviz`. Although superficially similar in appearance to Gazebo,
rviz has a very different purpose— unlike Gazebo, which shows the reality of the simulated world, rviz shows
the robot's *perception* of its world, whether real or simulated. So while Gazebo won't be used with your
real Ridgeback, rviz is used with both.

You can using the following launch invocation to start rviz with a pre-cooked configuration suitable for
visualizing any standard Ridgeback config:

.. code-block:: bash

    roslaunch ridgeback_viz view_robot.launch

You should see rviz appear:

.. image:: images/Simulation2.png
    :alt: Ridgeback with laser scanner in rviz.

The rviz display only shows what the robot knows about its world, which presently, is nothing. Because the
robot doesn't yet know about the barriers which exist in its Gazebo world, they're not shown here.


Driving Ridgeback
-----------------

What is shown, however, is Ridgeback's interactive markers. These are the simplest way to command your robot
to move around. If you don't see them in your rviz display, select the Interact tool from the top toolbar.
You should see red arrows and a blue circle appear around the Ridgeback model.

Drag the red arrows in Rviz to move in the linear x and the blue circle to move in the angular z. Rviz shows you
Ridgeback moving relative to its odometric frame, but it is also moving relative to the simulated world supplied by
Gazebo. If you click over to the Gazebo window, you will see Ridgeback moving within its simulated world. Or, if you
drive real Ridgeback using this method, it will have moved in the real world.

Once you start your own development, have your nodes send ``geometry_msgs/Twist`` commands to the ``cmd_vel``
topic to drive Ridgeback, either real or simulated. This is the standard ROS interface to differential-drive and
holonomic ground vehicles.

You can also use a game controller to drive your robot.  Connect your controller using either a USB cable or Bluetooth
as appropriate and then launch the teleop node by running:

.. code-block:: bash

  roslaunch ridgeback_control teleop.launch joy_dev:=/dev/input/js0

Replace ``/dev/input/js0`` with the joy device you wish to use as input.  By default ``ridgeback_control`` accepts input
from ``/dev/input/ps4`` unless another device is specified.  If you use a PS4 controller, you can add the following udev
rule to automatically symlink your js* device to ``/dev/input/ps4``:

.. code-block:: bash

  KERNEL=="js*", SUBSYSTEM=="input", ATTRS{name}=="Wireless Controller", MODE="0666", SYMLINK+="input/ps4"

Put the above in ``/etc/udev/rules.d/41-playstation.rules`` and then run

.. code-block:: bash

  sudo udevadm control --reload-rules
  sudo udevadm trigger

If you use a different game controller, e.g. an Xbox controller or Logitech F710 you will need to specify the device
using the ``joy_dev:=/dev/input/js*`` argument, described earlier.

Regardless of the controller, Axis 0 controls the robot's side-to-side movement, Axis 1 controls the robot's
forward/backward velocity, and Axis 2 controls the robot's steering.  Buttons 4 and 5 act as enable and
enable-turbo respectively.  On common controllers these correspond to the following physical controls:

============= ==================================== ===== ===== =========
Axis/Button   Physical Input                       PS4   F710  Xbox One
============= ==================================== ===== ===== =========
Axis 0        Left thumb stick horizontal          LJ    LJ    LJ
Axis 1        Left thumb stick vertical            LJ    LJ    LJ
Axis 2        Right thumb stick horizontal         RJ    RJ    RJ
Button 4      Left shoulder button or trigger      L1    LB    LB
Button 5      Right shoulder button or trigger     R1    RB    RB
============= ==================================== ===== ===== =========


Visualizing Sensors
-------------------

The rviz tool is capable of visualizing many common robotic sensors, as well as other data feeds which can give
us clues as to what the robot is doing and why. A great place to start with this is adding the
:roswiki:`LaserScan <rviz/DisplayTypes/LaserScan>` plugin to visualize the laser scans being produced by the
simulated LMS111. In the left panel, click the "Add" button, then select the "Topics" tab, and then select the
``front/scan`` topic:

.. image:: images/Simulation3.png
    :alt: Adding a laser scan visualization to Ridgeback.

Click OK, and you should see laser scan points now visible in the rviz window, relative to the robot:

.. image:: images/Simulation4.png
    :alt: Visualizing Ridgeback with simulated laser scans.

If you use the interactive markers to drive around, you'll notice that the laser scan points move a little bit
but generally stay where they are. This is the first step toward map making using :roswiki:`gmapping`, which
is covered in the next tutorial, :doc:`Navigating`.
