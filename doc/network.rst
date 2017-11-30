Setting Up Ridgeback's Network
===========================


First Connection
----------------

By default, Ridgeback's wireless is in client mode, looking for the wireless network at the Clearpath factory. In
order to set it up to connect to your own network, you'll have to open up the user bay and connect a network cable to
the PC's ``STATIC`` port. The other end of this cable should be connected to your laptop, and you should give yourself an IP address in the ``192.168.131.x`` space, such as ``192.168.131.50``. Then, make the connection to Ridgeback's default
static IP:

.. code-block:: bash

    ssh administrator@192.168.131.11

The default password is ``clearpath``. You should now be logged into Ridgeback as the administrator user.


Connecting to Wifi Access Point
--------------------------------

Once connected via wire, execute connmanctl to enter the command line interface for Connman, from which you can configure Ridgeback to either join an existing network, or supply its own standalone access point. An example session to connect to an existing network:

| connmanctl > enable wifi
| connmanctl > scan wifi
| connmanctl > services
| connmanctl > agent on
| connmanctl > connect wifi_123456_123456789123456789_managed_psk
| 

After the connect line, connman will prompt you for your network's passphrase. Once connected, connman will remember and attempt to reconnect on successive power-ons.


.. _remote:

Remote ROS Connection
---------------------

To use ROS desktop tools, you'll need your computer to be able to connect to Ridgeback's ROS master. This can be a
tricky process, but we've tried to make it as simple as possible.

In order for the ROS tools on your computer to talk to Ridgeback, they need to know two things:

- How to find the ROS master, which is set in the ``ROS_MASTER_URI`` environment variable, and
- How processes on the other computer can find *your computer*, which is the ``ROS_IP`` environment variable.

The suggested pattern is to create a file in your home directory called ``remote-ridgeback.sh`` with the following
contents:

.. code-block:: bash

    export ROS_MASTER_URI=http://cpr-ridgeback-0001:11311  # Ridgeback's hostname
    export ROS_IP=10.25.0.102                           # Your laptop's wireless IP address

If your network doesn't already resolve Ridgeback's hostname to its wireless IP address, you may need to add
a corresponding line to your computer's ``/etc/hosts`` file:

.. code-block:: bash

    10.25.0.101 cpr-ridgeback-0001

Then, when you're ready to communicate remotely with Ridgeback, you can source that script like so, thus defining
those two key environment variables in the present context.

.. code-block:: bash

    source remote-ridgeback.sh

Now, when you run commands like ``rostopic list``, ``rostopic echo``, ``rosnode list``, and others, the output 
you see should reflect the activity on Ridgeback's ROS master, rather than on your own machine. Once you've
verified the basics (list, echo) from the prompt, try launching some of the standard visual ROS tools:

.. code-block:: bash

    roslaunch ridgeback_viz view_robot.launch
    rosrun rqt_robot_monitor rqt_robot_monitor
    rosrun rqt_console rqt_console

If there are particular :roswiki:`rqt` widgets you find yourself using a lot, you may find it an advantage to dock them together
and then export this configuration as the default RQT perspective. Then, to bring up your standard GUI, you can simply
run:

.. code-block:: bash

    rqt


