Pairing Controller
===================

Remote controllers are used for teleoperation; they allow you to remotely drive the Ridgeback, whether it is a physical Ridgeback robot, or a simulated Ridgeback. The following instructions below detail how to pair different controllers to the Ridgeback's computer; however, these instructions can also be used to pair these controllers to your own computer.

Logitech F710 Controller
---------------------------

.. Note::

  If your Ridgeback comes with a Logitech F710 controller, it will be paired already. Simply turn on the Ridgeback, plug the USB dongle into one of the Ridgeback's USB ports, and turn on the controller.

To re-pair the Logitech F710 controller or pair a new Logitech F710 controller, plug the controller's USB dongle into the Ridgeback's computer and turn on the controller. The controller automatically pair.

PS4 Controller
---------------

.. Note::

  If your Ridgeback comes with a PS4 controller, it will be paired already. Simply turn on the Ridgeback and turn on the controller.

To re-pair the PS4 controller or pair a new PS4 controller:

1. Install the ``python-ds4drv`` package if it is not installed already. In terminal, run:

.. code-block:: bash

  sudo apt-get install python-ds4drv

2. Put the controller in pairing mode. Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white.

3. Run the ``ds4drv-pair`` script to pair the controller to the computer. In terminal, run:

.. code-block:: bash

  sudo ds4drv-pair

This script will scan for nearby Bluetooth devices, and pair automatically to the controller.

Alternatively, if ``ds4drv-pair`` fails to detect the controller, you can pair the controller using ``bluetoothctl``:

1. Install the ``bluez`` package if it is not installed already. In terminal, run:

.. code-block:: text

  sudo apt-get install bluez

2. Run the ``bluetoothctl`` command. In terminal, run:

.. code-block:: text

  sudo bluetoothctl

3. Use ``bluetoothctl`` to scan for nearby devices. In the bluetooth control application, run:

.. code-block:: text

  agent on
  scan on

4. Put the controller in pairing mode. Press and hold the PS and Share buttons on your controller until the LED begins rapidly flashing white.

5. The bluetooth scan will display the MAC addresses of nearby devices. Determine which MAC address corresponds to the
controller and copy it. In the bluetooth control application, run:

.. code-block:: text

  scan off
  pair <MAC Address>
  trust <MAC Address>
  connect <MAC Address>

The controller should now be paired.

6. Once the controller is paired, you should see the file ``/dev/input/ps4`` appear in Ridgeback's terminal by running:

.. code-block:: bash

  ls -l /dev/input

You should see these lines in the output:

.. code-block:: bash

  crw-rw-rw-+ 1 root root  13,  0 Aug 10 12:16 js0
  lrwxrwxrwx  1 root root       3 Aug 10 12:16 ps4 -> js0

If you do not see the ps4 device, create the ``/etc/udev/rules.d/41-playstation.rules`` file and add the following line:

.. code-block:: text

  KERNEL=="js*", SUBSYSTEM=="input", ATTRS{name}=="Wireless Controller", MODE="0666", SYMLINK+="input/ps4"

Afterwards, reload ``udev``:

.. code-block:: bash

  sudo udevadm control --reload-rules
  sudo udevadm trigger