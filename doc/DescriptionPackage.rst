Description Package
====================

The ``ridgeback_description`` package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Ridgeback. 

Ridgeback's URDF model can be visualized in rviz. In terminal, run:

.. code-block:: bash
    roslaunch ridgeback_viz view_model.launch

.. image:: images/ridgeback_urdf.png
    :alt: Ridgeback URDF

Accessories
------------

Ridgeback has a suite of optional payloads called accessories. These payloads can be enabled and placed on the robot using environment variables specified at the time the `xacro <http://wiki.ros.org/xacro>`_ is rendered to URDF. Available accessory vars are:

.. raw:: html

  <table><tbody>
    <tr> <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_CONTROL_EXTRAS</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>If <tt>1</tt> then the file specified by <tt>RIDGEBACK_CONTROL_EXRAS_PATH</tt> will be loaded</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_CONTROL_EXTRAS_PATH</tt></p></td>
      <td><p><i>undefined</i></p></td>
      <td><p>The path to a file containing additional custom controls</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_USE_MCU</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with an MCU</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_FRONT_HOKUYO_LASER</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a front-facing Hokuyo LIDAR unit (e.g. UST10)</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_FRONT_SICK_LASER</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a front-facing SICK LIDAR unit</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_FRONT_LASER_HOST</tt></p></td>
      <td><p><tt>192.168.131.20</tt></p></td>
      <td><p>The IP address of the front-facing LIDAR unit.  If using an S300 sensor, which does not communicate over TCP/IP, see below</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_FRONT_S300_LASER</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a front-facing S300 sensor</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_FRONT_S300_LASER_PORT</tt></p></td>
      <td><p><tt>/dev/clearpath/s300_front</tt></p></td>
      <td><p>The port that the front-facing S300 is connected to</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_REAR_HOKUYO_LASER</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a rear-facing Hokuyo LIDAR unit (e.g. UST10)</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_REAR_SICK_LASER</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a rear-facing SICK LIDAR unit</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_REAR_LASER_HOST</tt></p></td>
      <td><p><tt>192.168.131.21</tt></p></td>
      <td><p>The IP address of the rear-facing LIDAR unit.  If using an S300 sensor, which does not communicate over TCP/IP, see below</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_REAR_S300_LASER</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a rear-facing S300 sensor</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_REAR_S300_LASER_PORT</tt></p></td>
      <td><p><tt>/dev/clearpath/s300_rear</tt></p></td>
      <td><p>The port that the rear-facing S300 sensor is connected to</p></td>
    </tr>

    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROTRAIN_IMU</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> if the robot is equipped with a Microstrain IMU</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROTRAIN_IMU_LINK</tt></p></td>
      <td><p><tt>upgraded</tt></p></td>
      <td><p>Prepended to <tt>_imu_link</tt> to define the link the IMU is physically connected to</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROTRAIN_IMU_PORT</tt></p></td>
      <td><p><tt>/dev/microstrain</tt></p></td>
      <td><p>The port that the IMU is connected to</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROTRAIN_IMU_TOPIC  </tt></p></td>
      <td><p><tt>upgraded</tt></p></td>
      <td><p>Prepended to <tt>_imu</tt> to define the topic the IMU publishes to</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROSTRAIN_IMU_MOUNT</tt></p></td>
      <td><p><tt>mid</tt></p></td>
      <td><p>Prepended to <tt>_mount</tt> to define the mount point that the IMU link is connected to</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROSTRAIN_IMU_OFFSET</tt></p></td>
      <td><p><tt>0 0 0</tt></p></td>
      <td><p>The XYZ offset of the IMU relative to its mounting point (in meters)</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_MICROSTRAIN_IMU_RPY</tt></p></td>
      <td><p><tt>0 0 0</tt></p></td>
      <td><p>The Roll/Pitch/Yaw offset of the IMU relative to its mounting point (in radians)</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_PS3</tt></p></td>
      <td><p><tt>0</tt></p></td>
      <td><p>Set to <tt>1</tt> to enable teleoperation via PS3 controller</p></td>
    </tr>
    <tr>
      <td><span class="anchor" id="line-11"></span><p><tt>RIDGEBACK_URDF_EXTRAS</tt></p></td>
      <td><p><tt>empty.urdf</tt></p></td>
      <td><p>Path to a URDF file with additional modules connected to the robot</p></td>
    </tr>
  </tbody></table>
