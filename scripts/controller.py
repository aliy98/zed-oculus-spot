#!/usr/bin/env python
"""
.. module:: controller
    :platform: Windows
    :synopsis: The controller python script in ``zed-oculus-spot`` package

.. moduleauthor:: Ali Yousefi <ali.yousefi@edu.unige.it>
	Provides a simple closed-loop controller for the head tracking task using the ``simple-pid`` python module with the method ``get_hmd_controls(setpoints)``. 
    Additionally, computes the locomotion control signals based on the touch input reference signals with the method ``get_touch_controls(setpoints)``.
"""
from simple_pid import PID

INPUT_TRESHOLD = 0.5
VELOCITY_BASE_SPEED = 0.5 # m/s
VELOCITY_BASE_ANGULAR = 0.8 # m/s
MAX_ANG_VEL = 0.8 # rad/s

class Controller:
    """
        Defines the controller gain variables. Provides the required methods for the HMD and touch controllers.
    """
    def __init__(self):
        self._kp_ang = 1
        self._ki_ang = 0
        self._kd_ang = 0

        self._pid_ang_x = PID(self._kp_ang, self._ki_ang, self._kd_ang)
        self._pid_ang_y = PID(self._kp_ang, self._ki_ang, self._kd_ang)
        self._pid_ang_z = PID(self._kp_ang, self._ki_ang, self._kd_ang)

        self._pid_ang_x.output_limits = (-MAX_ANG_VEL, MAX_ANG_VEL)
        self._pid_ang_y.output_limits = (-MAX_ANG_VEL, MAX_ANG_VEL)
        self._pid_ang_z.output_limits = (-MAX_ANG_VEL, MAX_ANG_VEL)

        self._pid_ang_x.set_auto_mode(enabled=True)
        self._pid_ang_y.set_auto_mode(enabled=True)
        self._pid_ang_z.set_auto_mode(enabled=True)


    def get_touch_controls(self, setpoints):
        """
            Computes the touch control signals by checking the touch input values. The control signals will be constant velocities in the case if 
            the touch inputs have passed the defined dead-zones.

            Args:
                setpoints([float])
            
            Returns:
                touch_controls([float])
        """
        touch_controls = []
        if abs(setpoints[0]) > INPUT_TRESHOLD:
            if setpoints[0] > 0:
                touch_controls.append(VELOCITY_BASE_SPEED)
            else:
                touch_controls.append(-VELOCITY_BASE_SPEED)
        else:
            touch_controls.append(0)

        if abs(setpoints[1]) > INPUT_TRESHOLD:
            if setpoints[1] > 0:
                touch_controls.append(-VELOCITY_BASE_SPEED)
            else:
                touch_controls.append(VELOCITY_BASE_SPEED)
        else:
            touch_controls.append(0)

        if abs(setpoints[2]) > INPUT_TRESHOLD:
            if setpoints[2] > 0:
                touch_controls.append(-VELOCITY_BASE_ANGULAR)
            else:
                touch_controls.append(VELOCITY_BASE_ANGULAR)
        else:
            touch_controls.append(0)
    
        return touch_controls
    
    def get_hmd_controls(self, errors):
        """
            Computes the HMD control signals by checking the errors between the HMD and robot IMU values. The control signals will be computed based on the 
            closed-loop system gain values.

            Args:
                setpoints([float])
            
            Returns:
                hmd_controls([float])
        """
        hmd_controls = []
        hmd_controls.append(self._pid_ang_z(errors[0]))
        hmd_controls.append(self._pid_ang_y(errors[1]))
        hmd_controls.append(self._pid_ang_x(errors[2]))
        return hmd_controls
        



