#!/usr/bin/env python
"""
.. module:: main
    :platform: Windows
    :synopsis: The main python script in ``zed-oculus-spot`` package

.. moduleauthor:: Ali Yousefi <ali.yousefi@edu.unige.it>

	Gets executed by the main.cpp`` file. Uses the ``SpotInterface`` and ``Controller`` classes methods for the headtracking and locomotion tasks.

    Uses named pipe:
        MyPipe

"""
import os
import time
import struct
from spot_interface import SpotInterface
from controller import Controller

def get_log(omega_z, omega_y, v_x, v_y, v_rot):
    """
        In case that is required, uncomment it to get a log of the head tracking task results on terminal.

        Args:
                omega_z([float])
                omega_y([float])
                v_x([float])
                v_y([float])
                v_rot([float])
    """
    if omega_y > 0.05:
        if abs(omega_z) < 0.05:
            print("facing down")
        if omega_z > 0.05:
            print("facing down left")
        elif omega_z < -0.05:
            print("facing down right")
    elif omega_y < -0.05:
        if abs(omega_z) < 0.05:
            print("facing up")
        elif omega_z > 0.05:
            print("facing up left")
        elif omega_z < -0.05:
            print("facing up right")
    elif abs(omega_y) < 0.05:
        if omega_z > 0.05:
            print("facing left")
        elif omega_z < -0.05:
            print("facing right")   
    if v_x > 0:
        print("moving forward") 
    elif v_x < 0:
        print("moving backward")
    if v_y > 0:
        print("moving left")
    elif v_y < 0:
        print("moving right")
    if v_rot > 0:
        print("rotating left")
    elif v_rot < 0:
        print("rotating right")



def main():
    """
        The main function of the head tracking and locomotion task. Uses the named pipe ``MyPipe`` to get IMU data from the HMD, 
        and the ``get_body_vel()`` to get IMU data from the robot. In the main loop the control signals are computed using the 
        methods of ``Controller`` class, and sent to the robot using the ``SpotInterface`` class.

    """
    pipe_path = r'\\.\pipe\MyPipe'
    pipe = open(pipe_path, 'rb')
    spot = SpotInterface()
    controller = Controller()
    start_time = time.time()
    prev_time = start_time
    try:
        while True:
            current_time = time.time()
            dt = current_time - prev_time
            data = pipe.read(24) 
            received_tuple = struct.unpack('6f', data)
            setpoints = list(received_tuple)
            hmd_setpoints = setpoints[0:3]
            touch_setpoints = setpoints[3:6]
            spot_measures = spot.get_body_vel()
            hmd_errors = [a - b for a, b in zip(spot_measures, hmd_setpoints)]
            hmd_controls = controller.get_hmd_controls(hmd_errors)
            touch_controls = controller.get_touch_controls(touch_setpoints)
            controls = hmd_controls + touch_controls
            # get_log(hmd_setpoints[0], hmd_setpoints[1], touch_controls[0], touch_controls[1], touch_controls[2])
            print(setpoints)
            spot.set_controls(controls, dt)
            time.sleep(0.01)
            prev_time = current_time

    except KeyboardInterrupt:
        pass
    
    finally:
        pipe.close()
        spot.shutdown()
        

if __name__ == '__main__':
    main()
