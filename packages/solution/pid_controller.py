#!/usr/bin/env python3

from typing import Tuple
import numpy as np

class PIDController():
    def __init__(self):

        # We will initialize some variables that might be useful
        self.prev_e_heading = 0.0
        self.prev_e_offset = 0.0
        self.prev_int_heading = 0.0
        self.prev_int_offset = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0


    def HeadingControl(self,
                       v_ref: float,
                       theta_ref: float,
                       theta_curr: float,
                       delta_t: float
    ) -> Tuple[float, float]:
        """
        PID performing heading control.
        Args:
            v_ref:      reference velocity.
            theta_ref:  reference heading pose.
            theta_curr: the current estimated heading.
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference heading
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_heading to track the integral term
        # self.prev_e_heading the previous error. But note that you
        # should be the one to update them also.

        if delta_t > 1.0:
            self.prev_int_heading = 0.0
            self.prev_e_heading = 0.0
            delta_t = 0.0

        r_t = theta_ref
        y_t = theta_curr

        e_t = r_t - y_t
        e_t = np.arctan2(np.sin(e_t), np.cos(e_t)) # Normalizing angular error between [-pi, +pi]
        
        integral = self.prev_int_heading + e_t * delta_t
        integral = np.clip(integral, -2.0, 2.0)

        derivative = (e_t - self.prev_e_heading) / delta_t if delta_t > 0.0 else 0.0

        omega = self.kp * e_t + self.ki * integral + self.kd * derivative
        v = v_ref

        self.prev_int_heading = integral
        self.prev_e_heading = e_t

        print(f"[PID] ref={r_t:.3f} | curr={y_t:.3f} | err={e_t:.3f} | int={integral:.3f} | omega={omega:.3f}", flush=True)
        return v, omega

    def OffsetControl(self,
                      v_ref: float,
                      y_ref: float,
                      y_curr: float,
                      delta_t: float
                      ) -> Tuple[float, float]:
        """
        PID performing lateral offset control.
        Args:
            v_ref:      linear Duckiebot speed.
            y_ref:      reference heading pose.
            y_curr:     the current estimated "y" coordinate (offset)
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference lateral offset
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_offset to track the integral term
        # self.prev_e_offset the previous error. But note that you
        # should be the one to update them also.

        if delta_t > 1.0:
            self.prev_int_heading = 0.0
            self.prev_e_heading = 0.0
            delta_t = 0.0

        r_t = y_ref
        y_t = y_curr

        e_t = r_t - y_t    
        
        integral = self.prev_int_offset + e_t * delta_t
        integral = np.clip(integral, -2.0, 2.0)

        derivative = (e_t - self.prev_e_offset) / delta_t if delta_t > 0.0 else 0.0

        omega = self.kp * e_t + self.ki * integral + self.kd * derivative
        v = v_ref

        self.prev_int_offset = integral
        self.prev_e_offset = e_t

        print(f"[PID] ref={r_t:.3f} | curr={y_t:.3f} | err={e_t:.3f} | int={integral:.3f} | omega={omega:.3f}", flush=True)
        return v, omega

    def SetGains(self, kp: float, ki: float, kd: float) -> None:
        # Set the PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_e_heading = 0.0
        self.prev_e_offset = 0.0
        self.prev_int_heading = 0.0
        self.prev_int_offset = 0.0