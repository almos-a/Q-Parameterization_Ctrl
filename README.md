# Q-Parameterization_Ctrl
Design of Q-Parameterization controller in MATLAB

In Q_par_1, Q_par_2; the design of the controller for the plant is such that the feedback system is internally stable. For a unit step input with no disturbance, the output value is 1 (in accordance to the tracking theorem, this requirement is achieved if the sensitivity is 0 or if the controller has a pole at s=0, C(0)=inf). With a sinusoidal disturbance and zero input the output goes to zero (to reject sinusoidal disturbance of frequecy jf, either the tf from disturbace to output must have a zero at, or the controller has poles at: s=+/-jf where f is the value).
Pole placement and double coprime factorization are used.


Q_par_3; reflects tracking response of the closed loop system to a ramp input


For any comments, contributions, corrections. Please feel free to reach out
