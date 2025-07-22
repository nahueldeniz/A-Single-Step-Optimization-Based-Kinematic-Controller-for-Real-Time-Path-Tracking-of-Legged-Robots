# A-Single-Step-Optimization-Based-Kinematic-Controller-for-Real-Time-Path-Tracking-of-Legged-Robots
A Single-Step Optimization-Based Kinematic Controller for Real-Time Path Tracking of Legged Robots
The main script is go1Sim.m, and it contains several helper functions.
The function build_setup(S) lets you adjust a variety of parameters, including:
    - sampling time Ts
    - control method
    - solver
    - noise models
    - constraints
and more.

In the function init() you can select the trajectory. The predefined paths are:
    - infinity (lemniscate)
    - rectangular
    - circular
    - rect
To add your own path, implement it in gen_path(S, type). Then, inside init(), choose it by calling
gen_path(S, 'YOUR_PATH_TYPE').

Make sure CasADi (https://web.casadi.org/) and the ACADO Toolkit (https://acado.github.io/) are both installed and configured correctly.
If the script doesn’t behave as expected, drop me a line at ndeniz@sinc.unl.edu.ar.
