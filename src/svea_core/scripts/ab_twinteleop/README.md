# Teleoperation with a Virtual Environment Interface

Here is code for testing and evaluating the concept of vehicle teleoperation using a virtual environment interface in simulation as examined in the master thesis by Anton Björnberg (anton.035@hotmail.com) titled *Shared Control for vehicle Teleoperation with a Virtual Environment Interface*. The thesis can be found [here](http://kth.diva-portal.org/smash/record.jsf?pid=diva2%3A1504690&dswid=-7810).
It is thus a shared control concept where an operator demonstrates their driving intentions by maneuvering a virtual vehicle, which the real vehicle attempts to mimic. Within the scope of the work, RVIZ is used to visualize the virtual environment which is static and known *a priori*.

Longitudinal control is achieved with an approach inspired by adaptive cruise control and PID controller.
Lateral control is achieved with MPC and a linear bicycle model.
Local path planning for obstacle avoidance is achieved with a genetic algorithm.

## Overview

Within the framework of the thesis, the main script files produced are:
* `control_tower_module.py`
  * Main script for the system running at the operator end, which is refferred to as the *Control Tower*.
  * Contains functions for the operator to control the virtual vehicle.
* `telerobot_module.py`
  * Main script for the system running at the real vehicle, the *Telerobot*.
  * Manages motion control using PID and MPC to stabilize to a given path.
* `genetic_path_planner.py`
  * Methods for local path planning using a genetic algorithm.
* `path_handler.py`
  * Classes and helper functions for managing path representations
* `ab_twinteleop_controllers.py`
  * Classes for PID and MPC control
* `ab_bezier_path.py`
  * A modified version of the code for generating Beziér paths provided by PythonRobotics. In this modified version, computation time is decreased by utilizing numpy-arrays instead of for-loops.

In addition, the following minor scripts were used for testing and evaluation:
* `delay_sim.py`
  * Functions to simulate communication latency.
* `eval_logger.py`
  * Functions to log data from tests and record to simple csv-files.
* `control_tower_module_eval.py`
  * A modified version of `control_tower_module.py` for evaluation purposes with added simulated latency and lap counts etc.
  
Two maps were predetermined and used for local path planning. They are represented by gridmaps and stored as csv-files. Functions for generating these from a probability gridmap are contained in the `MapHandler` class in `path_handler.py`
* `binmap.csv`
  * A binary gridmap where a 1 indicates that the cell is occupied by an obstacle, and 0 that the cell is free.
* `distmap.csv`
  * A precomputed mapping where every cell contains the smallest distance to an obstacle.
  * The top row contains metadata corresponding to cell resolution and origin coordinates.
  
## Required packages
The system requires the following packages to be installed (in addition to those required for the main SVEA codebase):
* Scipy version 1.2.3 (install with `pip install scipy==1.2.3`)
* cvxpy version 1.0.29 (install with `pip install cvxpy==1.0.29`)

## Running

To run system demonstrations in simulation, type `roslaunch svea LAUNCHFILE`, where `LAUNCHFILE` is replaced with one of the below files. Note that one must wait around 10 seconds for the system to launch, and that controls are issued with a mouse interface that appears in a separate window upon launch. 

* `ab_twinteleop_sim.launch`
  * For simple demonstration of the system.
* `ab_twinteleop_eval_direct.launch`
  * For evaluation purposes, simulating the case of using direct control with communication latency.
* `ab_twinteleop_eval_indirect.launch`
  * For evaluation purposes, simulating the case of using shared control with the virtual environment interface.
* `ab_twinteleop_eval_test.launch`
  * For evaluation purposes, for test participants to become familiar with the controls.
