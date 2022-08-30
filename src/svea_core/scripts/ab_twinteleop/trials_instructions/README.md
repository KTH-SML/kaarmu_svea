# Trials

Please read the information below carefully. The complete test takes around 15 minutes (excluding time for potential installation).

## Intro
Thank you for wanting to participate in the trials to evaluate a novel teleoperation concept for my master thesis!

The goal is to navigate one SVEA vehicle around a looping track under influence of simulated network latencies.

You will use two different control methods.

#### Method 1 - Direct control
You control the SVEA vehicle directly. You may experience communication delays due to network latencies - this is intentional.

#### Method 2 - Indirect control
You control a virtual representation of the SVEA vehicle - a digital twin. The actual SVEA vehicle manages itself and you will see it following the digital twin that you control.

## Instructions

Below are instructions for running the trials.

1. Install the SVEA Research codebase by by following the instructions [here](https://github.com/KTH-SML/svea_research/blob/master/README.md).
2. Make sure scipy version 1.2.3 and cvxpy version 1.0.29 are installed (other versions may work, but this has not been tested).

```
pip install scipy==1.2.3
pip install cvxpy==1.0.29
```

3. Switch to the branch `anton_thesis-evaluation`
4. Before starting the real trials, familiarize yourself with the controls by typing ```roslaunch svea ab_twinteleop_eval_test.launch``` and wait ~10 seconds for the vehicle to load. Velocity and steering is controlled by clicking and dragging in the control interface window as shown below. Quit with ctrl-C and by manually closing the mouse control window. 

![teleop example](path_plan_sim.gif)

5. When ready, start the trial with **direct control** by typing ```roslaunch svea ab_twinteleop_eval_direct.launch``` and wait ~10 seconds for the vehicle to load. When the cuboid representing the SVEA vehicle appears, drive as accurately as possible along the green path for 3 laps until the program terminates by itself. Close the mouse interface window manually and ctrl + C in the terminal to completely terminate the session before moving on.
6. Next, start the trial with **indirect control** by typing ```roslaunch svea ab_twinteleop_eval_indirect.launch``` and wait ~10 seconds for the vehicle to load. Then, drive along the green path as in the previous step until the program terminates by itself.

You can repeat step 4 and 5 as many times as you want. When you are satisfied, proceed to the final steps.

7. Locate the folder `twinteleop_data` in your home directory. It contains csv-logs from the trials. Email all csv-files to antonbjo@kth.se.
8. Finally, fill out the [survey](https://docs.google.com/forms/d/e/1FAIpQLSdQsmWEMSOpHen1Ca3LQoowDBwdYNFt-Find7_V-alkHvGmSg/viewform?usp=sf_link).

### Thank you for your participation!
