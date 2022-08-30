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
2. Switch to the branch `anton_thesis-evaluation`
3. Before starting the real trials, familiarize yourself with the controls by typing ```roslaunch svea ab_twinteleop_eval_test.launch```. Velocity and steering is controlled by clicking and dragging in the control window as shown below. Quit with ctrl-C and by manually closing the mouse control window. 

![teleop example](path_plan_sim.gif)

**Note**: For the trials below, a pre-defined path will appear and your task is to follow this path **as accurately as possible** until the program terminates by itself (after 3 laps). You may have to ctrl-C in the terminal and manually close the mouse control window to completely terminate the session. 

4. When ready, start the trial with **direct control** by typing ```roslaunch svea ab_twinteleop_eval_direct.launch```.
5. Next, start the trial with **indirect control** by typing ```roslaunch svea ab_twinteleop_eval_indirect.launch```.

You can repeat step 4 and 5 as many times as you want. When you are satisfied, please proceed to the final steps.

6. Locate the folder `twinteleop_data` in your home directory. It contains csv-logs from the trials. Please, email all csv-files to antonbjo@kth.se.
7. Finally, fill out the [survey](https://docs.google.com/forms/d/e/1FAIpQLSdQsmWEMSOpHen1Ca3LQoowDBwdYNFt-Find7_V-alkHvGmSg/viewform?usp=sf_link).

### Thank you for your participation!
