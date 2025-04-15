# IMRL Workspace

The structure of the IMRL workspace is as follows:

imrl_workspace/ <br>
├── fleet_management/ <br>
├── json_examples/ <br>
├── motion_control/ <br>
└── README.md <br>

In the folders `fleet_management` and `motion_control` you can find the code for the praptical part of this course. In the folder `json_examples` you can find relevant json files, which are given as examples.

## Motion Control Simulation

### Run the Motion Control Simulation

## Fleet Management Simulation

### Run the Fleet Management Simulation

- Open the file `run_fleet_management_simulation.py` in the directory `.../imrl_workspace/fleet_managmenet` and run it by clicking the run button in the upper right corner of the window.
- Now the fleet managmenet simulation should open and the mobile robot should execute a transportation task. If the mobile robot does not move, try increasing the sleep time in code line 114 of the file `run_fleet_management_simulation.py`. Starting the `agent_simulation` executable may take longer.
- Detailed instructions on how to use the fleet management simulation can be found on Ilias in the file `Fleet Managmenet Simulation - Tutorial`.