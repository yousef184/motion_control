# IMRL Workspace

The structure of the IMRL workspace is as follows:

imrl_workspace/ <br>
├── fleet_management/ <br>
├── json_examples/ <br>
├── motion_control/ <br>
└── README.md <br>

The practical part of this course is organized into two main folders: `fleet_management` and `motion_control`. Each of these contain a `data` folder and a `scr` folder. The `data` folders contains the data required to run the simulations and the `src` folders contain the source code of the simulation. Additionally, the `json_examples` folder provides examples of all JSON files you are goung to work with.

## Motion Control Simulation

### Run the Motion Control Simulation
- Open the file `run_motion_control_simulation.py` in the directory `.../imrl_workspace/motion_control`.
- The default task is the task 1, you need to change the task number in the file `run_motion_control_simulation.py`.
- Run it by clicking the run button in the upper right corner of the window or execute it in a terminal.
- The motion control simulation should open in some seconds and an order will be published. The route is visualized as a green path in the simulation.
- Now you can control the via mqtt. A script `.../imrl_workspace/motion_control/src/motion_control/keyboard_control_commented.py` is offered to help you test the simulation.
- You can reset the task by clicking on the 'reset' in the simulation UI. The mobile robot will be reinitialized and the oder will be published again.

## Fleet Management Simulation

### Run the Fleet Management Simulation
- Open the file `run_fleet_management_simulation.py` in the directory `.../imrl_workspace/fleet_managment`.
- Run it by clicking the run button in the upper right corner of the window or execute it in a terminal.
- The fleet managmenet simulation should open and an order message should be published by the fleet management after a few seconds. 
- After receiving the order message, the mobile robot should execute one transportation task.
- If the mobile robot does not move, try increasing the sleep time in code line 114 of the file `run_fleet_management_simulation.py`. Starting the `agent_simulation` executable may take longer and must be done before the order message is published.
- End the simulation by killing the terminal.
- Detailed instructions on how to use the fleet management simulation can be found on Ilias.