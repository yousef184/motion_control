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
- Open the file `run_motion_control_simulation.py` in the directory `.../imrl_workspace/motion_control`.
- The default task is the task 1, you need to change the task number in the file `run_motion_control_simulation.py`.
- Run it by clicking the run button in the upper right corner of the window or execute it in a terminal.
- The motion control simulation should open in some seconds and an order will be published. The route is visualized as a green path in the simulation.
- Now you can control the via mqtt. A script `.../imrl_workspace/motion_control/src/motion_control/keyboard_control_commented.py` is offered to help you test the simulation.
- You can reset the task by clicking on the 'reset' in the simulation UI. The mobile robot will be reinitialized and the oder will be published again.

## Fleet Management Simulation

### Run the Fleet Management Simulation

- Open the file `run_fleet_management_simulation.py` in the directory `.../imrl_workspace/fleet_managmenet` and run it by clicking the run button in the upper right corner of the window.
- Now the fleet managmenet simulation should open and the mobile robot should execute a transportation task. If the mobile robot does not move, try increasing the sleep time in code line 114 of the file `run_fleet_management_simulation.py`. Starting the `agent_simulation` executable may take longer.
- Detailed instructions on how to use the fleet management simulation can be found on Ilias.