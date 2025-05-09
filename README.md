# IMRL Workspace

The structure of the IMRL workspace is as follows:

imrl_workspace/ <br>
├── fleet_management/ <br>
├── json_examples/ <br>
├── motion_control/ <br>
├── environment.yml <br>
└── README.md <br>

The practical part of this course is organized into two main folders: `fleet_management` and `motion_control`. Each of these contain a `data` folder and a `scr` folder. The `data` folders contains the data required to run the simulations and the `src` folders contain the source code of the simulation. Additionally, the `json_examples` folder provides examples of all JSON files you are going to work with.

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

## Getting Started (not relevant if you are using an IFL USB stick or laptop - everything is already set up here)

Make sure that Ubuntu 20 is running on your computer.

### IMRL Workspace

1. Download the `imrl_workspace` from [bwSync&Share](https://bwsyncandshare.kit.edu/s/8TnLtjMkWFNkLMX).
2. Unzip the `imrl_workspace` at the desired location on your computer.

### Visual Studio Code

1. Install VS Code:
    - If not already installed, install [VS Code](https://code.visualstudio.com/Download) based on the instructions for Ubuntu as your integrated development environment (IDE).

2. Install the Python extension in VS Code:
    - Open VS Code.
    - Go to the `Extensions` view by clicking on the icon with the squares in the activity bar on the left-hand side of the screen.
    - Search for `Python` in the Extensions view search bar.
    - Click on the Install button for the Python extension by Microsoft.
    - Here is a short tutorial on how to work with [Python in VS Code](https://code.visualstudio.com/docs/languages/python).

3. Open the IMRL workspace:
    - Open the `imrl_workspace` in VS Code by clicking on `File` in the menu bar at the top left of the screen, then select `Open Folder...` and navigate to the unzipped folder `imrl_workspace`.

### Conda - Virtual Environment

1. Install Anaconda:
    - If not already installed, install [Anaconda](https://www.anaconda.com/download/success) based on the instructions for Ubuntu as your conda package and environment manager.
    - Here you can find a [conda cheatsheet](https://docs.conda.io/projects/conda/en/latest/_downloads/843d9e0198f2a193a3484886fa28163c/conda-cheatsheet.pdf) with all important conda commands.

2. Create a virtual environment:
    - If not already opened, open VS Code and the `imrl_workspace` folder.
    - Open a new terminal in VS Code by clicking on `Terminal` in the menu bar at the top left of the screen and select `New Terminal`.
    - Verify the successful installation of Anaconda:
        ```
        conda --version
        ```

    - The output should be the version of Anaconda installed on your system. If errors occure, restart VS Code or your computer and try again.
    - Make sure that you are in the root directory of the `imrl_workspace` folder. The shown directory should end with `...\imrl_workspace`.
    - Create a new virtual environment named `imrl_env` using the `environment.yml` file with the following command:
        ```
        conda env create -n imrl_env -f environment.yml
        ```

    - Activate the virtual environment with the following command:
        ```
        conda activate imrl_env
        ```

    - Verify the installation by checking the installed packages:
        ```
        conda list
        ```

### Mosquitto - MQTT Broker

1. Install the MQTT broker Mosquitto:
    - If not already installed, install [Mosquitto](https://mosquitto.org/download/) based on the instructions for Ubuntu as your MQTT broker.

2. Run the Mosquitto broker:
    - Open a terminal. 
    - Navigate to the folder where mosquitto ist installed.
    - Run the Mosquitto broker with the following command:
        ```
        mosquitto -v
        ```

3. Test the Mosquitto Broker:
    - Use mosquitto_pub (publish) and mosquitto_sub (subscribe) commands to test the broker.
    - Subscribe to the topic `test/topic` with:
        ```
        mosquitto_sub -h localhost -t test/topic
        ```

    - In another terminal, publish a test message to the topic `test/topic` with:
        ```
        mosquitto_pub -h localhost -t test/topic -m "Hello, World!"
        ```
    
    - If successful, you should see the message "Hello, World!" appear in the subscriber terminal.

## Next Steps

Now your environment is set up correctly and you can run the [Motion Control Simulation](#motion-control-simulation) and the [Fleet Management Simulation](#fleet-management-simulation).