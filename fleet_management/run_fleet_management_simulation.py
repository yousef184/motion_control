# Import the necessary libraries and modules.
import os
import sys
import json
import time
import logging
import subprocess
from typing import Dict

# Get the current working directory.
current_dir = os.getcwd()

# Define the folder names.
imrl_workspace_folder = 'imrl_workspace'
fleet_management_folder = 'fleet_management'
src_folder = 'src'

# Check the current working directory and change it if necessary.
if imrl_workspace_folder in current_dir.split('\\')[-1]:
    # Change the current working directory to the 'fleet-management-simulation' directory.
    os.chdir(os.path.join(current_dir, fleet_management_folder))
elif fleet_management_folder in current_dir.split('\\')[-1]:
    # Do nothing if the current working directory is already 'fleet-management-simulation'.
    pass
elif src_folder in current_dir.split('\\')[-1]:
    # Change the current working directory to the 'fleet-management-simulation' directory.
    os.chdir(os.pardir)
else:
    raise Exception("Current working directory is neither 'imrl_workspace', 'fleet_management', nor 'src'.")

# Add the path to the src directory to the system path.
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

# Import the necessary classes.
from src.fleet_management.graph import Graph
from src.fleet_management.agents import Agents
from src.fleet_management.task_management import TaskManagement
from src.fleet_management.task_assignment import TaskAssignment
from src.fleet_management.fleet_management import FleetManagement

def setup_logging(log_file_path: str) -> logging.Logger:
    """
    Configure the logging setup.

    :param log_file_path: The path to the logging file.
    :return: The logging object.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        filename=log_file_path,
        filemode='a'
    )
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(formatter)
    logging.getLogger('').addHandler(console_handler)
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    return logging.getLogger(__name__)

class ConfigManager:
    """
    Class handles loading and managing configurations.
    """
    def __init__(self, config_paths: Dict[str, str]):
        self.config_data = self._load_json(config_paths['config'])
        self.lif_data = self._load_json(config_paths['lif'])
        self.agents_initialization_data = self._load_json(config_paths['agents_initialization'])
        self.transportation_tasks_data = self._load_json(config_paths['transportation_tasks'])

    @staticmethod
    def _load_json(path: str) -> dict:
        """
        Loads a JSON file from the given path.
        
        :param path: The path to the JSON file.
        :return: The JSON data.
        """
        try:
            with open(path, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            logging.error(f"File not found: {path}")
            sys.exit(1)
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON format in file: {path}")
            sys.exit(1)

def start_threads(task_management: TaskManagement, task_assignment: TaskAssignment, fleet_management: FleetManagement):
    """
    Start the threads for the task management, task assignment, and fleet management.

    :param task_management: The task management object.
    :param task_assignment: The task assignment object.
    :param fleet_management: The fleet management object.
    """
    # TODO: Start the threads here if you are using threads in your implementation.
    task_management.check_completion_thread.start()

def run_simulation(config_manager: ConfigManager, logging: logging.Logger):
    """
    Run the fleet management simulation.
    
    :param config_manager: The configuration manager object.
    :param logger: The logging object.
    """
    # Start the agent simulation.
    agent_simulation_path = "src/mobile_robot_simulation/dist/agent_simulation"
    try:
        subprocess.Popen([agent_simulation_path])
        # Wait for the agent simulation to start.
        # TODO: You may need to adjust the sleep time based on the time it takes for the agent simulation to start on your system.
        time.sleep(5)
    except subprocess.CalledProcessError as e:
        logging.error(f"Failed to execute {agent_simulation_path}: {e}")
        sys.exit(1)
    
    # Start the simulation timer.
    simulation_start_time = time.time()

    # Initialize the objects of the fleet manager.
    # TODO: You can change which arguments are passed to the class constructors based on your implementation.
    graph = Graph(lif_data=config_manager.lif_data)
    agents = Agents(config_data=config_manager.config_data, graph=graph,
                           agents_initialization_data=config_manager.agents_initialization_data, logging=logging,
                           simulation_start_time=simulation_start_time)
    task_management = TaskManagement(graph=graph, agents=agents,
                                     transportation_tasks_data=config_manager.transportation_tasks_data,
                                     simulation_start_time=simulation_start_time)
    task_assignment = TaskAssignment(graph=graph, agents=agents, task_management=task_management,
                                     simulation_start_time=simulation_start_time)
    fleet_management = FleetManagement(config_data=config_manager.config_data, graph=graph, agents=agents,
                                       task_management=task_management, simulation_start_time=simulation_start_time)
    
    # Start the threads.
    start_threads(task_management, task_assignment, fleet_management)

    # Run the simulation for the specified time.
    time.sleep(config_manager.config_data["simulation_run_time"])

def main():
    """
    Main function to set up and start the simulation.
    """
    # TODO: You understand how the code works and know where in the code what can be changed.

    # Logging setup.
    logging = setup_logging("data/output_files/logging_file.log")

    # Configuration paths.
    # The content of the input files can be changed but the file names should remain the same.
    config_paths = {
        "config": "data/input_files/config_file.json",
        "lif": "data/input_files/lif_file.json",
        "agents_initialization": "data/input_files/agentsInitialization_file.json",
        "transportation_tasks": "data/input_files/transportationTasks_file.json",
    }

    # Load configurations.
    config_manager = ConfigManager(config_paths)

    # Run the simulation.
    run_simulation(config_manager, logging)

if __name__ == "__main__":
    main()
