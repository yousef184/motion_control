import time
import threading


class TaskManagement:
    """
    Class for managing the transportation tasks of the agents.
    """
    def __init__(self, graph, agents, transportation_tasks_data, simulation_start_time) -> None:
        """
        Initialize the task management object.

        :param graph: The graph object.
        :param agents: The agents object containing the digital twin agents.
        :param transportation_tasks_data: The transportation orders data.
        :param simulation_start_time: The start time of the simulation.
        """
        self.simulation_start_time = simulation_start_time
        self.graph = graph
        self.agents = agents
        self.transportation_tasks_data = transportation_tasks_data
        self.task_list = self.get_task_list()
        self.check_completion_thread = threading.Thread(target=self.check_tasks_completion)

    def get_task_list(self) -> list:
        """
        Get the list of tasks from the transportation taskss data.

        :return: A list containing the tasks of the agents to complete as dictionaries.
        """
        task_list = []
        for task in self.transportation_tasks_data['transportationTasks']:
            task_dict = {'task_id': task['transportationTaskId'],
                         'startStationId': task['startStationId'],
                         'goalStationId': task['goalStationId'],
                         'task_assigned': False,
                         'task_completed': False}
            task_list.append(task_dict)
        return task_list

    def check_tasks_completion(self) -> None:
        """
        Check if all the tasks are completed.
        The function runs in a separate thread.
        """
        running = True
        while running:
            task_completed = True

            # Check if all tasks are completed.
            for task in self.task_list:
                if not task['task_completed']:
                    task_completed = False
                    break

            # Check if all agents are state IDLE.
            for agent in self.agents.agents:
                if agent.agent_state != "IDLE":
                    task_completed = False
                    break

            if task_completed:
                self.agents.logging.info(f"All tasks are completed at time {round(time.time() - self.simulation_start_time, 2)}.")
                running = False

            time.sleep(1)
