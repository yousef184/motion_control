import math
import uuid
import time
import threading


class FleetManagement:
    """
    Manages the fleet: computes paths and sends VDA 5050 orders to agents.

    Task 7: Integrate A* into the full order pipeline (multi-stop tasks).
        Implement the helper methods build_path_for_task(), build_order_nodes(),
        and build_order_edges(), then call them from fleet_manager().

    Task 10: Automate all transportation tasks.
        Run fleet_manager() in a loop inside a daemon thread so tasks are
        picked up and executed one after another until all are done.
    """

    def __init__(self, config_data, graph, agents, task_management,
                 simulation_start_time) -> None:
        self.simulation_start_time = simulation_start_time
        self.config_data = config_data
        self.graph = graph
        self.agents = agents
        self.task_management = task_management
        self.path_planning = PathPlanning(config_data=self.config_data,
                                         graph=self.graph)
        self.fleet_manager()

    def fleet_manager(self) -> None:
        """
        Send a movement order to the first agent.

        Every transportation task follows this movement structure:
            dwelling (start) -> pick station -> process station(s) -> drop station -> dwelling

        Task 3 — manually fill the 'nodes' and 'edges' lists for the first
        transportation task (T1) and pass them to generate_order_message().

        Fill each entry following these rules:
            Node dict:  {"nodeId": str, "x": float, "y": float,
                         "theta": float | None, "actions": list}
            Edge dict:  {"edgeId": str, "startNodeId": str, "endNodeId": str,
                         "actions": list}

        Action rules (see orderMessage_Example.json for a reference):
          - Node immediately BEFORE a TRANSFER station (pick/drop):
                {"actionType": "init_fine_positioning",
                 "actionId": "<unique-uuid-string>", "blockingType": "HARD"}
          - TRANSFER station node (actionType pick or drop):
                {"actionType": "pick",  "actionId": "...", "blockingType": "HARD"}
             or {"actionType": "drop",  "actionId": "...", "blockingType": "HARD"}
          - PROCESS station node:
                {"actionType": "process", "actionId": "...", "blockingType": "HARD",
                 "processingTime": <float from task>}

        Node positions and theta values can be read from lif_file.json or
        self.graph.nodes[node_id] (if Task 4 is implemented).
        Use str(uuid.uuid4()) to generate unique actionId strings.

        Task 7 — replace the manual lists with A*:
            1. Pick the first unassigned task from self.task_management.task_list.
            2. Call self.build_path_for_task(task, agent.current_node).
            3. Call self.build_order_nodes(path_nodes, task) and
               self.build_order_edges(path_nodes, path_edges).
            4. Pass the results to generate_order_message() below.
            5. Mark task['task_assigned'] = True and agent.agent_state = 'EXECUTING'.

        Task 10 — automate over all tasks in a thread:
            Wrap the logic above in a while loop and run in a daemon thread:
                import threading
                threading.Thread(target=self.fleet_manager, daemon=True).start()
        """
        # Task 7 — A* pipeline (replaces Task 3 manual nodes/edges)
        task = next((t for t in self.task_management.task_list if not t['task_assigned']), None)
        if task is None:
            return

        agent = self.agents.agents[0]
        path_nodes, path_edges = self.build_path_for_task(task, agent.current_node)
        if path_nodes is None:
            return
        nodes = self.build_order_nodes(path_nodes, task)
        edges = self.build_order_edges(path_nodes, path_edges)

        task['task_assigned'] = True
        agent.agent_state = 'EXECUTING'
        agent.current_task = task

        agent.order_interface.generate_order_message(
            agent=agent,
            orderId=str(self.agents.order_header_id),
            order_updateId=0,
            nodes=nodes,
            edges=edges)

    def build_path_for_task(self, task: dict, start_node: str) -> tuple:
        """
        Task 7: Chain multiple A* searches to cover all stations in a task.

        Every task follows: dwelling -> pick -> process(es) -> drop -> dwelling

        A task's stations list covers only the waypoints (not the dwelling legs):
            [{"nodeId": "N4", "actionType": "pick"},
             {"nodeId": "N3", "actionType": "process", "processingTime": 1.0},
             {"nodeId": "N2", "actionType": "drop"}]

        Build the full path by planning one A* leg per station plus a return leg:
            Leg 0: start_node  -> stations[0]['nodeId']   (travel to pick)
            Leg 1: stations[0] -> stations[1]['nodeId']   (pick to process)
            Leg 2: stations[1] -> stations[2]['nodeId']   (process to drop)
            Return: last station -> nearest dwelling node

        After each astar_search() call, extend the combined lists by:
            - path_nodes: skip the first node of each new leg (it duplicates
              the last node of the previous leg).
            - path_edges: concatenate as-is.

        For the return leg, choose the nearest dwelling node from
        self.graph.dwelling_nodes using self.graph.nodes[n]['pos'].

        Returns (path_nodes, path_edges).
        """
        # TODO Task 7: Implement this method.
        path_nodes = []
        path_edges = []
        current = start_node

        last_station = task['stations'][-1]['nodeId']
        pos_last = self.graph.nodes[last_station]['pos']
        nearest_dwelling = min(
            self.graph.dwelling_nodes,
            key=lambda d: math.dist(pos_last, self.graph.nodes[d]['pos']),
        )
        goals = [s['nodeId'] for s in task['stations']] + [nearest_dwelling]

        for goal in goals:
            leg_nodes, leg_edges = self.path_planning.astar_search(current, goal)
            if leg_nodes is None:
                return (None, None)
            if path_nodes:
                leg_nodes = leg_nodes[1:]
            path_nodes.extend(leg_nodes)
            path_edges.extend(leg_edges)
            current = goal

        return (path_nodes, path_edges)

    def build_order_nodes(self, path_nodes: list, task: dict) -> list:
        """
        Task 7: Assign VDA 5050 actions to each node in the combined path.

        Action rules (matching the dwelling -> pick -> process -> drop -> dwelling structure):
            - For stations with actionType 'pick' or 'drop' (TRANSFER):
                * The node immediately BEFORE the station in path_nodes
                  gets an 'init_fine_positioning' action.
                * The station node itself gets a 'pick' or 'drop' action.
            - For stations with actionType 'process' (PROCESS):
                * The station node gets a 'process' action with
                  processingTime from station['processingTime'].
            - All other nodes (intermediate + dwelling): empty actions list.

        Use uuid.uuid4() to generate unique actionId strings.

        For each node, also look up:
            - x, y from self.graph.nodes[n]['pos']
            - theta from self.graph.nodes[n].get('vehicleTypeNodeProperties', [])
              for the entry matching vehicleTypeId 'Longitudinal_Conveyor'.
              If the value is None or the string "None", use theta=None.

        Returns a list of node dicts:
            [{"nodeId": n, "x": x, "y": y, "theta": theta_or_None, "actions": [...]}, ...]

        Hint: build a lookup dict {station_nodeId: station} first, then iterate
        through path_nodes and check each node against the lookup.
        To detect the 'before TRANSFER' node, look ahead:
            if i + 1 < len(path_nodes) and path_nodes[i+1] is a TRANSFER station:
                add init_fine_positioning to path_nodes[i]
        """
        # TODO Task 7: Implement this method.
        station_by_node = {s['nodeId']: s for s in task['stations']}
        order_nodes = []
        for i, node_id in enumerate(path_nodes):
            x, y = self.graph.nodes[node_id]['pos']
            theta = None
            for prop in self.graph.nodes[node_id].get('vehicleTypeNodeProperties', []):
                if prop.get('vehicleTypeId') == 'Longitudinal_Conveyor':
                    t = prop.get('theta')
                    if t is not None and t != 'None':
                        theta = t
                    break

            actions = []
            if i + 1 < len(path_nodes):
                next_id = path_nodes[i + 1]
                if next_id in station_by_node:
                    if station_by_node[next_id]['actionType'] in ('pick', 'drop'):
                        actions.append({
                            'actionType': 'init_fine_positioning',
                            'actionId': str(uuid.uuid4()),
                            'blockingType': 'HARD',
                        })
            if node_id in station_by_node:
                station = station_by_node[node_id]
                action_type = station['actionType']
                if action_type == 'process':
                    actions.append({
                        'actionType': 'process',
                        'actionId': str(uuid.uuid4()),
                        'blockingType': 'HARD',
                        'processingTime': station['processingTime'],
                    })
                else:
                    actions.append({
                        'actionType': action_type,
                        'actionId': str(uuid.uuid4()),
                        'blockingType': 'HARD',
                    })

            order_nodes.append({
                'nodeId': node_id,
                'x': x,
                'y': y,
                'theta': theta,
                'actions': actions,
            })
        return order_nodes

    def build_order_edges(self, path_nodes: list, path_edges: list) -> list:
        """
        Task 7: Build the edges input list for generate_order_message().

        Returns:
            [{
              "edgeId": path_edges[i],
              "startNodeId": path_nodes[i],
              "endNodeId": path_nodes[i+1],
              "actions": []}]
        """
        # TODO Task 7: Implement this method.
        return [
            {
                'edgeId': path_edges[i],
                'startNodeId': path_nodes[i],
                'endNodeId': path_nodes[i + 1],
                'actions': [],
            }
            for i in range(len(path_edges))
        ]


class PathPlanning:
    """
    A* shortest-path search over the graph.

    Task 6: Implement all three methods.
    Graph interfaces you will likely need:
        self.graph.get_connected_nodes(node_id)  -> list of neighbour node IDs
        self.graph.get_connected_edge(a, b)       -> edge ID connecting a and b
        self.graph.nodes[node_id]['pos']           -> (x, y) position tuple
    """

    def __init__(self, config_data, graph) -> None:
        self.config_data = config_data
        self.graph = graph

    def astar_search(self, start_node: str, goal_node: str) -> tuple:
        """
        Find the shortest path from start_node to goal_node using A*.

        Returns:
            (path_nodes, path_edges)
            path_nodes : ordered list of node IDs,  e.g. ["N5", "N1", "N7", "N2"]
            path_edges : ordered list of edge IDs,  e.g. ["E21", "E1", "E2"]
            len(path_edges) == len(path_nodes) - 1

        Returns (None, None) if no path exists.

        Task 6 hints:
            - Handle the edge case start_node == goal_node first:
                  return ([start_node], [])
            - Use a min-heap (heapq) ordered by f = g + h:
                  import heapq
                  open_set = []
                  heapq.heappush(open_set, (f, node_id))
            - g(n) = accumulated travel cost from start to n.
                  Increment it with self.get_distance(current, neighbour).
            - h(n) = self.get_h(n, goal_node)  -- Euclidean, never overestimates.
            - Keep a came_from dict to reconstruct the path on success.
            - Build path_edges by calling self.graph.get_connected_edge(a, b)
              for each consecutive pair in the reconstructed path_nodes.
        """
        # TODO Task 6: Implement A* here.
        if start_node == goal_node:
            return ([start_node], [])

        import heapq
        open_heap = [(self.get_h(start_node, goal_node), start_node)]
        g_score = {start_node: 0.0}
        came_from = {}

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current == goal_node:
                path_nodes = [goal_node]
                node = goal_node
                while node in came_from:
                    node = came_from[node]
                    path_nodes.append(node)
                path_nodes.reverse()
                path_edges = [
                    self.graph.get_connected_edge(path_nodes[i], path_nodes[i + 1])
                    for i in range(len(path_nodes) - 1)
                ]
                return (path_nodes, path_edges)

            current_g = g_score[current]
            for neighbor in self.graph.get_connected_nodes(current):
                tentative_g = current_g + self.get_distance(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + self.get_h(neighbor, goal_node)
                    heapq.heappush(open_heap, (f, neighbor))

        return (None, None)

    def get_h(self, current_node: str, goal_node: str) -> float:
        """
        Heuristic -- Euclidean distance from current_node to goal_node.

        Task 6: Use self.graph.nodes[node_id]['pos'] for (x, y) coordinates,
        then return math.dist(pos_current, pos_goal).
        This heuristic is admissible (straight-line <= actual path length).
        """
        # TODO Task 6: Implement this method.
        pos_current = self.graph.nodes[current_node]["pos"]
        pos_goal = self.graph.nodes[goal_node]["pos"]
        return math.dist(pos_current, pos_goal)

    def get_distance(self, start_node: str, goal_node: str) -> float:
        """
        Actual edge cost -- Euclidean distance between two adjacent nodes.

        Task 6: Same formula as get_h(); used as the g-score increment per step.
        """
        # TODO Task 6: Implement this method.
        return self.get_h(start_node, goal_node)
   ### pytest fleet_management/tests/test_task6_astar.py -v   来验证
