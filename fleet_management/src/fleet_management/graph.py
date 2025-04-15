# Import necessary libraries and modules.


class Graph:
    """
    Class representing the graph, based on which the agents are controlled by the fleet manager.
    """
    # TODO: Model the layout as a graph based on the data from the LIF file.

    def __init__(self, lif_data):
        """
        Initialize the graph based on the data from the LIF file.

        :param lif_data: Data from the LIF file.
        """
        self.nodes = self.get_nodes(lif_data)
        self.edges = self.get_edges(lif_data)
        self.stations = self.get_stations(lif_data)
        self.dwelling_nodes = self.get_dwelling_nodes(lif_data)

    def get_nodes(self, lif_data):
        """
        Get the nodes from the LIF data.

        :param lif_data: Data from the LIF file.
        :return: The nodes in the graph.
        """
        # TODO: Implement the method to get the nodes from the LIF data.
        pass

    def get_edges(self, lif_data):
        """
        Get the edges from the LIF data.

        :param lif_data: Data from the LIF file.
        :return: The edges in the graph.
        """
        # TODO: Implement the method to get the edges from the LIF data.
        pass

    def get_stations(self, lif_data):
        """
        Get the stations from the LIF data.
        At station nodes the agents can interact with the environment.

        :param lif_data: Data from the LIF file.
        :return: The stations in the graph.
        """
        # TODO: Implement the method to get the stations from the LIF data.
        pass

    def get_dwelling_nodes(self, lif_data) -> list:
        """
        Get the dwelling nodes.
        Dwelling nodes are nodes, where the agents can park and wait for the next task.
        Number of dwelling nodes must be equal or greater than the number of agents.

        :param lif_data: The data from the LIF file.
        :return: A list of node IDs of the dwelling nodes.
        """
        # TODO: Implement the method to get the dwelling nodes from the LIF data.
        pass
