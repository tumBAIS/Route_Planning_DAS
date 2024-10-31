from typing import List, Tuple, Dict

from helper.json_loader import load_json
import networkx as nx
from datetime import datetime, timedelta
import math


# import helper.visualize_instance as visualize

def load_data(instance_name) -> Dict:
    "Loads data"
    data = load_json(instance_name)

    return data


class Stop:
    def __init__(self, latitude: float, longitude: float,
                 is_compulsory: bool, order: int, segment: List, tw: Tuple, id: int,
                 name: str):
        self.latitude = latitude
        self.longitude = longitude
        self.is_compulsory = is_compulsory
        self.order = order
        self.segment = segment
        self.id = id
        self.tw = self.set_tw(tw)
        self.name = name

    def __hash__(self):
        return self.id

    def __eq__(self, other):
        return self.id == other.id

    def __str__(self):
        return f'id: {self.id} compulsory: {self.is_compulsory}, order: {self.order}, segment: {self.segment}, tw: {self.tw} '

    @staticmethod
    def set_tw(tw):
        if tw is not None:
            return (datetime.strptime(tw[0], '%H:%M:%S'), datetime.strptime(tw[1], '%H:%M:%S'))

        return None


class Request:
    def __init__(self, origin, destination, utility=0):
        self.origin = origin
        self.destination = destination
        self.utility = utility




class Instance:
    def __init__(self, data: Dict, walking_distance: int):
        self.data = data
        self.requests = self.get_requests()
        self.network = self.build_network()
        self.compulsory_stops = self.get_compulsory_stops()
        self.delta = self.calculate_delta(walking_distance)
        self.time_windows = self.calculate_time_windows()

    @staticmethod
    def get_origin_and_destination(requests: Dict, n: int) -> Tuple[Dict, Dict]:
        for item in requests:
            if item['location'] == 'origin' and item['trip_id'] == n:
                origin = item
                break

        for item in requests:
            if item['location'] == 'destination' and item['trip_id'] == n:
                destination = item
                break

        return origin, destination

    def get_requests(self) -> List[Request]:
        requests = []
        for i in range(int(len(self.data['sampled_building_pairs']) / 2)):
            origin, destination = self.get_origin_and_destination(self.data['sampled_building_pairs'], i)
            requests.append(Request(origin, destination))
        return requests

    def get_stops(self) -> Tuple[List, List]:

        time_windows = {
            tw['node_id']: (tw['time_window']['earliest_departure_time'], tw['time_window']['latest_departure_time'])
            for tw in self.data['time_windows']}

        compulsory_names = {tw['node_id']: tw['stop_name'] for tw in self.data['time_windows']}

        # Get ordered list of compulsory stops
        compulsory_stops_data = sorted([node for node in self.data['route_nodes']
                                        if node['node_id'] in time_windows.keys()],
                                       key=lambda d: d['order'])

        # Get ordered list of optional stops
        optional_stops_data = sorted([node for node in self.data['route_nodes']
                                      if node['node_id'] not in time_windows.keys()],
                                     key=lambda d: d['order'])

        # Value of all order values of the compulsory stops
        segment_ticks = [node['order'] for node in compulsory_stops_data]

        # Determine segments of compulsory stops
        for i in range(len(compulsory_stops_data)):

            if i == 0:
                compulsory_stops_data[i]['segment'] = [1]

            elif i < len(compulsory_stops_data) - 1:
                compulsory_stops_data[i]['segment'] = [i, i + 1]

            else:
                compulsory_stops_data[i]['segment'] = [i]

        compulsory_stops = [
            Stop(node['latitude'], node['longitude'], True,
                 node['order'], node['segment'], time_windows[node['node_id']], node['node_id'],
                 compulsory_names[node['node_id']])
            for node in compulsory_stops_data]

        # Determine segments of optional stops
        for node in optional_stops_data:
            for i in range(len(segment_ticks) - 1):

                # Should not happen after order is fixed
                if node['order'] < segment_ticks[0]:
                    node['segment'] = [1]

                # should not happen after order is fixed
                elif node['order'] > segment_ticks[-1]:
                    node['segment'] = [len(segment_ticks) - 1]

                if segment_ticks[i] < node['order'] < segment_ticks[i + 1]:
                    node['segment'] = [i + 1]
                    break

        optional_stops = [
            Stop(node['latitude'], node['longitude'], node['node_id'] in compulsory_stops_data,
                 node['order'], node['segment'], None, node['node_id'], None)
            for node in optional_stops_data]

        return compulsory_stops, optional_stops

    def build_graph(self, compulsory_stops: List[Stop], optional_stops: List[Stop]) -> nx.DiGraph:

        # Generate an empty networkx graph for each segment of the DAS
        segment_graphs = [nx.DiGraph() for _ in range(compulsory_stops[-1].segment[0])]

        for n in range(len(segment_graphs)):

            graph = segment_graphs[n]

            # Add all relevant nodes to it
            relevant_compulsory_stops = [stop for stop in compulsory_stops if n + 1 in stop.segment]

            # Add the two compulsory stops to the segment graph
            for node in relevant_compulsory_stops:
                graph.add_node(node, pos=(node.latitude, node.longitude),
                               segment=node.segment, is_compulsory=node.is_compulsory, tw=node.tw, node_id=node.id)
            # graph.add_nodes_from([node for node in compulsory_stops if i + 1 in node.segment])

            # Add all optional stops to the segment graph
            for node in [node for node in optional_stops if n + 1 in node.segment]:
                graph.add_node(node, pos=(node.latitude, node.longitude),
                               segment=node.segment, is_compulsory=node.is_compulsory, tw=node.tw)

            # Add all relevant arcs to it
            for i in graph.nodes():
                for j in graph.nodes():
                    if i != j and j != relevant_compulsory_stops[0] and i != relevant_compulsory_stops[1]:
                        graph.add_edge(i, j, weight=self.data['distance_matrix'][i.order][j.order],
                                       segment=[j.segment[0]],
                                       travel_time=self.data['travel_time_matrix'][i.order][j.order])

        # Join all segment graphs to one complete graph
        complete_graph = nx.compose_all(segment_graphs)

        return complete_graph

    def build_network(self) -> nx.DiGraph:
        compulsory_stops, optional_stops = self.get_stops()
        assert len(compulsory_stops) == len(self.data['time_windows']), f"Likely two times the same node in route nodes"
        graph = self.build_graph(compulsory_stops, optional_stops)
        return graph

    def get_compulsory_stops(self):
        return [stop for stop in self.network.nodes() if stop.is_compulsory]

    def calculate_delta(self, walking_distance: int) -> Dict:
        """
        Determine all stops within walking distance for each each request
        """
        delta = dict()

        for request in self.requests:
            delta[request] = dict()

            # Delta for origin building
            delta[request]['origin'] = list()
            building_num = request.origin['trip_id'] * 2

            for stop in self.network.nodes():
                if self.data['buildings_distance_matrix'][building_num][stop.order] < walking_distance:
                    delta[request]['origin'].append(stop)

            # Delta for destination buidling
            delta[request]['destination'] = list()
            building_num = request.origin['trip_id'] * 2 + 1

            for stop in self.network.nodes():
                if self.data['buildings_distance_matrix'][building_num][stop.order] < walking_distance:
                    delta[request]['destination'].append(stop)

        return delta

    def calculate_time_windows(self) -> Dict:
        """
        Convert datetime timewindows into seconds
        """
        time_windows = dict()

        for stop in self.compulsory_stops:
            time_windows[stop] = dict()
            time_only_a = stop.tw[0].time()
            time_only_b = stop.tw[1].time()
            time_windows[stop]['a'] = timedelta(hours=time_only_a.hour,
                                                minutes=time_only_a.minute,
                                                seconds=time_only_a.second).total_seconds()

            time_windows[stop]['b'] = timedelta(hours=time_only_b.hour,
                                                minutes=time_only_b.minute,
                                                seconds=time_only_b.second).total_seconds()

        return time_windows

    def request_segments_unordered(self) -> List[Request]:
        """
        Return requests in which the segments do not have to be ordered
        """
        unordered_requests = list()
        for request in self.requests:
            if (max([stop.segment[0] for stop in self.delta[request]['origin']], default=-math.inf) >=
                    min([stop.segment[0] for stop in self.delta[request]['destination']], default=math.inf)):
                unordered_requests.append(self.delta[request])

        return unordered_requests

    def check_time_windows(self) -> List:
        """
        Return time windows which are not feasible even by the direct route between compulsory stops
        """
        inf_tw = list()
        for index in range(1, len(self.compulsory_stops)):
            max_time = (self.time_windows[self.compulsory_stops[index]]['b'] -
                        self.time_windows[self.compulsory_stops[index - 1]]['a'])

            min_time = self.network[self.compulsory_stops[index - 1]][self.compulsory_stops[index]]['travel_time']
            if max_time < min_time:
                inf_tw.append(index)
        return inf_tw

    def unservable_requests(self) -> List:
        """
        Return and remove requests which do not have an origin or a destination
        """
        unservable_requests = [req for req in self.requests
                               if len(self.delta[req]['origin']) == 0 or len(self.delta[req]['destination']) == 0]

        for req in unservable_requests:
            self.requests.remove(req)

        return unservable_requests


def get_instance(json_file: str, walking_distance: int) -> Instance:
    data = load_json(json_file)
    return Instance(data, walking_distance)


if __name__ == '__main__':
    get_instance('../data/55.json', 300)
    # visualize.draw_graph(instance.network)
    # visualize.draw_graph_with_streets(instance, 'Test')
