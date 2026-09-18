from typing import List, Tuple, Dict
from generators.generate_instance import Instance, Stop
from helper.json_loader import load_json
import csv
import random

big_M = 10000

class Request:
    def __init__(self, origin, destination, request_time, utility=0):
        self.origin = origin
        self.destination = destination
        self.utility = utility
        self.request_time = request_time

    def __str__(self):
        return f'request_time: {self.request_time}, utility: {self.utility}, origin: {self.origin}, destination: {self.destination}'

def get_instance(json_files: List[str], walking_distance: int, num_features: int) -> Instance:
    # Load all scenarios
    scenarios = [load_json(json_file) for json_file in json_files]

    # Add an id to each scenario
    for i, scenario in enumerate(scenarios):
        scenario['id'] = i
        scenario['probability'] = 1 / len(scenarios)

    return MultiScenario(scenarios, walking_distance, num_features)


class MultiScenario(Instance):
    def __init__(self, scenarios: List[Dict], walking_distance: int, num_features: int):


        self.scenarios = {scenario['id']: scenario for scenario in scenarios}

        # Get network, compulsory stops and time windows which are equal for all scenarios
        self.data = scenarios[0]
        self.network = self.build_network()
        self.compulsory_stops = self.get_compulsory_stops()
        self.time_windows = self.calculate_time_windows()
        self.avail_time_segments = self.calc_avail_time_segments()

        # Get requests, delta, and request features which differ for each scenario
        self.requests = {i: self.get_requests(scenario) for i, scenario in self.scenarios.items()}
        self.deltas = {i: self.calculate_delta(scenario, walking_distance) for i, scenario in self.scenarios.items()}
        self.unservable_requests = self.get_unservable_requests()
        # self.features = {i: self.calc_features(scenario, num_features) for i, scenario in self.scenarios.items()}
        # self.normalize_features()
        self.features = self.random_features(20)

    def get_requests(self, scenario: Dict) -> List[Request]:
        requests = []
        for i in range(int(len(scenario['sampled_building_pairs']) / 2)):
            origin, destination = self.get_origin_and_destination(scenario['sampled_building_pairs'], i)
            requests.append(Request(origin, destination, scenario['request_times'][origin['trip_id']]))
        return requests

    def calculate_delta(self, scenario: Dict, walking_distance: int) -> Dict:
        """
        Determine all stops within walking distance for each each request
        """
        delta = dict()

        for request in self.requests[scenario['id']]:
            delta[request] = dict()

            # Delta for origin building
            delta[request]['origin'] = list()
            building_num = request.origin['trip_id'] * 2

            for stop in self.network.nodes():
                if (self.scenarios[scenario['id']]['buildings_distance_matrix'][building_num][stop.order] <
                        walking_distance):
                    delta[request]['origin'].append(stop)

            # Delta for destination buidling
            delta[request]['destination'] = list()
            building_num = request.origin['trip_id'] * 2 + 1

            for stop in self.network.nodes():
                if (self.scenarios[scenario['id']]['buildings_distance_matrix'][building_num][stop.order] <
                        walking_distance):
                    delta[request]['destination'].append(stop)

        return delta

    def calc_features(self, scenario, num_features):
        features = {}
        for request in self.requests[scenario['id']]:
            features[request] = {'bais': 1,
                                 'time include origin': self.attr_include(scenario['id'], request, 'origin', 'travel_time'),
                                 'time include destination': self.attr_include(scenario['id'], request, 'destination', 'travel_time'),
                                 'cost include origin': self.attr_include(scenario['id'], request, 'origin', 'weight'),
                                 'cost include destination': self.attr_include(scenario['id'], request, 'destination', 'weight'),
                                 'available time origin': self.avail_time(scenario['id'], request, 'origin'),
                                 'available time destination': self.avail_time(scenario['id'], request, 'destination'),
                                 'origin stops': self.get_origin_stops(self.deltas[scenario['id']], request),
                                 'destination stops': self.get_destination_stops(self.deltas[scenario['id']], request)}

        feature_names = ['bais', 'time include origin', 'time include destination', 'cost include origin',
                         'cost include destination', 'available time origin', 'available time destination',
                         'origin stops', 'destination stops']

        features_pruned = {}
        for request in features:
            features_pruned[request] = {}
            for feature in feature_names[:num_features+1]:
                features_pruned[request][feature] = features[request][feature]
        return features_pruned

    def calc_avail_time_segments(self):
        time_segments = {}
        for stop1, stop2 in zip(self.time_windows.keys(), list(self.time_windows.keys())[1:]):
            time_segments[stop1.segment[-1]] = (self.time_windows[stop2]['b'] - self.time_windows[stop1]['a'] -
                                                self.network[stop1][stop2]['travel_time'])
        return time_segments

    def attr_include(self, scenario_id: int, request: Request, stop_type: str, attr: str):
        """
        Calculate minimum additional time/ cost it takes to include request into current tour (currently just compulsory stops)
        """

        shortest_include_t = big_M

        for stop in self.deltas[scenario_id][request][stop_type]:

            # A compulsory stop is always in the route, so return 0
            if stop.is_compulsory:
                return 0

            # Get compulsory stops of stop segment
            segment_start = [cp for cp in self.compulsory_stops if cp.segment[-1] == stop.segment[0]][0]
            segment_end = [cp for cp in self.compulsory_stops if cp.segment[0] == stop.segment[0]][-1]

            # Calculate additional time to include stop in route
            attr_with_req = self.network[segment_start][stop][attr] + self.network[stop][segment_end][attr]
            additional_attr = attr_with_req - self.network[segment_start][segment_end][attr]

            if attr == 'travel_time':
                # Make sure stop can theoretically be included in segment and update shortest_include_t
                # if additional_attr < self.avail_time_segments[stop.segment[0]]:
                shortest_include_t = min(shortest_include_t, additional_attr)

            elif attr == 'weight':
                shortest_include_t = min(shortest_include_t, additional_attr)

        return shortest_include_t


    def avail_time(self, scenario_id: int, request: Request, stop_type: str):
        available_time = 0
        for stop in self.deltas[scenario_id][request][stop_type]:

            # For a compulsory stop we do not care about the available time in a segment
            if stop.is_compulsory:
                return big_M

            # Get compulsory stops of stop segment
            segment_start = [cp for cp in self.compulsory_stops if cp.segment[-1] == stop.segment[0]][0]
            segment_end = [cp for cp in self.compulsory_stops if cp.segment[0] == stop.segment[0]][-1]

            # Calculate additional time to include stop in route
            available_time = max(available_time, self.network[segment_start][segment_end]['travel_time'])
        return available_time

    def set_utilities(self, utility):
        for scenario in self.scenarios:
            for request in self.requests[scenario]:
                request.utility = utility

    @staticmethod
    def get_origin_stops(delta, request):
        return len(delta[request]['origin'])

    @staticmethod
    def get_destination_stops(delta, request):
        return len(delta[request]['destination'])

    def normalize_features(self):
        features = list([feature.keys() for feature in self.features[0].values()][0])
        max_values = {feature: 0 for feature in features}

        # Find the maximum value for each feature
        for scenario in self.features:
            for request in self.features[scenario]:
                for feature in features:
                    max_values[feature] = max(max_values[feature], self.features[scenario][request][feature])

        # Divide by maximum value of each feature and account for rounding errors
        for scenario in self.features:
            for request in self.features[scenario]:
                for feature in features:
                    # Account for rounding errors
                    if self.features[scenario][request][feature] < 0:
                        self.features[scenario][request][feature] = 0
                    elif max_values[feature] > 0:
                        self.features[scenario][request][feature] /= max_values[feature]
        return

    def get_avg_requests(self):
        return (sum([len(req_scen) for req_scen in self.requests.values()]) +
                sum([len(req_scen) for req_scen in self.unservable_requests.values()])) / len(self.requests)

    def get_csv_features(self):
        rows = [['bais', 'time include origin', 'time include destination', 'cost include origin',
                 'cost include destination', 'available time origin', 'available time destination',
                 'origin stops', 'destination stops']]

        for scenario in self.features:
            for req in self.features[scenario]:
                new_row = []
                for feature in self.features[scenario][req].values():
                    new_row.append(feature)
                rows.append(new_row)

        with open(f'results/features.csv',
                  'w') as f:
            writer = csv.writer(f)
            writer.writerows(rows)

    def get_unservable_requests(self):
        # Find all unservable requests
        unservable_requests = {}
        for scenario in self.requests:
            unservable_requests[scenario] = set()
            for req in self.requests[scenario]:
                if len(self.deltas[scenario][req]['origin']) == 0 or len(self.deltas[scenario][req]['destination']) == 0:
                    unservable_requests[scenario].add(req)

        # Remove all unservable requests from set of requests
        for scenario in unservable_requests:
            for req in unservable_requests[scenario]:
                self.requests[scenario].remove(req)

        return unservable_requests


    def random_features(self, n):

        features = {}
        for i, scenario in self.scenarios.items():
            features[i] = {}
            for request in self.requests[scenario['id']]:
                features[i][request] = {key: random.random() for key in range(n)}
        return features