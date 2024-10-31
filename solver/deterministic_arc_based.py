import gurobipy as grp
from gurobipy import GRB
from datetime import datetime, timedelta


def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = grp.tuplelist(edge for edge in model._edges
                                 if vals[edge] > 0.5)
        # find the shortest cycle in the selected edge list
        subtours = subtour(selected, model._nodes)
        if len(subtours) > 1:
            for tour in subtours:
                # add subtour elimination constr. for every pair of cities in subtour
                model.cbLazy(grp.quicksum(model._vars[edge] for edge in tour)
                             <= len(tour) - 1)
                model._subtours.append(tour)


def subtour_arcs(used_arcs, origin, destination):

    origin_arc = next(arc for arc in used_arcs if arc[0] == origin)
    od_path_origin = [origin_arc]

    while current_arc[1] != destination:
        current_arc = next(arc for arc in used_arcs if arc[0] == current_arc[1])
        od_path_origin.append(current_arc)

        used_arcs.remove(current_arc)

    subtours = list()
    while len(used_arcs) > 0:
        current_arc = used_arcs[0]
        tour = [current_arc]
        used_arcs.remove(current_arc)

        while current_arc[1] != tour[0][-1]:
            current_arc = next(arc for arc in used_arcs if arc[0] == current_arc[1])
            tour.append(current_arc)

            used_arcs.remove(current_arc)

        subtours.append(tour)

    return subtours

def subtour(used_edges, nodes):
    subtours = list()

    while len(used_edges) > 0:
        tour_nodes = set([list(used_edges[0])[0], list(used_edges[0])[1]])
        tour_edges = [used_edges[0]]
        used_edges.remove(used_edges[0])

        while True:
            # next_edges = [edge for edge in used_edges if list(edge)[0] in tour_nodes or list(edge)[1] in tour_nodes]
            next_edges = [edge for edge in used_edges if list(edge)[0] in tour_nodes]
            for edge in next_edges:
                tour_nodes.update(edge)
                tour_edges.append(edge)
                used_edges.remove(edge)

            if len(next_edges) == 0:
                subtours.append(tour_edges)
                break

    return subtours


class DeterministicModel:
    def __init__(self, network, compulsory_stops, requests, utility, delta, time_windows):
        self.network = network
        self.compulsory_stops = compulsory_stops

        self.requests = requests
        self.utility = utility
        self.delta = delta  # {request: {'origin': {stop1: 0, stop2: 1}}, {'destination': {stop1: 1, stop2: 0}}}
        self.time_windows = time_windows

        # Model
        self.m = None

        # Variables
        self.x = None
        self.y = None
        self.t = None

        # Constraints
        self.stop_out = None
        self.stop_in = None
        self.flow_conservation = None
        self.flow_conservation_origin = None
        self.flow_conservation_destination = None
        self.time_compulsory = None
        self.time_window_a = None
        self.time_window_b = None
        self.out_flow = None

    def get_segment_edges(self, segment):
        return [(i, j) for (i, j) in self.network.edges() if segment in self.network[i][j]['segment']]

    def get_segment_nodes(self, segment):
        return [stop for stop in self.network.nodes() if segment in stop.segment]

    def build_model(self):
        self.m = grp.Model('Deterministic')

        # Variables
        self.x = {(i, j): self.m.addVar(vtype=GRB.BINARY, name=f'x_{(i, j)}', obj=-self.network[i][j]['weight'])
                  for (i, j) in self.network.edges()}
        self.y = {req: self.m.addVar(vtype=GRB.BINARY, obj=self.utility[req]) for req in self.requests}
        self.t = {stop: self.m.addVar(vtype=GRB.CONTINUOUS, name=f't_{stop}') for stop in self.compulsory_stops}

        # Constraints
        self.stop_out = {req: self.m.addConstr(self.y[req] <=
                                               grp.quicksum(self.x[arc]
                                                            for stop in self.delta[req]['origin']
                                                            for arc in self.network.out_edges(stop)
                                                            ))
                         for req in self.requests}

        self.stop_in = {req: self.m.addConstr(self.y[req] <=
                                              grp.quicksum(self.x[arc]
                                                           for stop in self.delta[req]['destination']
                                                           for arc in self.network.in_edges(stop)
                                                           ))
                        for req in self.requests}

        self.flow_conservation = {
            stop: self.m.addConstr(grp.quicksum(self.x[arc] for arc in self.network.out_edges(stop))
                                   - grp.quicksum(self.x[arc] for arc in self.network.in_edges(stop))
                                   == 0, name=f'Flow conservation {stop}')
            for stop in self.network.nodes()
            if stop != self.compulsory_stops[0] and stop != self.compulsory_stops[-1]}

        self.flow_conservation_origin = self.m.addConstr(grp.quicksum(self.x[arc]
                                                                      for arc in
                                                                      self.network.out_edges(self.compulsory_stops[0]))
                                                         == 1, name=f'Flow conservation origin')

        self.flow_conservation_destination = self.m.addConstr(grp.quicksum(self.x[arc]
                                                                           for arc in
                                                                           self.network.in_edges(
                                                                               self.compulsory_stops[-1]))
                                                              == 1, name=f'Flow conservation destination')

        self.time_compulsory = {segment: self.m.addConstr(self.t[self.compulsory_stops[segment]] +
                                                          grp.quicksum(self.network[i][j]['travel_time'] *
                                                                       self.x[(i, j)]
                                                                       for (i, j) in
                                                                       self.get_segment_edges(segment + 1))
                                                          <= self.t[self.compulsory_stops[segment + 1]],
                                                          name=f'Time_compulsory_{segment}')
                                for segment in range(len(self.compulsory_stops) - 1)}

        self.time_window_a = {stop: self.m.addConstr(self.t[stop] >= self.time_windows[stop]['a'],
                                                     name=f'Time_window_a_{stop}')
                              for stop in self.compulsory_stops}

        self.time_window_b = {stop: self.m.addConstr(self.t[stop] <= self.time_windows[stop]['b'],
                                                     name=f'Time_window_b_{stop}')
                              for stop in self.compulsory_stops}

        self.out_flow = {
            stop: self.m.addConstr(grp.quicksum(self.x[arc] for arc in self.network.out_edges(stop))
                                   <= 1, name=f'Out flow {stop}')
            for stop in self.network.nodes()}

    def solve(self):
        # Set objective to maximization
        self.m.Params.OutputFlag = 0
        self.m.ModelSense = -1
        self.m.Params.TimeLimit = 2*60
        self.m.Params.MIPGap = 0.01

        # Define attributes needed for subtour elimination
        self.m._vars = self.x
        self.m._nodes = self.network.nodes()
        self.m._edges = self.network.edges()
        self.m._origin = self.compulsory_stops[0]
        self.m._subtours = list()

        # Enable lazy constraints
        self.m.Params.lazyConstraints = 1

        self.m.optimize(subtourelim)

    def get_used_arcs(self):
        used_arcs = [arc for arc, var in self.x.items() if var.X > 0.5]
        current_arc = next(arc for arc in used_arcs if arc[0] == self.compulsory_stops[0])
        ordered_arcs = [current_arc]

        used_arcs.remove(current_arc)
        while len(used_arcs) > 0:
            try:
                current_arc = next(arc for arc in used_arcs if arc[0] == current_arc[1])
            except:
                return
            ordered_arcs.append(current_arc)
            used_arcs.remove(current_arc)
        return ordered_arcs

    def get_served_requests(self):
        return [req for req, var in self.y.items() if var.X > 0.5]

    def get_arrival_times(self):
        return [var.X for stop, var in self.t.items()]

    def optional_stops_visited(self):
        optional_stops = set()
        for arc in self.get_used_arcs():
            if not arc[0].is_compulsory:
                optional_stops.add(arc[0])

            if not arc[1].is_compulsory:
                optional_stops.add(arc[1])

        return list(optional_stops)

    def travel_distance(self):
        return sum(self.network[i][j]['weight'] for (i, j) in self.get_used_arcs())

    def travel_time(self):
        return sum(self.network[i][j]['travel_time'] for (i, j) in self.get_used_arcs())

    def get_wrong_order_requests(self):
        arc_set = self.get_used_arcs()
        ordered_nodes = [arc[0] for arc in arc_set] + [arc_set[-1][1]]

        wrong_order_requests = list()

        for req in self.get_served_requests():
            earliest_pickup = min([ordered_nodes.index(stop)
                                   # else len(ordered_nodes) + 1
                                   for stop in self.delta[req]['origin']
                                   if stop in ordered_nodes])

            latest_dropoff = max([ordered_nodes.index(stop)
                                  # else -1
                                  for stop in self.delta[req]['destination']
                                  if stop in ordered_nodes])

            if earliest_pickup > latest_dropoff:
                wrong_order_requests.append(req)

        return wrong_order_requests

    def force_request_accept(self, requests):
        for req in requests:
            self.m.addConstr(self.y[req] >= 1)

    def force_fixed_route(self):
        for a, b in zip(self.compulsory_stops, self.compulsory_stops[1:]):
            self.m.addConstr(self.x[(a, b)] == 1)
        return
