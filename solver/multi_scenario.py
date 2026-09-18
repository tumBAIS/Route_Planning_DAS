import gurobipy as grp
from gurobipy import GRB
import time

def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # Save data from new solution
        cur_obj = model.cbGet(grp.GRB.Callback.MIPSOL_OBJBST)
        cur_bd = model.cbGet(grp.GRB.Callback.MIPSOL_OBJBND)

        # Did objective value or best bound change?
        if model._obj != cur_obj or model._bd != cur_bd:
            model._obj = cur_obj
            model._bd = cur_bd
            model._data.append([time.time() - model._start, cur_obj, cur_bd, abs((cur_bd - cur_obj)/cur_obj)])

        # Find subtours in current solution and add respective lazy elimination constraints to the model
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)


        subtours = {}
        for scenario1 in model._scenarios:
            # For each scenario, find all selected edges
            selected = grp.tuplelist(edge for edge in model._edges if vals[(scenario1, edge)] > 0.5)

            # Find all subtours in each scenario
            subtours[scenario1] = subtour(selected)

            # Add a subtour elimination constraint to each scenario for each subtour
            if len(subtours[scenario1]) > 1:
                for tour in subtours[scenario1]:
                    if tour not in model._subtours:
                        for scenario2 in model._scenarios:
                            # add subtour elimination constr. for every pair of cities in subtour
                            model.cbLazy(grp.quicksum(model._vars[(scenario2, edge)] for edge in tour)
                                         <= len(tour) - 1)
                            model._subtours.append(tour)

        # selected = grp.tuplelist(edge for edge in model._edges
        #                          if vals[edge] > 0.5)
        #
        # # find the shortest cycle in the selected edge list
        # subtours = subtour(selected)
        # if len(subtours) > 1:
        #     for tour in subtours:
        #         # add subtour elimination constr. for every pair of cities in subtour
        #         model.cbLazy(grp.quicksum(model._vars[edge] for edge in tour)
        #                      <= len(tour) - 1)
        #         model._subtours.append(tour)

def subtour(used_edges):
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

def data_cb(model, where):
    if where == grp.GRB.Callback.MIPSOL:
        cur_obj = model.cbGet(grp.GRB.Callback.MIPSOL_OBJBST)
        cur_bd = model.cbGet(grp.GRB.Callback.MIPSOL_OBJBND)

        # Did objective value or best bound change?
        if model._obj != cur_obj or model._bd != cur_bd:
            model._obj = cur_obj
            model._bd = cur_bd
            model._data.append([time.time() - model._start, cur_obj, cur_bd, (cur_bd - cur_obj)/cur_obj])

class MultiScenario:
    def __init__(self, network, compulsory_stops, scenarios, requests, deltas, time_windows, features):
        self.network = network
        self.compulsory_stops = compulsory_stops
        self.time_windows = time_windows

        self.scenarios = scenarios
        self.requests = requests
        self.deltas = deltas  # {scenario: {request: {'origin': {stop1: 0, stop2: 1}}, {'destination': {stop1: 1, stop2: 0}}}}
        self.features = features
        self.M = 100000
        self.epsilon = 1

        # Model
        self.m = None

        # Variables
        self.x = None
        self.y = None
        self.t = None
        self.w = None

        # Constraints
        self.positive_w = None
        self.negative_w = None

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
        # Not optimized
        return [(i, j) for (i, j) in self.network.edges() if segment in self.network[i][j]['segment']]

    def get_segment_nodes(self, segment):
        return [stop for stop in self.network.nodes() if segment in stop.segment]

    def build_model(self):
        self.m = grp.Model('Deterministic')

        # Variables
        self.x = {(scenario, (i, j)): self.m.addVar(vtype=GRB.BINARY, name=f'x_{scenario}_{(i, j)}',
                                                    obj=-self.network[i][j]['weight'] * self.scenarios[scenario][
                                                        'probability'])
                  for (i, j) in self.network.edges() for scenario in self.scenarios}
        self.y = {(scenario, req): self.m.addVar(vtype=GRB.BINARY,
                                                 obj=req.utility * self.scenarios[scenario]['probability'])
                  for scenario in self.scenarios for req in self.requests[scenario]}
        self.t = {(scenario, stop): self.m.addVar(vtype=GRB.CONTINUOUS, name=f't_{scenario}_{stop}')
                  for scenario in self.scenarios for stop in self.compulsory_stops}

        self.w = {feature: self.m.addVar(vtype=GRB.CONTINUOUS, ub=1, lb=-1, name=f'w_{feature}')
                  for feature in list([feature.keys() for feature in self.features[0].values()][0])}

        # Policy constraints
        print('Add policy constraints')
        self.positive_w = {(scenario, req): self.m.addConstr(self.M * self.y[(scenario, req)] >=
                                                             grp.quicksum(
                                                                 self.w[feature] * self.features[scenario][req][feature]
                                                                 for feature in self.features[scenario][req])
                                                             + self.epsilon)
                           for scenario in self.scenarios for req in self.requests[scenario]}

        self.negative_w = {(scenario, req): self.m.addConstr(self.M * (self.y[(scenario, req)] - 1) <=
                                                             grp.quicksum(
                                                                 self.w[feature] * self.features[scenario][req][feature]
                                                                 for feature in self.features[scenario][req])
                                                             )
                           for scenario in self.scenarios for req in self.requests[scenario]}

        # Scenario constraints
        self.add_stop_constraints()
        self.add_flow_conservation()
        self.add_time_constraints()
        self.add_out_constraints()


    def add_stop_constraints(self):
        print('Add stop out constraints')
        self.stop_out = {(scenario, req): self.m.addConstr(self.y[(scenario, req)] <=
                                                           grp.quicksum(self.x[(scenario, arc)]
                                                                        for stop in self.deltas[scenario][req]['origin']
                                                                        for arc in self.network.out_edges(stop)
                                                                        ))
                         for scenario in self.scenarios for req in self.requests[scenario]}

        print('Add stop in constraints')
        self.stop_in = {(scenario, req): self.m.addConstr(self.y[(scenario, req)] <=
                                                          grp.quicksum(self.x[(scenario, arc)]
                                                                       for stop in self.deltas[scenario][req][
                                                                           'destination']
                                                                       for arc in self.network.in_edges(stop)
                                                                       ))
                        for scenario in self.scenarios for req in self.requests[scenario]}

    def add_flow_conservation(self):
        print('Add flow conservation constraints')
        self.flow_conservation = {
            (scenario, stop): self.m.addConstr(
                grp.quicksum(self.x[(scenario, arc)] for arc in self.network.out_edges(stop))
                - grp.quicksum(self.x[(scenario, arc)] for arc in self.network.in_edges(stop))
                == 0, name=f'Flow conservation {scenario} {stop}')
            for stop in self.network.nodes()
            if stop != self.compulsory_stops[0] and stop != self.compulsory_stops[-1]
            for scenario in self.scenarios}

        print('Add flow origin constraints')
        self.flow_conservation_origin = {scenario:
                                             self.m.addConstr(grp.quicksum(self.x[(scenario, arc)]
                                                                           for arc in
                                                                           self.network.out_edges(
                                                                               self.compulsory_stops[0]))
                                                              == 1, name=f'Flow conservation origin {scenario}')
                                         for scenario in self.scenarios}

        print('Add flow destination constraints')
        self.flow_conservation_destination = {scenario: self.m.addConstr(grp.quicksum(self.x[(scenario, arc)]
                                                                                      for arc in
                                                                                      self.network.in_edges(
                                                                                          self.compulsory_stops[-1]))
                                                                         == 1,
                                                                         name=f'Flow conservation destination {scenario}')
                                              for scenario in self.scenarios}

    def add_time_constraints(self):
        print('Add time constraints')
        self.time_compulsory = {
            (scenario, segment): self.m.addConstr(self.t[(scenario, self.compulsory_stops[segment])] +
                                                  grp.quicksum(self.network[i][j]['travel_time'] *
                                                               self.x[(scenario, (i, j))]
                                                               for (i, j) in
                                                               self.get_segment_edges(segment + 1))
                                                  <= self.t[(scenario, self.compulsory_stops[segment + 1])],
                                                  name=f'Time_compulsory_{scenario}_{segment}')
            for segment in range(len(self.compulsory_stops) - 1)
            for scenario in self.scenarios}

        self.time_window_a = {
            (scenario, stop): self.m.addConstr(self.t[(scenario, stop)] >= self.time_windows[stop]['a'],
                                               name=f'Time_window_a_{scenario}_{stop}')
            for stop in self.compulsory_stops
            for scenario in self.scenarios}

        self.time_window_b = {
            (scenario, stop): self.m.addConstr(self.t[(scenario, stop)] <= self.time_windows[stop]['b'],
                                               name=f'Time_window_b_{scenario}_{stop}')
            for stop in self.compulsory_stops
            for scenario in self.scenarios}

    def add_out_constraints(self):
        self.out_flow = {
            stop: self.m.addConstr(grp.quicksum(self.x[(scenario, arc)] for arc in self.network.out_edges(stop))
                                   <= 1, name=f'Out flow {stop}')
            for stop in self.network.nodes()
        for scenario in self.scenarios}


    def solve(self, with_subtourelim=False):
        # Set objective to maximization
        self.m.ModelSense = -1
        self.m.Params.TimeLimit = 10*60
        self.m.Params.MIPGap = 0.01

        # Define attributes needed for subtour elimination
        self.m._vars = self.x
        self.m._nodes = self.network.nodes()
        self.m._edges = self.network.edges()
        self.m._origin = self.compulsory_stops[0]
        self.m._subtours = list()
        self.m._scenarios = self.scenarios.keys()

        # Define attributes to save bounds and gap
        self.m._obj = None
        self.m._bd = None
        self.m._data = [['Time', 'Objective', 'Bound', 'Gap']]
        self.m._start = time.time()


        if with_subtourelim:
            # Enable lazy constraints
            self.m.Params.lazyConstraints = 1
            self.m.optimize(callback=subtourelim)
        else:
            self.m.optimize(callback=data_cb)
        self.m._data.append([time.time() - self.m._start, self.m.objVal, self.m.ObjBound, self.m.MIPGap])

    def get_avg_served_req(self):
        return sum([val.X for val in self.y.values()]) / len(self.requests)

