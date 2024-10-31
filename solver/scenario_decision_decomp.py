import gurobipy as grp
from gurobipy import GRB
import time
import math

def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # Save data from new solution
        cur_obj = model.cbGet(grp.GRB.Callback.MIPSOL_OBJBST)
        cur_bd = model.cbGet(grp.GRB.Callback.MIPSOL_OBJBND)

        # Did objective value or best bound change?
        if model._obj != cur_obj or model._bd != cur_bd:
            model._obj = cur_obj
            model._bd = cur_bd
            model._data.append([time.time() - model._start, cur_obj, cur_bd, abs((cur_bd - cur_obj) / cur_obj)])

        # Find subtours in current solution and add respective lazy elimination constraints to the model
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)

        # For each scenario, find all selected edges
        selected = grp.tuplelist(edge for edge in model._edges if vals[edge] > 0.5)

        # Find all subtours in each scenario
        subtours = subtour(selected)

        # Add a subtour elimination constraint to each scenario for each subtour
        if len(subtours) > 1:
            for tour in subtours:
                if tour not in model._subtours:
                    for scenario2 in model._scenarios:
                        # add subtour elimination constr. for every pair of cities in subtour
                        model.cbLazy(grp.quicksum(model._vars[edge] for edge in tour)
                                     <= len(tour) - 1)
                        model._subtours.append(tour)


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
            model._data.append([time.time() - model._start, cur_obj, cur_bd, (cur_bd - cur_obj) / cur_obj])


class ScenarioDecisionDecomp:
    def __init__(self, network, compulsory_stops, scenarios, accepted_requests, prev_requests, curr_request, deltas,
                 time_windows):
        self.network = network
        self.compulsory_stops = compulsory_stops
        self.time_windows = time_windows

        self.scenarios = scenarios
        self.prev_requests = prev_requests
        self.accepted_requests = accepted_requests
        self.curr_request = curr_request
        self.deltas = deltas  # {scenario: {request: {'origin': {stop1: 0, stop2: 1}}, {'destination': {stop1: 1, stop2: 0}}}}

        # Models
        self.models = dict()

        # Variables
        self.x = dict()
        self.y = dict()
        self.t = dict()
        self.curr_request_y = None

        # Constraints
        self.accepted_requests_constraints = dict()
        self.linking_constraint = dict()

        self.stop_out = dict()
        self.stop_in = dict()
        self.flow_conservation = dict()
        self.flow_conservation_origin = dict()
        self.flow_conservation_destination = dict()
        self.time_compulsory = dict()
        self.time_window_a = dict()
        self.time_window_b = dict()
        self.out_flow = dict()

    def get_segment_edges(self, segment):
        # Not optimized
        return [(i, j) for (i, j) in self.network.edges() if segment in self.network[i][j]['segment']]

    def get_segment_nodes(self, segment):
        return [stop for stop in self.network.nodes() if segment in stop.segment]

    def build_models(self, decision=None):
        build_times = []
        for scenario in self.scenarios:
            t = time.time()
            self.models[scenario] = grp.Model(f'{scenario}_model')

            # Variables for each scenario
            self.x[scenario] = {(i, j): self.models[scenario].addVar(vtype=GRB.BINARY, name=f'x_{scenario}_{(i, j)}',
                                                      obj=-self.network[i][j]['weight'] * self.scenarios[scenario][
                                                          'probability'])
                                for (i, j) in self.network.edges()}

            self.y[scenario] = {req: self.models[scenario].addVar(vtype=GRB.BINARY, name=f'y_{scenario}_{req.request_time}',
                                                   obj=req.utility * self.scenarios[scenario]['probability'])
                                for req in self.prev_requests[scenario]}
            self.t[scenario] = {stop: self.models[scenario].addVar(vtype=GRB.CONTINUOUS, name=f't_{scenario}_{stop}')
                                for stop in self.compulsory_stops}

            # All previously accepted requests are still served
            self.accepted_requests_constraints[scenario] = {
                req: self.models[scenario].addConstr(self.y[scenario][req] >= 1)
                for req in self.prev_requests[scenario] if
                req in self.accepted_requests}

            # Scenario constraints
            self.add_stop_constraints(scenario)
            self.add_flow_conservation(scenario)
            self.add_time_constraints(scenario)
            self.add_out_constraints(scenario)
            build_times.append(time.time() - t)

        if decision in [True, False]:
            self.fix_decision(decision)
        return build_times

    # def solve_models_parallel(self, scenario, with_subtourelim=True):
    #     with grp.Env() as env, grp.Model() as model:
    #         t = time.time()
    #         self.models[scenario] = grp.Model(f'{scenario}_model')
    #
    #         # Variables for each scenario
    #         self.x[scenario] = {(i, j): self.models[scenario].addVar(vtype=GRB.BINARY, name=f'x_{scenario}_{(i, j)}',
    #                                                   obj=-self.network[i][j]['weight'] * self.scenarios[scenario][
    #                                                       'probability'])
    #                             for (i, j) in self.network.edges()}
    #
    #         self.y[scenario] = {req: self.models[scenario].addVar(vtype=GRB.BINARY, name=f'y_{scenario}_{req.request_time}',
    #                                                obj=req.utility * self.scenarios[scenario]['probability'])
    #                             for req in self.prev_requests[scenario]}
    #         self.t[scenario] = {stop: self.models[scenario].addVar(vtype=GRB.CONTINUOUS, name=f't_{scenario}_{stop}')
    #                             for stop in self.compulsory_stops}
    #
    #         # All previously accepted requests are still served
    #         self.accepted_requests_constraints[scenario] = {
    #             req: self.models[scenario].addConstr(self.y[scenario][req] >= 1)
    #             for req in self.prev_requests[scenario] if
    #             req in self.accepted_requests}
    #
    #         # Scenario constraints
    #         self.add_stop_constraints(scenario)
    #         self.add_flow_conservation(scenario)
    #         self.add_time_constraints(scenario)
    #         self.add_out_constraints(scenario)
    #         build_time = time.time() - t
    #
    #         ### Solve
    #         t = time.time()
    #         # Set objective to maximization
    #         self.models[scenario].Params.Threads = 1
    #         self.models[scenario].ModelSense = -1
    #         self.models[scenario].Params.TimeLimit = 10 * 60
    #         self.models[scenario].Params.MIPGap = 0.02
    #
    #         # Define attributes needed for subtour elimination
    #         self.models[scenario]._vars = self.x[scenario]
    #         self.models[scenario]._nodes = self.network.nodes()
    #         self.models[scenario]._edges = self.network.edges()
    #         self.models[scenario]._origin = self.compulsory_stops[0]
    #         self.models[scenario]._subtours = list()
    #         self.models[scenario]._scenarios = self.scenarios.keys()
    #
    #         # Define attributes to save bounds and gap
    #         self.models[scenario]._obj = None
    #         self.models[scenario]._bd = None
    #         self.models[scenario]._data = [['Time', 'Objective', 'Bound', 'Gap']]
    #         self.models[scenario]._start = time.time()
    #
    #         if with_subtourelim:
    #             # Enable lazy constraints
    #             self.models[scenario].Params.lazyConstraints = 1
    #             self.models[scenario].optimize(callback=subtourelim)
    #         else:
    #             self.models[scenario].optimize(callback=data_cb)
    #         self.models[scenario]._data.append(
    #             [time.time() - self.models[scenario]._start, self.models[scenario].objVal,
    #              self.models[scenario].ObjBound, self.models[scenario].MIPGap])
    #
    #         solve_time = time.time() - t
    #         return {'decision': self.y[scenario][self.curr_request].X, 'build_time': build_time, 'solve_time': solve_time}

    def add_stop_constraints(self, scenario):
        # print('Add stop out constraints')
        self.stop_out[scenario] = {req: self.models[scenario].addConstr(self.y[scenario][req] <=
                                                                        grp.quicksum(self.x[scenario][arc]
                                                                                     for stop in
                                                                                     self.deltas[scenario][req][
                                                                                         'origin']
                                                                                     for arc in
                                                                                     self.network.out_edges(stop)
                                                                                     ))
                                   for req in self.prev_requests[scenario]}

        # print('Add stop in constraints')
        self.stop_in[scenario] = {req: self.models[scenario].addConstr(self.y[scenario][req] <=
                                                                       grp.quicksum(self.x[scenario][arc]
                                                                                    for stop in
                                                                                    self.deltas[scenario][req][
                                                                                        'destination']
                                                                                    for arc in
                                                                                    self.network.in_edges(stop)
                                                                                    ))
                                  for req in self.prev_requests[scenario]}

    def add_flow_conservation(self, scenario):
        # print('Add flow conservation constraints')
        self.flow_conservation[scenario] = {stop: self.models[scenario].addConstr(
            grp.quicksum(self.x[scenario][arc] for arc in self.network.out_edges(stop))
            - grp.quicksum(self.x[scenario][arc] for arc in self.network.in_edges(stop))
            == 0, name=f'Flow conservation {scenario} {stop}')
            for stop in self.network.nodes()
            if stop != self.compulsory_stops[0] and stop != self.compulsory_stops[-1]}

        # print('Add flow origin constraints')
        self.flow_conservation_origin[scenario] = self.models[scenario].addConstr(
            grp.quicksum(self.x[scenario][arc]
                         for arc in
                         self.network.out_edges(
                             self.compulsory_stops[
                                 0]))
            == 1,
            name=f'Flow conservation origin {scenario}')

        # print('Add flow destination constraints')
        self.flow_conservation_destination[scenario] = self.models[scenario].addConstr(
            grp.quicksum(self.x[scenario][arc]
                         for arc in
                         self.network.in_edges(
                             self.compulsory_stops[-1]))
            == 1,
            name=f'Flow conservation destination {scenario}')

    def add_time_constraints(self, scenario):
        # print('Add time constraints')
        self.time_compulsory[scenario] = {
            segment: self.models[scenario].addConstr(self.t[scenario][self.compulsory_stops[segment]] +
                                                     grp.quicksum(self.network[i][j]['travel_time'] *
                                                                  self.x[scenario][(i, j)]
                                                                  for (i, j) in
                                                                  self.get_segment_edges(segment + 1))
                                                     <= self.t[scenario][self.compulsory_stops[segment + 1]],
                                                     name=f'Time_compulsory_{scenario}_{segment}')
            for segment in range(len(self.compulsory_stops) - 1)}

        self.time_window_a[scenario] = {
            stop: self.models[scenario].addConstr(self.t[scenario][stop] >= self.time_windows[stop]['a'],
                                                  name=f'Time_window_a_{scenario}_{stop}')
            for stop in self.compulsory_stops}

        self.time_window_b[scenario] = {
            stop: self.models[scenario].addConstr(self.t[scenario][stop] <= self.time_windows[stop]['b'],
                                                  name=f'Time_window_b_{scenario}_{stop}')
            for stop in self.compulsory_stops}

    def add_out_constraints(self, scenario):
        self.out_flow[scenario] = {
            stop: self.models[scenario].addConstr(grp.quicksum(self.x[scenario][arc] for arc in self.network.out_edges(stop))
                                   <= 1, name=f'Out flow {stop}')
            for stop in self.network.nodes()}

    def solve(self, with_subtourelim=False):
        solve_times = []
        for scenario in self.scenarios:
            t = time.time()
            # Set objective to maximization
            self.models[scenario].Params.OutputFlag = 0
            self.models[scenario].Params.Threads = 2
            self.models[scenario].ModelSense = -1
            self.models[scenario].Params.TimeLimit = 10 * 60
            self.models[scenario].Params.MIPGap = 0.03

            # Define attributes needed for subtour elimination
            self.models[scenario]._vars = self.x[scenario]
            self.models[scenario]._nodes = self.network.nodes()
            self.models[scenario]._edges = self.network.edges()
            self.models[scenario]._origin = self.compulsory_stops[0]
            self.models[scenario]._subtours = list()
            self.models[scenario]._scenarios = self.scenarios.keys()

            # Define attributes to save bounds and gap
            self.models[scenario]._obj = None
            self.models[scenario]._bd = None
            self.models[scenario]._data = [['Time', 'Objective', 'Bound', 'Gap']]
            self.models[scenario]._start = time.time()

            if with_subtourelim:
                # Enable lazy constraints
                self.models[scenario].Params.lazyConstraints = 1
                self.models[scenario].optimize(callback=subtourelim)
            else:
                self.models[scenario].optimize(callback=data_cb)
            # self.models[scenario]._data.append([time.time() - self.models[scenario]._start, self.models[scenario].objVal, self.models[scenario].ObjBound, self.models[scenario].MIPGap])
            solve_times.append(time.time() - t)

        return solve_times
    def get_avg_served_req(self):
        return sum([val.X for val in self.y.values()]) / len(self.prev_requests)

    def get_accepted_requests(self):
        count_accepted = sum([self.y[scenario][self.curr_request].X for scenario in self.scenarios])
        return self.accepted_requests + [self.curr_request] if count_accepted > len(self.scenarios) // 2 else self.accepted_requests

    def get_accepted_requests_parallel(self, decisions):
        return self.accepted_requests + [self.curr_request] if sum(decisions) > len(self.scenarios) // 2 else self.accepted_requests

    def fix_decision(self, decision):
        for scenario in self.scenarios:
            if decision:
                self.models[scenario].addConstr(self.y[scenario][self.curr_request] >= 1, name=f'Fix decision')
            else:
                self.models[scenario].addConstr(self.y[scenario][self.curr_request] <= 0, name=f'Fix decision')

    def get_objectives_fix_decision(self):
        # Check that all scanrios are feasible (we can break after the first scenario because of theorem 3.1)
        for scenario in self.scenarios:
            if self.models[scenario].status == 3:
                return -math.inf
            else:
                break

        return sum(self.models[scenario].objVal for scenario in self.scenarios)





