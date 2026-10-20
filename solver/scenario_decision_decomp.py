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

        # make a list of arcs selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = [arc for arc, val in vals.items() if val > 0.5]

        # add a generalized subtour elimination constraint (GSEC) for every cycle and every stop k in it:
        # sum_{(i, j) in A(S)} x_ij <= sum_{i in S \ {k}} v_i, where v_i = sum_{(i, j) in out(i)} x_ij (i visited)
        # cycles never contain the destination, so the cut is valid for every origin -> destination route
        for cycle in find_cycles(selected, model._origin):
            stops = set(cycle)
            inside_arcs = [(i, j) for i in cycle for (i, j) in model._out_arcs[i] if j in stops]
            for k in cycle:
                model.cbLazy(grp.quicksum(model._vars[arc] for arc in inside_arcs)
                             <= grp.quicksum(model._vars[arc] for i in cycle if i != k for arc in model._out_arcs[i]))
            model._subtours.append(cycle)


def find_cycles(selected, origin):
    """Split the selected arcs of an integer solution into the origin -> destination path and the remaining cycles.

    Every stop has at most one outgoing arc, flow is conserved at all stops except origin and destination, the origin
    has no incoming and the destination no outgoing arcs. Hence, all selected arcs that are not on the path form
    disjoint cycles. Returns the cycles as lists of stops."""
    succ = dict(selected)

    # Remove the origin -> destination path
    node = origin
    while node in succ:
        node = succ.pop(node)

    # All remaining arcs form disjoint cycles
    cycles = list()
    while succ:
        node, cycle = next(iter(succ)), list()
        while node in succ:
            cycle.append(node)
            node = succ.pop(node)
        cycles.append(cycle)

    return cycles


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
                 time_windows, bus_capacity, accepted_stops=None):
        self.network = network
        self.compulsory_stops = compulsory_stops
        self.time_windows = time_windows

        self.scenarios = scenarios
        self.prev_requests = prev_requests
        self.accepted_requests = accepted_requests
        self.curr_request = curr_request
        self.deltas = deltas  # {scenario: {request: {'origin': {stop1: 0, stop2: 1}}, {'destination': {stop1: 1, stop2: 0}}}}
        self.bus_capacity = bus_capacity
        self.accepted_stops = accepted_stops if accepted_stops else dict()

        # Models
        self.models = dict()

        # Variables
        self.x = dict()
        self.y = dict()
        self.t = dict()
        self.k = dict()
        self.p = dict()
        self.o = dict()
        self.u = dict()
        self.pickup_and_dropoff = dict()
        self.curr_request_y = None

        # Constraints
        self.accepted_requests_constraints = dict()
        self.linking_constraint = dict()

        self.stop_out = dict()
        self.stop_in = dict()
        self.request_p = dict()
        self.request_o = dict()
        self.request_p_low = dict()
        self.request_o_low = dict()
        self.flow_conservation = dict()
        self.flow_conservation_origin = dict()
        self.flow_conservation_destination = dict()
        self.time_compulsory = dict()
        self.time_window_a = dict()
        self.time_window_b = dict()
        self.out_flow = dict()
        self.capacity_limit = dict()
        self.picked_up_req = dict()
        self.dropped_off_req = dict()
        self.capacity_propagation = dict()
        self.initial_capacity = dict()
        self.invalid_pairs = dict()
        self.order_propagation = dict()
        self.pickup_and_dropoff_and = dict()
        self.pickup_before_dropoff = dict()
        self.fixed_stops_constraints = dict()

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
                                                                     obj=-self.network[i][j]['weight'] *
                                                                         self.scenarios[scenario][
                                                                             'probability'])
                                for (i, j) in self.network.edges()}

            self.y[scenario] = {
                req: self.models[scenario].addVar(vtype=GRB.BINARY, name=f'y_{scenario}_{req.request_time}',
                                                          obj=req.utility * self.scenarios[scenario]['probability'])
                for req in self.prev_requests[scenario]}

            self.t[scenario] = {stop: self.models[scenario].addVar(vtype=GRB.CONTINUOUS, name=f't_{scenario}_{stop}')
                                for stop in self.compulsory_stops}

            self.k[scenario] = {(i, j): self.models[scenario].addVar(vtype=GRB.CONTINUOUS, name=f'k_{scenario}_{(i, j)}')
                                for (i, j) in self.network.edges()}

            self.p[scenario] = {(req, i): self.models[scenario].addVar(vtype=GRB.BINARY, name=f'pickup_{scenario}_{(req, i)}')
                                for req in self.prev_requests[scenario]
                                for i in self.deltas[scenario][req]['origin']}

            self.o[scenario] = {(req, j): self.models[scenario].addVar(vtype=GRB.BINARY, name=f'dropoff_{scenario}_{(req, j)}')
                                for req in self.prev_requests[scenario]
                                for j in self.deltas[scenario][req]['destination']}

            # All previously accepted requests are still served
            self.accepted_requests_constraints[scenario] = {
                req: self.models[scenario].addConstr(self.y[scenario][req] >= 1)
                for req in self.prev_requests[scenario] if
                req in self.accepted_requests}

            # Serve previously accepted stops at their determined pickup/dropoff locations
            self.fixed_stops_constraints[scenario] = dict()
            for req, stops in self.accepted_stops.items():
                if not stops:
                    continue
                pick, drop = stops
                if (req, pick) in self.p[scenario] and (req, drop) in self.o[scenario]:
                    self.fixed_stops_constraints[scenario][req] = (
                        self.models[scenario].addConstr(self.p[scenario][(req, pick)] >= 1),
                        self.models[scenario].addConstr(self.o[scenario][(req, drop)] >= 1)
                    )

            # Scenario constraints
            self.add_stop_constraints(scenario)
            self.add_flow_conservation(scenario)
            self.add_time_constraints(scenario)
            self.add_out_constraints(scenario)
            self.add_capacity_constraints(scenario)
            self.add_precedence_constraints(scenario)
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
        self.stop_out[scenario] = {
            (req, stop): self.models[scenario].addConstr(
                self.p[scenario][(req, stop)] <= (
                    grp.quicksum(self.x[scenario][arc] for arc in self.network.out_edges(stop))
                )
            )
            for req in self.prev_requests[scenario]
            for stop in self.deltas[scenario][req]['origin']
        }

        self.stop_in[scenario] = {
            (req, stop): self.models[scenario].addConstr(
                self.o[scenario][(req, stop)] <= (
                    grp.quicksum(self.x[scenario][arc] for arc in self.network.in_edges(stop))
                )
            )
            for req in self.prev_requests[scenario]
            for stop in self.deltas[scenario][req]['destination']
        }

        self.request_p[scenario] = {
            req: self.models[scenario].addConstr(grp.quicksum(self.p[scenario][(req, origin)]
                                                              for origin in self.deltas[scenario][req]['origin'])
                                                 == self.y[scenario][req])
            for req in self.prev_requests[scenario]}

        self.request_o[scenario] = {
            req: self.models[scenario].addConstr(grp.quicksum(self.o[scenario][(req, dest)]
                                                              for dest in self.deltas[scenario][req]['destination'])
                                                 == self.y[scenario][req])
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
            stop: self.models[scenario].addConstr(
                grp.quicksum(self.x[scenario][arc] for arc in self.network.out_edges(stop))
                <= 1, name=f'Out flow {stop}')
            for stop in self.network.nodes()}

    def add_capacity_constraints(self, scenario):
        # print('Add capacity constraints')
        self.capacity_limit[scenario] = {
            (i, j): self.models[scenario].addConstr(
                self.k[scenario][(i, j)] <= self.bus_capacity * self.x[scenario][(i, j)],
                name=f'Capa_limit_{scenario}_{(i, j)}')
            for (i, j) in self.network.edges()}

        self.capacity_propagation[scenario] = {
            stop: self.models[scenario].addConstr(
                grp.quicksum(self.k[scenario][(stop, dest)] for (stop, dest) in self.network.out_edges(stop)) -
                grp.quicksum(self.k[scenario][(origin, stop)] for (origin, stop) in self.network.in_edges(stop)) ==
                grp.quicksum(self.p[scenario][(req, stop)] for req in self.prev_requests[scenario] if (req, stop) in self.p[scenario]) -
                grp.quicksum(self.o[scenario][(req, stop)] for req in self.prev_requests[scenario] if (req, stop) in self.o[scenario]),
                name=f'Capa_prop_{scenario}_{stop}')
            for stop in self.network.nodes() if stop not in (self.compulsory_stops[0], self.compulsory_stops[-1])}

        origin = self.compulsory_stops[0]
        self.initial_capacity[scenario] = {
            self.models[scenario].addConstr(
                grp.quicksum(self.k[scenario][(origin, dest)] for (origin, dest) in self.network.out_edges(origin)) == 
                grp.quicksum(self.p[scenario][(req,origin)] for req in self.prev_requests[scenario] if (req, origin) in self.p[scenario]),
            name= f'Initial_capacity_{scenario}')}

    def _stop_interval(self, stop):
        if stop.is_compulsory:
            pos = self.compulsory_stops.index(stop)
            return pos, pos
        return min(stop.segment) - 1, max(stop.segment)

    def add_precedence_constraints(self, scenario):
        self.invalid_pairs[scenario] = dict()
        ordering_pairs = list()
        affected_segments = set()

        for req in self.prev_requests[scenario]:
            for seg_p in self.deltas[scenario][req]['origin']:
                low_p, high_p = self._stop_interval(seg_p)
                for seg_o in self.deltas[scenario][req]['destination']:
                    low_o, high_o = self._stop_interval(seg_o)
                    if seg_p == seg_o or high_o <= low_p:
                        self.invalid_pairs[scenario][(req, seg_p, seg_o)] = self.models[scenario].addConstr(
                            self.p[scenario][(req, seg_p)] + self.o[scenario][(req, seg_o)] <= 1,
                            name=f'Invalid_pair_{scenario}_{(req, seg_p, seg_o)}')
                    elif high_p <= low_o:
                        continue
                    else:
                        ordering_pairs.append((req, seg_p, seg_o))
                        affected_segments.update(set(seg_p.segment) & set(seg_o.segment))

        if not ordering_pairs:
            self.u[scenario] = dict()
            self.order_propagation[scenario] = dict()
            self.pickup_and_dropoff[scenario] = dict()
            self.pickup_and_dropoff_and[scenario] = dict()
            self.pickup_before_dropoff[scenario] = dict()
            return

        order_arcs = set()
        order_nodes = set()
        for segment in affected_segments:
            segment_arcs = self.get_segment_edges(segment)
            order_arcs.update(segment_arcs)
            order_nodes.update(node for arc in segment_arcs for node in arc)
        for (req, seg_p, seg_o) in ordering_pairs:
            order_nodes.update((seg_p, seg_o))

        self.u[scenario] = {stop: self.models[scenario].addVar(vtype=GRB.CONTINUOUS, ub=len(order_nodes), name=f'u_{scenario}_{stop}')
                            for stop in order_nodes}

        self.order_propagation[scenario] = {
            (i, j): self.models[scenario].addGenConstrIndicator(
                self.x[scenario][(i, j)], True, self.u[scenario][j] - self.u[scenario][i] >= 1, name=f'Order_prop_{scenario}_{(i, j)}')
                for (i, j) in order_arcs
        }

        self.pickup_and_dropoff[scenario] = {
            (req, seg_p, seg_o): self.models[scenario].addVar(
                vtype=GRB.BINARY, name=f'PickupDropoff_{scenario}_{(req, seg_p, seg_o)}')
                for (req, seg_p, seg_o) in ordering_pairs
        }

        self.pickup_and_dropoff_and[scenario] = {
            (req, seg_p, seg_o): self.models[scenario].addGenConstrAnd(
                self.pickup_and_dropoff[scenario][(req, seg_p, seg_o)],
                [self.p[scenario][(req, seg_p)], self.o[scenario][(req, seg_o)]],
                name=f'PickupDropoffAnd_{scenario}_{(req, seg_p, seg_o)}')
            for (req, seg_p, seg_o) in ordering_pairs
        }

        self.pickup_before_dropoff[scenario] = {
            (req, seg_p, seg_o): self.models[scenario].addGenConstrIndicator(
                self.pickup_and_dropoff[scenario][(req, seg_p, seg_o)], True, self.u[scenario][seg_o] - self.u[scenario][seg_p] >= 1,
                name=f'PickupBeforeDropoff_{scenario}_{(req, seg_p, seg_o)}')
            for (req, seg_p, seg_o) in ordering_pairs
        }

    def solve(self, with_subtourelim=False, stop_on_infeasible=False):
        print(f'solving')
        solve_times = []
        num_scen = 0
        len_scenario = len(self.scenarios)
        out_arcs = {stop: list(self.network.out_edges(stop)) for stop in self.network.nodes()}
        for scenario in self.scenarios:
            num_scen += 1
            print(f'solving scenario {num_scen} of {len_scenario}')
            t = time.time()
            # Set objective to maximization
            self.models[scenario].Params.OutputFlag = 0
            self.models[scenario].Params.Threads = 2
            self.models[scenario].ModelSense = -1
            self.models[scenario].Params.TimeLimit = 10 * 60
            self.models[scenario].Params.MIPGap = 0.01

            # Define attributes needed for subtour elimination
            self.models[scenario]._vars = self.x[scenario]
            self.models[scenario]._out_arcs = out_arcs
            self.models[scenario]._origin = self.compulsory_stops[0]
            self.models[scenario]._dest = self.compulsory_stops[-1]
            self.models[scenario]._subtours = list()
            self.models[scenario]._requests = self.prev_requests
            self.models[scenario]._p = self.p[scenario]
            self.models[scenario]._o = self.o[scenario]
            self.models[scenario]._k = self.k[scenario]

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

            if stop_on_infeasible and self.models[scenario].SolCount == 0:
                print(f'No solution for scenario {scenario} (status {self.models[scenario].status}), '
                      f'skipping the remaining {len_scenario - num_scen} scenarios')
                solve_times += [0.0] * (len_scenario - num_scen)
                break

        return solve_times

    def get_avg_served_req(self):
        return sum([val.X for val in self.y.values()]) / len(self.prev_requests)

    def get_accepted_requests(self):
        count_accepted = 0
        for scenario in self.scenarios:
            # No solution found within the time limit or model infeasible: count the scenario as a rejection
            if self.models[scenario].SolCount == 0:
                print(f'No solution for scenario {scenario} (status {self.models[scenario].status}), counted as reject')
                continue
            y = self.y[scenario][self.curr_request].X
            if 0.5 > y > 0:
                y = 0
            count_accepted += y
        return self.accepted_requests + [self.curr_request] if count_accepted > len(
            self.scenarios) // 2 else self.accepted_requests

    def get_accepted_requests_parallel(self, decisions):
        return self.accepted_requests + [self.curr_request] if sum(decisions) > len(
            self.scenarios) // 2 else self.accepted_requests

    ### Majority vote across per scenario solutions for the pickup and dropoff locations of a request;
    ### Tie breaker: highest average objective
    ### Called right after solving with y_{\theta} = 1
    def get_stop_assignment(self, request):
        votes = dict()
        objectives = dict()
        for scenario in self.scenarios:
            if self.models[scenario].SolCount == 0:
                continue
            pick = next((stop for (req, stop), var in self.p[scenario].items() if req == request and var.X > 0.5), None)
            drop = next((stop for (req, stop), var in self.o[scenario].items() if req == request and var.X > 0.5), None)
            if pick is not None and drop is not None:
                votes[(pick, drop)] = votes.get((pick, drop), 0) + 1
                objectives[(pick, drop)] = objectives.get((pick, drop), 0.0) + self.models[scenario].ObjVal
        if not votes:
            return None
        return max(votes, key=lambda pair: (votes[pair], objectives[pair] / votes[pair]))


    def fix_decision(self, decision):
        for scenario in self.scenarios:
            if decision:
                # Force at least one of the y_vars to be 1 (accept the request)
                self.models[scenario].addConstr(self.y[scenario][self.curr_request] >= 1, name=f'Fix decision')
            else:
                # Force all y_vars to 0 (reject the request)
                self.models[scenario].addConstr(self.y[scenario][self.curr_request] <= 0, name=f'Fix decision')

    def switch_to_reject(self):
        # Set start from solved accept without the current request, fix current request to reject
        switch_times = []
        for scenario in self.scenarios:
            t = time.time()
            model = self.models[scenario]
            start = dict()
            if model.SolCount > 0:
                start = {var: var.X for var in model.getVars()}
                # vehicle loads changed without current request, recompute loads
                for var in self.k[scenario].values():
                    start[var] = GRB.UNDEFINED
                start[self.y[scenario][self.curr_request]] = 0
                for (req, _), var in list(self.p[scenario].items()) + list(self.o[scenario].items()):
                    if req == self.curr_request:
                        start[var] = 0
                for (req, _, _), var in self.pickup_and_dropoff[scenario].items():
                    if req == self.curr_request:
                        start[var] = 0

            model.remove(model.getConstrByName('Fix decision'))
            model.addConstr(self.y[scenario][self.curr_request] <= 0, name=f'Fix decision')
            if start:
                model.setAttr('Start', list(start), list(start.values()))
            switch_times.append(time.time() - t)
        return switch_times

    def get_objectives_fix_decision(self):
        # Check that all scenarios are feasible (we can break after the first scenario because of theorem 3.1)
        for scenario in self.scenarios:
            # print(f'Model status {self.models[scenario].status}')
            if self.models[scenario].status == 3:
                return -math.inf
            # No solution found within the time limit: treat like infeasibility
            if self.models[scenario].SolCount == 0:
                print(f'No solution for scenario {scenario} (status {self.models[scenario].status}), treated as infeasible')
                return -math.inf

        return sum(self.models[scenario].ObjVal for scenario in self.scenarios)
