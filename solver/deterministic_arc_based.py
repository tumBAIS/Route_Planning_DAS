import gurobipy as grp
from gurobipy import GRB
from datetime import datetime, timedelta

# from solver.scenario_decision_decomp_revision import combined_lazy_cb


def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
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

class DeterministicModel:
    def __init__(self, network, compulsory_stops, requests, utility, delta, time_windows, bus_capacity):
        self.network = network
        self.compulsory_stops = compulsory_stops

        self.requests = requests
        self.utility = utility
        self.delta = delta  # {request: {'origin': {stop1: 0, stop2: 1}}, {'destination': {stop1: 1, stop2: 0}}}
        self.time_windows = time_windows
        self.bus_capacity = bus_capacity

        # Model
        self.m = None

        # Variables
        self.x = None
        self.y = None
        self.t = None
        self.k = None
        self.p = None
        self.o = None
        self.u = None
        self.pickup_and_dropoff = None

        # Constraints
        self.stop_out = None
        self.stop_in = None
        self.request_p = None
        self.request_o = None
        self.flow_conservation = None
        self.flow_conservation_origin = None
        self.flow_conservation_destination = None
        self.time_compulsory = None
        self.time_window_a = None
        self.time_window_b = None
        self.out_flow = None
        self.capacity_limit = None
        self.picked_up_req = None
        self.dropped_off_req = None
        self.capacity_propagation = None
        self.initial_capacity = None
        self.invalid_pairs = None
        self.order_propagation = None
        self.pickup_and_dropoff_and = None
        self.pickup_before_dropoff = None

    def get_segment_edges(self, segment):
        return [(i, j) for (i, j) in self.network.edges() if segment in self.network[i][j]['segment']]

    def get_segment_nodes(self, segment):
        return [stop for stop in self.network.nodes() if segment in stop.segment]

    def get_stop_interval(self, stop):
        if stop.is_compulsory:
            pos = self.compulsory_stops.index(stop)
            return pos, pos
        return min(stop.segment) - 1, max(stop.segment)

    def build_model(self):
        self.m = grp.Model('Deterministic')

        # Variables
        self.x = {(i, j): self.m.addVar(vtype=GRB.BINARY, name=f'x_{(i, j)}', obj=-self.network[i][j]['weight'])
                  for (i, j) in self.network.edges()}
        self.y = {req: self.m.addVar(vtype=GRB.BINARY, obj=self.utility[req])
                  for req in self.requests}

        self.t = {stop: self.m.addVar(vtype=GRB.CONTINUOUS, name=f't_{stop}') for stop in self.compulsory_stops}

        self.k = {(i, j): self.m.addVar(vtype=GRB.CONTINUOUS, name=f'k_{(i, j)}') for (i, j) in self.network.edges()}

        self.p = {(req, i): self.m.addVar(vtype=GRB.BINARY, name=f'pickup_{(req, i)}')
                       for req in self.requests
                       for i in self.delta[req]['origin']}

        self.o = {(req, j): self.m.addVar(vtype=GRB.BINARY, name=f'dropoff_{(req, j)}')
                        for req in self.requests
                        for j in self.delta[req]['destination']}

        # Constraints
        self.stop_out = {(req, stop): self.m.addConstr(self.p[(req, stop)] <=
                                               grp.quicksum(self.x[arc]
                                                            for arc in self.network.out_edges(stop)
                                                            ))
                         for req in self.requests
                         for stop in self.delta[req]['origin']}

        self.stop_in = {(req, stop): self.m.addConstr(self.o[(req, stop)] <=
                                              grp.quicksum(self.x[arc]
                                                           for arc in self.network.in_edges(stop)
                                                           ))
                        for req in self.requests
                        for stop in self.delta[req]['destination']}

        self.request_p = {req: self.m.addConstr(grp.quicksum(self.p[(req, origin)]
                                                            for origin in self.delta[req]['origin'])
                                                     == self.y[req])
                        for req in self.requests}

        self.request_o = {req: self.m.addConstr(grp.quicksum(self.o[(req, dest)]
                                                             for dest in self.delta[req]['destination'])
                                                == self.y[req])
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

        self.capacity_limit = {
            (i, j): self.m.addConstr(self.k[(i, j)] <= self.bus_capacity * self.x[(i, j)],
                name=f'Capa_limit_{(i, j)}')
            for (i, j) in self.network.edges()}

        self.capacity_propagation = {
            stop: self.m.addConstr(
                grp.quicksum(self.k[(stop, dest)] for (stop, dest) in self.network.out_edges(stop)) ==
                grp.quicksum(self.k[(origin, stop)] for (origin, stop) in self.network.in_edges(stop)) +
                grp.quicksum(self.p[(req, stop)] for req in self.requests if (req, stop) in self.p) -
                grp.quicksum(self.o[(req, stop)] for req in self.requests if (req, stop) in self.o),
                name=f'Capa_prop_{stop}')
            for stop in self.network.nodes() if stop not in (self.compulsory_stops[0], self.compulsory_stops[-1])}

        # bus leaves the origin carrying exactly the passengers picked up there
        origin = self.compulsory_stops[0]
        self.initial_capacity = self.m.addConstr(
            grp.quicksum(self.k[arc] for arc in self.network.out_edges(origin)) ==
            grp.quicksum(self.p[(req, origin)] for req in self.requests if (req, origin) in self.p),
            name='Initial_capacity')

        self.invalid_pairs = dict()

        ordering_pairs = list()
        affected_segments = set()

        for req in self.requests:
            for seg_p in self.delta[req]['origin']:
                low_p, high_p = self.get_stop_interval(seg_p)
                for seg_o in self.delta[req]['destination']:
                    low_o, high_o = self.get_stop_interval(seg_o)
                    if seg_p == seg_o or high_o <= low_p:
                        self.invalid_pairs[(req, seg_p, seg_o)] = self.m.addConstr(
                            self.p[(req, seg_p)] + self.o[(req, seg_o)] <= 1, name=f'invalid_pair_{(req, seg_p, seg_o)}')
                    elif high_p <= low_o:
                        continue
                    else:
                        ordering_pairs.append((req, seg_p, seg_o))
                        affected_segments.update(set(seg_p.segment) & set(seg_o.segment))

        if not ordering_pairs:
            self.u = dict()
            self.order_propagation = dict()
            self.pickup_and_dropoff = dict()
            self.pickup_and_dropoff_and = dict()
            self.pickup_before_dropoff = dict()
            return    

        order_arcs = set()
        order_nodes = set()

        for seg in affected_segments:
            seg_arcs = self.get_segment_edges(seg)
            order_arcs.update(seg_arcs)
            order_nodes.update(node for arc in seg_arcs for node in arc)
        for (req, seg_p, seg_o) in ordering_pairs:
            order_nodes.update((seg_p, seg_o))

        # order of visits
        self.u = {stop: self.m.addVar(vtype=GRB.CONTINUOUS, ub=len(order_nodes), name=f'u_{stop}') for stop in order_nodes}

        self.order_propagation = {
            (i, j): self.m.addGenConstrIndicator(self.x[(i, j)], True, self.u[j] - self.u[i] >= 1, name=f'Order_prop_{(i, j)}')
            for (i, j) in order_arcs
        }

        #activator for GenConstrIndicator for pickup and dropoff
        self.pickup_and_dropoff = {(req, seg_p, seg_o): self.m.addVar(vtype=GRB.BINARY, name=f'pickup_dropoff_{(req, seg_p, seg_o)}') for (req, seg_p, seg_o) in ordering_pairs}

        self.pickup_and_dropoff_and = {
            (req, seg_p, seg_o): self.m.addGenConstrAnd(
                self.pickup_and_dropoff[(req, seg_p, seg_o)],
                [self.p[(req, seg_p)], self.o[(req, seg_o)]],
                name=f'pickup_dropoff_and_{(req, seg_p, seg_o)}'
            )
            for (req, seg_p, seg_o) in ordering_pairs
        }

        self.pickup_before_dropoff = {
            (req, seg_p, seg_o): self.m.addGenConstrIndicator(
                self.pickup_and_dropoff[(req, seg_p, seg_o)], True, self.u[seg_o] - self.u[seg_p] >= 1,
                name=f'precedence_{(req, seg_p, seg_o)}')
            for (req, seg_p, seg_o) in ordering_pairs
        }



    def solve(self):
        # Set objective to maximization
        self.m.Params.OutputFlag = 0
        self.m.ModelSense = -1
        self.m.Params.TimeLimit = 10*60
        self.m.Params.MIPGap = 0.01

        # Define attributes needed for subtour elimination
        self.m._vars = self.x
        self.m._out_arcs = {stop: list(self.network.out_edges(stop)) for stop in self.network.nodes()}
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

    # force already accepted requests to be accepted and picked up/dropped off at defined stops
    def force_request_accept(self, requests, fixed_stops=None):
        for req in requests:
            self.m.addConstr(self.y[req] >= 1)
            if fixed_stops and fixed_stops.get(req):
                pick, drop = fixed_stops[req]
                self.m.addConstr(self.p[(req, pick)] >= 1)
                self.m.addConstr(self.o[(req, drop)] >= 1)

    def force_fixed_route(self):
        for a, b in zip(self.compulsory_stops, self.compulsory_stops[1:]):
            self.m.addConstr(self.x[(a, b)] == 1)
        return
