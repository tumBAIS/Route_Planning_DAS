import os

import generators.generate_multi_scenario as generate
from solver.scenario_decision_decomp import ScenarioDecisionDecomp
from solver.deterministic_arc_based import DeterministicModel
import csv

import time
import numpy as np


def save_data(filename, data):
    columns = ['route', 'route_time', 'comp_stops', 'comp_stops_perc', 'utility', 'walking_distance',
               'num_scenarios', 'num_features',
               'build_time', 'solve_time', 'upper_bound', 'lower_bound', 'opt_gap']

    if not os.path.isfile(filename):
        with open(filename, 'w', newline='') as csv_file:
            # Create a CSV writer
            csv_writer = csv.writer(csv_file)

            # Write the header row with column names
            csv_writer.writerow(columns)

    with open(filename, 'a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(data)
    return

def update_scenarios(curr_req, accepted_req, prev_scenarios, curr_scenario):
    ''' Remove all requests from the scenarios which occured before current request, fix all previously accepted
    requests, and add current request to all scenarios'''

    for scenario in prev_scenarios.requests:
        for req in list(prev_scenarios.requests[scenario]):
            # Remove all requests which occurred before the current request time
            if req.request_time < curr_req.request_time and req not in accepted_req:
                prev_scenarios.requests[scenario].remove(req)

        # Add all already accepted request to each scenario
        for req in accepted_req:
            if req not in prev_scenarios.requests[scenario]:
                prev_scenarios.requests[scenario].append(req)

        # Add current request to all scenarios
        prev_scenarios.requests[scenario].append(curr_req)

    for scenario in prev_scenarios.deltas:
        # Add all already accepted request to each scenario
        for req in accepted_req:
            if req not in prev_scenarios.deltas[scenario]:
                prev_scenarios.deltas[scenario][req] = curr_scenario.deltas[0][req]

        # Add current request to all scenarios
        prev_scenarios.deltas[scenario][curr_req] = curr_scenario.deltas[0][curr_req]

    return prev_scenarios

def run_decomp(json_files, walking_distance, util_value, seed, bus_capacity=145):
    request_file = json_files.pop(1)
    curr_scenario = generate.get_instance([request_file], walking_distance, 1)
    prev_scenarios = generate.get_instance(json_files, walking_distance, 1)

    prev_scenarios.set_utilities(util_value)
    curr_scenario.set_utilities(util_value)

    # Sort the request of the current scenario by their request time
    curr_scenario.requests[0].sort(key=lambda x: x.request_time)
    for scenario in prev_scenarios.requests:
        prev_scenarios.requests[scenario].sort(key=lambda x: x.request_time)

    # {request: (pickup_stop, dropoff_stop)}
    accepted_requests = dict()

    build_times = []
    solve_times = []
    decision_times = []

    # For each request in the current scenario, solve the corresponding stochastic program
    i = 0
    print('----------------------- Solver information -----------------------')
    run_start = time.time()
    for curr_req in curr_scenario.requests[0]:
        print(f'Determine whether to accept or reject request {i}')
        i += 1
        start_time = time.time()
        update_scenarios(curr_req, accepted_requests, prev_scenarios, curr_scenario)

        model = ScenarioDecisionDecomp(prev_scenarios.network,
                                 prev_scenarios.compulsory_stops,
                                 prev_scenarios.scenarios,
                                 list(accepted_requests),
                                 prev_scenarios.requests,
                                 curr_req,
                                 prev_scenarios.deltas,
                                 prev_scenarios.time_windows,
                                 bus_capacity,
                                 accepted_stops=accepted_requests)


        build_times_scenarios = model.build_models()
        build_times += build_times_scenarios


        solve_times_scenarios = model.solve(True)
        solve_times += solve_times_scenarios

        # If request is accepted, store it together with its assigned pickup and dropoff stops
        new_accepted_requests = model.get_accepted_requests()
        if len(new_accepted_requests) > len(accepted_requests):
            accepted_requests[curr_req] = model.get_stop_assignment(curr_req)
            print('--- Accepted request ---')
        else:
            print('--- Rejected request ---')

        decision_times.append(time.time() - start_time)


    ############################ Get optimal route with all accepted requests ############################
    opt_route = DeterministicModel(curr_scenario.network,
                                   curr_scenario.compulsory_stops,
                                   list(accepted_requests),
                                   {req: req.utility for req in accepted_requests},
                                   curr_scenario.deltas[0],
                                   curr_scenario.time_windows,
                                   bus_capacity)
    # Build Gurobi Model
    opt_route.build_model()

    # Accept all accepted requests in Gurobi Model at their assigned stops
    opt_route.force_request_accept(list(accepted_requests), fixed_stops=accepted_requests)

    # Solve opt_route
    opt_route.solve()


    ############################ Get optimal deterministic solution ############################
    det_model = DeterministicModel(curr_scenario.network,
                                   curr_scenario.compulsory_stops,
                                   curr_scenario.requests[0],
                                   {req: req.utility for req in curr_scenario.requests[0]},
                                   curr_scenario.deltas[0],
                                   curr_scenario.time_windows,
                                   bus_capacity)

    # Build Gurobi Model
    det_model.build_model()

    # Solve deterministic model
    det_model.solve()

    # Get all accepted requests in the deterministic model
    accepted_requests_deterministic = det_model.get_served_requests()

    ########################### Save data from online and offline model ###########################
    print('----------------------- Solution information -----------------------')
    total_objective_online = opt_route.m.objVal
    total_objective_offline = det_model.m.objVal

    def get_visited_stops(model):
        next_stop = {i: j for (i, j), var in model.x.items() if var.X > 0.5}
        stop = model.compulsory_stops[0]
        visited = []
        while stop is not None:
            visited.append(str(stop.id))
            stop = next_stop.get(stop)
        return ';'.join(visited)

    visited_stops_online = get_visited_stops(opt_route)
    visited_stops_offline = get_visited_stops(det_model)

    travel_time_online = opt_route.t[opt_route.compulsory_stops[-1]].X
    travel_time_offline = det_model.t[det_model.compulsory_stops[-1]].X

    def get_walking_distances(model):
        # Per served request: origin building to pickup stop plus dropoff stop to destination building
        distances = curr_scenario.data['buildings_distance_matrix']
        walking = {req: 0 for req in model.get_served_requests()}
        for (req, stop), var in model.p.items():
            if req in walking and var.X > 0.5:
                walking[req] += distances[req.origin['trip_id'] * 2][stop.order]
        for (req, stop), var in model.o.items():
            if req in walking and var.X > 0.5:
                walking[req] += distances[req.origin['trip_id'] * 2 + 1][stop.order]
        return [round(dist, 2) for dist in walking.values()]

    walking_distances_online = get_walking_distances(opt_route)
    walking_distances_offline = get_walking_distances(det_model)

    def get_stop_assignment(model):
        # Per served request: (trip_id, pickup stop, dropoff stop)
        assignment = {req: [req.origin['trip_id'], None, None] for req in model.get_served_requests()}
        for (req, stop), var in model.p.items():
            if req in assignment and var.X > 0.5:
                assignment[req][1] = stop.id
        for (req, stop), var in model.o.items():
            if req in assignment and var.X > 0.5:
                assignment[req][2] = stop.id
        return [tuple(a) for a in assignment.values()]

    def get_loads(model):
        # Passengers on board per driven segment, loads[i] between visited stops i and i + 1
        next_stop = {i: j for (i, j), var in model.x.items() if var.X > 0.5}
        stop = model.compulsory_stops[0]
        loads = []
        while stop in next_stop:
            loads.append(round(model.k[(stop, next_stop[stop])].X))
            stop = next_stop[stop]
        return loads

    stop_assignment_online = get_stop_assignment(opt_route)
    stop_assignment_offline = get_stop_assignment(det_model)
    loads_online = get_loads(opt_route)
    loads_offline = get_loads(det_model)

    avg_walking_distance_online = round(np.mean(walking_distances_online), 2) if walking_distances_online else np.nan
    avg_walking_distance_offline = round(np.mean(walking_distances_offline), 2) if walking_distances_offline else np.nan

    # The generator already removed all requests without a stop within walking distance
    num_requests = len(curr_scenario.requests[0])
    num_solved = len(accepted_requests)
    total_run_time = round(time.time() - run_start, 2)

    print("----------------------------------------------------------------------")
    print(f'Number of solved requests: {num_solved}')
    print(f'Number of rejected requests: {num_requests - num_solved}')
    print(f'Number of solved requests in deterministic model: {len(accepted_requests_deterministic)}')
    print(f'Number of rejected requests in deterministic model: {num_requests - len(accepted_requests_deterministic)}')
    print(f'Number of feasible requests: {num_requests}')
    print(f'Average decision time: {round(np.nanmean(decision_times), 2)} seconds')
    print(f'Min Decision Time: {min(decision_times)}, max decision time: {max(decision_times)}')
    print(f'Average build time: {round(np.nanmean(build_times), 2)} seconds')
    print(f'Average solve time: {round(np.nanmean(solve_times), 2)} seconds')
    print(f'Total run time: {total_run_time} seconds')
    print(f'Total objective online: {total_objective_online}')
    print(f'Total objective offline: {total_objective_offline}')
    print(f'Travel time online: {travel_time_online}')
    print(f'Travel time offline: {travel_time_offline}')
    print(f'Average walking distance online: {avg_walking_distance_online}')
    print(f'Average walking distance offline: {avg_walking_distance_offline}')

    data_head = ['feasible_requests', 'num_solved_requests_online', 'num_solved_requests_offline', 'avg_decision_time', 'min_decision_time', 'max_decision_time',
                 'avg_build_time', 'avg_solve_time',
                 'total_run_time', 'total_objective_online', 'total_objective_offline',
                 'travel_time_online', 'travel_time_offline',
                 'avg_walking_distance_online', 'avg_walking_distance_offline',
                 'visited_stops_online', 'visited_stops_offline',
                 # Per subproblem in solve order, combine_csv.py derives the serial and parallel time per request
                 'build_times', 'solve_times',
                 # Per served request in request order
                 'walking_distances_online', 'walking_distances_offline',
                 'stop_assignment_online', 'stop_assignment_offline',
                 # Per segment of visited_stops
                 'loads_online', 'loads_offline']
    data = [num_requests, num_solved, len(accepted_requests_deterministic), round(np.nanmean(decision_times), 2), min(decision_times), max(decision_times),
            round(np.nanmean(build_times), 2), round(np.nanmean(solve_times), 2),
            total_run_time, total_objective_online, total_objective_offline,
            travel_time_online, travel_time_offline,
            avg_walking_distance_online, avg_walking_distance_offline,
            visited_stops_online, visited_stops_offline,
            build_times, solve_times,
            walking_distances_online, walking_distances_offline,
            stop_assignment_online, stop_assignment_offline,
            loads_online, loads_offline]

    # Save data to csv file
    route = curr_scenario.data['route_name']
    route_time = curr_scenario.data['parameters']['erste_abzeit_text']
    route_time_text = route_time.replace(':', '_')
    comp_perc = curr_scenario.data['parameters']['compulsory_percentage']
    num_scenarios = len(json_files)

    # Group results by algorithm, walking distance, and compulsory stop percentage
    result_folder = os.path.join('results', 'heuristic', str(walking_distance), str(comp_perc))
    os.makedirs(result_folder, exist_ok=True)

    #   online_algorithm_<method>_<route>_<departure>_<csf>_walk<wd>_n<N>[_cap<C>]_<seed>_summary.csv
    model_tag = f'walk{walking_distance}_n{num_scenarios}'
    # Only tag non-default capacities, so default runs keep their file names
    if bus_capacity != 145:
        model_tag += f'_cap{bus_capacity}'

    file_path = (result_folder + '/online_algorithm_TS_heuristic_decomp_' +
                 f'{route}_{route_time_text}_{comp_perc}_{model_tag}_{seed}_summary.csv')
    with open(file_path, 'w') as f:
        writer = csv.writer(f)
        writer.writerows([data_head, data])
    return