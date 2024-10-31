import os

import generators.generate_multi_scenario as generate
from solver.scenario_decision_decomp import ScenarioDecisionDecomp
from solver.deterministic_arc_based import DeterministicModel
import csv
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

def run_decomp(json_files, walking_distance, util_value, seed):
    request_file = json_files.pop(1)
    curr_scenario = generate.get_instance([request_file], walking_distance, 1)
    prev_scenarios = generate.get_instance(json_files, walking_distance, 1)

    prev_scenarios.set_utilities(util_value)
    curr_scenario.set_utilities(util_value)

    # Sort the request of the current scenario by their request time
    curr_scenario.requests[0].sort(key=lambda x: x.request_time)
    for scenario in prev_scenarios.requests:
        prev_scenarios.requests[scenario].sort(key=lambda x: x.request_time)

    accepted_requests = list()

    build_times = []
    solve_times = []

    # For each request in the current scenario, solve the corresponding stochastic program
    i = 0
    print('----------------------- Solver information -----------------------')
    for curr_req in curr_scenario.requests[0]:
        print(f'Determine whether to accept or reject request {i}')
        i += 1

        # features = list([feature.keys() for feature in instance.features[0].values()][0])
        model = ScenarioDecisionDecomp(curr_scenario.network,
                                 curr_scenario.compulsory_stops,
                                 curr_scenario.scenarios,
                                 accepted_requests,
                                {0: accepted_requests + [curr_req]},
                                 curr_req,
                                 curr_scenario.deltas,
                                 curr_scenario.time_windows)

        build_times_scenarios = model.build_models()
        build_times += build_times_scenarios


        solve_times_scenarios = model.solve(True)
        solve_times += solve_times_scenarios

        # If request is accepted, add it to list of accepted requests
        new_accepted_requests = model.get_accepted_requests()
        if len(new_accepted_requests) > len(accepted_requests):
            accepted_requests = new_accepted_requests
            print('--- Accepted request ---')
        else:
            print('--- Rejected request ---')


    ############################ Get optimal route with all accepted requests ############################
    opt_route = DeterministicModel(curr_scenario.network,
                                   curr_scenario.compulsory_stops,
                                   accepted_requests,
                                   {req: req.utility for req in accepted_requests},
                                   curr_scenario.deltas[0],
                                   curr_scenario.time_windows)
    # Build Gurobi Model
    opt_route.build_model()

    # Accept all accepted requests in Gurobi Model
    opt_route.force_request_accept(accepted_requests)

    # Solve opt_route
    opt_route.solve()


    ############################ Get optimal deterministic solution ############################
    det_model = DeterministicModel(curr_scenario.network,
                                   curr_scenario.compulsory_stops,
                                   curr_scenario.requests[0],
                                   {req: req.utility for req in curr_scenario.requests[0]},
                                   curr_scenario.deltas[0],
                                   curr_scenario.time_windows)

    # Build Gurobi Model
    det_model.build_model()

    # Solve deterministic model
    det_model.solve()

    # Get all accepted requests in the deterministic model
    accepted_requests_deterministic = det_model.get_served_requests()

    ########################### Save data from online and offline model ###########################

    # Determine parameters
    route = curr_scenario.data['route_name']
    route_time = curr_scenario.data['parameters']['erste_abzeit_text']
    route_time_text = route_time.replace(':', '_')
    comp_stops = len(curr_scenario.compulsory_stops)
    comp_perc = curr_scenario.data['parameters']['compulsory_percentage']
    num_scenarios = len(json_files)
    num_requests = len(curr_scenario.requests[0])

    # Calculate routing costs and serving profits
    serving_profit = sum([opt_route.y[req].X * req.utility for req in accepted_requests])
    serving_profit_deterministic = sum([det_model.y[req].X * req.utility for req in accepted_requests_deterministic])
    routing_online = opt_route.m.objVal - serving_profit
    routing_offline = det_model.m.objVal - serving_profit_deterministic

    # Caluclate travel times
    travel_time = opt_route.t[opt_route.compulsory_stops[-1]].X
    travel_time_deterministic = det_model.t[det_model.compulsory_stops[-1]].X

    data_head = ['route', 'route_time', 'comp_stops', 'comp_stops_perc', 'utility', 'walking_distance', 'num_scenarios',
                 'num_requests', 'served_requests', 'served_requests_deterministic', 'objective', 'objective_deterministic',
                 'solve_times', 'build_times', 'routing_cost', 'routing_cost_deterministic', 'serving_profit',
                 'serving_profit_deterministic', 'travel_time', 'travel_time_deterministic']

    data = [route, route_time, comp_stops, comp_perc, util_value, walking_distance, num_scenarios, num_requests,
            len(accepted_requests), len(accepted_requests_deterministic), opt_route.m.objVal, det_model.m.objVal,
            solve_times, build_times, routing_online, routing_offline, serving_profit, serving_profit_deterministic,
            travel_time, travel_time_deterministic]



    with open(f'results/online_algorithm_greedy_{route}_{route_time_text}_{comp_perc}_{num_scenarios}_{seed}.csv',
              'w') as f:
        writer = csv.writer(f)
        writer.writerows([data_head, data])

    # Calculate serial computation times
    serial_solve_times = []
    for i in range(0, len(solve_times), 1):
        serial_solve_times.append(sum(solve_times[i:i + 1]))


    # Print relevant information
    print('----------------------- Solution information -----------------------')
    print(f'Accepted requests: {len(accepted_requests)}')
    print(f'Rejected requests: {num_requests - len(accepted_requests)}')
    print(f'Revenue: {round(opt_route.m.objVal, 2)}')
    print(f'Average request response time (serial): {round(np.nanmean(serial_solve_times), 2)} seconds')
    print(f'Average request response time (parallel): {round(np.nanmean(solve_times), 2)} seconds')
    print(f'Routing cost: {round(routing_online, 2)}')
    print(f'Travel time: {round(travel_time, 2)}')

    return
