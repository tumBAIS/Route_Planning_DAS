import run_decision_decomp_exact
import run_decision_decomp_heuristic
import run_decision_decomp_greedy

import sys
import random
import os

if __name__ == "__main__":
    route = sys.argv[1]  # 55 or 155
    route_time_text = sys.argv[2] # for 55: '18_56_00' or '13_50_00'; for 155: '13_50_00' or '18_56_00'
    comp_perc = sys.argv[3] # 0 - 1
    num_scenarios = int(sys.argv[4]) + 1 # 0 - 40
    walking_distance = int(sys.argv[5]) # [100, 150, 250, 300, 350, 400]
    util_value = int(sys.argv[6]) # 750
    algorithm = sys.argv[7] # exact, heuristic, or greedy
    seed = int(sys.argv[8]) # integer

    # route = 55  # 55 or 155
    # route_time_text = '18_56_00' # for 55: '18_56_00' or '13_50_00'; for 155: '13_50_00' or '18_56_00'
    # comp_perc = 1 # 0 - 1
    # num_scenarios = 2 # 0 - 40
    # walking_distance = 250 # [100, 150, 250, 300, 350, 400]
    # util_value = 750 # 750
    # algorithm = 'greedy' # exact, heuristic, or greedy
    # seed = 0 # integer

    # Pull num_scenarios random scenarios out of our sample set of 50 scenarios with a fixed seed
    random.seed(seed)
    scenario_ids = random.sample(range(50), num_scenarios)

    # Get the corresponding scenarios to the scenario_ids
    scenario_dict = os.path.dirname(
        os.path.realpath(__file__)) + '/generators/scenarios'
    json_files = [scenario_dict + f'/{route}_{route_time_text}_{comp_perc}_200_200_{seed}.json' for seed in
                  scenario_ids]

    print(json_files)

    if algorithm == 'exact':

        # Run Stochastic Programs in a rolling horizon algorithm
        run_decision_decomp_exact.run_decomp(json_files, walking_distance, util_value, seed)

    elif algorithm == 'heuristic':
        # Run Stochastic Programs in a rolling horizon algorithm
        run_decision_decomp_heuristic.run_decomp(json_files, walking_distance, util_value, seed)

    elif algorithm == 'greedy':
        # Run Stochastic Programs in a rolling horizon algorithm
        run_decision_decomp_greedy.run_decomp(json_files, walking_distance, util_value, seed)