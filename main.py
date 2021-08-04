import numpy as np
import pandas as pd
import yaml
import csv
import timeit
import time

from model.optimize import Optimizer
from OR_tools_own import ORtool

# Load problem parameters in a dictionary
def read_data():
    with open(r'data/parameters.yaml') as datafile:
        params = yaml.full_load(datafile)
    return params

# Read parameters
params = read_data()

# Create Optimizer class - Class that executes all calculations and takes care of the optimization problem
Optimizer = Optimizer(params)

# Obtain gps, time and demand data
gps_coordinates, time_windows, demand, service_time = Optimizer.data_preparation()
#print(service_time)
#print(demand)
#print("Total demand in L:", demand.sum(axis=0))

# time_window_df = pd.DataFrame(time_windows)
# time_window_df.to_csv('time_window.csv',index = True, header=True, sep = ';')

# Obtain euclidean distance matrix between pickup points
D, T = Optimizer.gps_to_distances(gps_coordinates)

# T_df = pd.DataFrame(T)
# T_df.to_csv('T.csv',index = False, header=True, sep = ';')
#
# service_df = pd.DataFrame(service_time)
# service_df.to_csv('service.csv',index = False, header=True, sep = ';')

T = Optimizer.add_service_times_to_T(T,service_time)
# T_df = pd.DataFrame(T)
# T_df.to_csv('T_with_service.csv',index = False, header=True, sep = ';')




# # Use Google's ORtool to create route optimized for total driving distance
ORtool = ORtool()
start_time = time.perf_counter()
total_route, total_schedule, total_schedule_max = ORtool.main(T,D,demand,service_time,time_windows)
runtime = time.perf_counter() - start_time
print("Total route:",total_route)
print("Total schedule:",total_schedule)

distance = 0
gps_locations = []

route = total_route[1]
schedule = total_schedule[1]
schedule_max = total_schedule_max[1]
#schedule = Optimizer.create_schedule_from_route(route, D, service_time)
objective = Optimizer.get_objective_distance(D, route)
gps_locations_route = np.zeros((len(route),2))
for i in range(len(route)):
    gps_locations_route[i] = gps_coordinates[route[i]]

gps_locations.append(gps_locations_route)
print("Route:",route)
#print("Schedule:",schedule)
print("Travelled distance:",objective)
distance += objective

print("Total travelled distance in km:",distance)

print("Schedule:",schedule)

# Add stopover to not exceed total capacity
solution, schedule, schedule_max = Optimizer.add_stopover(route,demand,D,T,schedule, schedule_max)
schedule, route_possible = Optimizer.create_schedule_from_route(solution,T,time_windows)
print("Final route:",solution)
print("Schedule:",schedule)

# schedule = Optimizer.add_service_times(solution,schedule,service_time)
# print("Schedule:",schedule)

schedule = Optimizer.from_schedule_to_time(schedule)
schedule_max = Optimizer.from_schedule_to_time(schedule_max)
print("Schedule",schedule)
print("Schedule max",schedule_max)

gps_locations = []

route = solution
#schedule = Optimizer.create_schedule_from_route(route, D, service_time)
objective = Optimizer.get_objective_distance(D, route)
gps_locations_route = np.zeros((len(route),2))
for i in range(len(route)):
    gps_locations_route[i] = gps_coordinates[solution[i]]

gps_locations.append(gps_locations_route)
print("Route:",route)
print("Travelled distance:",objective)
print("Found solution in:",runtime,"seconds")
distance += objective

# Visualize locations and route
Optimizer.visualize_locations(gps_coordinates,gps_locations,title="ORtool optimized for distance")

# route = pd.DataFrame(route)
# schedule = pd.DataFrame(schedule)
# schedule_max = pd.DataFrame(schedule_max)

# # saving the dataframes optimized for distance
# route.to_csv('route_distance.csv',index = False)
# schedule.to_csv('schedule_distance.csv',index = False)
# schedule_max.to_csv('schedule_max_distance.csv',index = False)


#
# # # Use Google's ORtool to create route optimized for total driving duration time
# start_time = time.perf_counter()
# ORtool = ORtool()
# total_route, total_schedule, total_schedule_max = ORtool.main(T,D,demand,service_time,time_windows)
# runtime = time.perf_counter() - start_time
#
# print("Total route:",total_route)
# print("Total schedule:",total_schedule)
#
# distance = 0
# gps_locations = []
#
# route = total_route[1]
# schedule = total_schedule[1]
# schedule_max = total_schedule_max[1]
# #schedule = Optimizer.create_schedule_from_route(route, D, service_time)
# objective = Optimizer.get_objective_distance(D, route)
# gps_locations_route = np.zeros((len(route),2))
# for i in range(len(route)):
#     gps_locations_route[i] = gps_coordinates[route[i]]
#
# gps_locations.append(gps_locations_route)
# print("Route:",route)
# #print("Schedule:",schedule)
# print("Travelled distance:",objective)
# distance += objective
#
# print("Total travelled distance in km:",distance)
#
# print("Schedule:",schedule)
#
# # Add stopover to not exceed total capacity
# solution, schedule, schedule_max = Optimizer.add_stopover(route,demand,D,T,schedule, schedule_max)
# schedule, route_possible = Optimizer.create_schedule_from_route(solution,T,time_windows)
# print("Final route:",solution)
# print("Schedule:",schedule)
#
# # schedule = Optimizer.add_service_times(solution,schedule,service_time)
# # print("Schedule:",schedule)
#
# schedule = Optimizer.from_schedule_to_time(schedule)
# schedule_max = Optimizer.from_schedule_to_time(schedule_max)
# print("Schedule",schedule)
# #print("Schedule max",schedule_max)
#
# gps_locations = []
#
# route = solution
# #schedule = Optimizer.create_schedule_from_route(route, D, service_time)
# objective = Optimizer.get_objective_distance(D, route)
# gps_locations_route = np.zeros((len(route),2))
# for i in range(len(route)):
#     gps_locations_route[i] = gps_coordinates[solution[i]]
#
# gps_locations.append(gps_locations_route)
# print("Route:",route)
# print("Travelled distance:",objective)
# print("Found solution in:",runtime,"seconds")
# distance += objective
#
# # Visualize locations and route
# Optimizer.visualize_locations(gps_coordinates,gps_locations,title="ORtool optimized for driving duration")
#
# # route = pd.DataFrame(route)
# # schedule = pd.DataFrame(schedule)
# # schedule_max = pd.DataFrame(schedule_max)
#
# # # saving the dataframes optimized for time
# # route.to_csv('route_time.csv',index = False)
# # schedule.to_csv('schedule_time.csv',index = False)






#
# # # Use own Savings algorithm to find optimal route when not considering time windows
#
# # Initial solution
# initial_solution = np.arange(0,len(gps_coordinates),1)
# print(initial_solution)
# objective = Optimizer.get_objective_distance(D, initial_solution)
# print(objective)
#
# # Initial route for savings algorithm
# initial_distance = Optimizer.get_initial_distance(D)
# print(initial_distance)
#
# # Use savings algorithm for optimal route
# start_time = time.perf_counter()
# solution, time1, distance = Optimizer.sequential_savings_init(D, initial_distance)
# runtime = time.perf_counter() - start_time
# objective = Optimizer.get_objective_distance(D, solution)
#
# print(solution)
# print(distance)
# print("Travelled distance without capacity constraint:", objective)
#
# # Add stopover to not exceed total capacity
# solution = Optimizer.add_stopover_no_constraints(solution,demand,D)
# objective = Optimizer.get_objective_distance(D, solution)
# print("Travelled distance with capacity constraint:", objective)
# print("Found solution in:",runtime,"seconds")
#
# # Create schedule for final route
# schedule, route_possible = Optimizer.create_schedule_from_route(solution,T,time_windows)
# print("Schedule:",schedule)
# schedule = Optimizer.from_schedule_to_time(schedule)
# print("Final route:",solution)
# print("Final schedule:", schedule)
#
# gps_locations = []
# gps_locations_route = np.zeros((len(solution),2))
# for i in range(len(solution)):
#     gps_locations_route[i] = gps_coordinates[solution[i]]
#
# gps_locations.append(gps_locations_route)
#
# # Visualize locations and route
# Optimizer.visualize_locations(gps_coordinates,gps_locations,title="Own Savings algorithm")
#
# # route = pd.DataFrame(solution)
# # schedule = pd.DataFrame(schedule)
#
# # # saving the dataframes optimized for distance
# # route.to_csv('route_own.csv',index = False)
# # schedule.to_csv('schedule_own.csv',index = False)
#
# ## Optimize for driving duration by Local Search
# start_time = time.perf_counter()
# route_short, schedule_short = Optimizer.local_search(solution,T,time_windows)
# runtime = time.perf_counter() - start_time
# schedule_short = Optimizer.from_schedule_to_time(schedule_short)
# print("Route optimized for duration:",route_short)
# print("Schedule optimized for duration:",schedule_short)
# objective = Optimizer.get_objective_distance(D, route_short)
# print("Travelled distance for shortest duration:", objective)
# print("Found solution in:",runtime,"seconds")
#
# # route = pd.DataFrame(route_short)
# # schedule = pd.DataFrame(schedule_short)
#
# # # saving the dataframes optimized for time
# # route.to_csv('route_own_short.csv',index = False)
# # schedule.to_csv('schedule_own_short.csv',index = False)









# # # Find the available locations
# # available_locations = Optimizer.select_available_locations(service_time,time_windows)
# # print(available_locations)
#
# # Create schedule from route
# print(service_time)
# schedule = Optimizer.create_schedule_from_route(solution,T,service_time)
#
# print("Schedule that does not work yet:",schedule)
#
# # Make schedule fit the time windows
# schedule, route = Optimizer.make_schedule_possible(schedule,solution,time_windows)
# # print("Schedule that works for every location:", schedule)
# # print("Solution:", solution)
#
# print("Schedule that works for every location:", schedule)
#
# # Add stopover to not exceed total capacity
# solution = Optimizer.add_stopover(route,demand)
# print(solution)
#
# objective = Optimizer.get_objective_distance(D, solution)
# print("Travelled distance with capacity constraint:", objective)
#
#
# # Check whether schedule matches time windows
# Optimizer.check_time_windows(schedule,solution,time_windows)
