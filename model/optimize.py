import numpy as np
import pandas as pd
import haversine as hs
import time
import matplotlib.pyplot as plt

"""Vehicles Routing Problem (VRP) with Time Windows."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class Optimizer:
    def __init__(self, params: dict):
        self.total_capacity = params['constraints']['total_capacity']
        self.total_range = params['constraints']['total_range']
        self.start_time = params['constraints']['starting_time']
        self.speed = params['constraints']['speed']
        self.wait_time = params['constraints']['wait_time']

        self.wait = False
        self.route = None

    def data_preparation(self):
        df = pd.read_csv("data/data.txt", index_col=None, header=0)
        gps_coordinates = df[["lat","lon"]].values
        time_window = df[["time_start","time_end"]]
        demand = df[["demand"]].values
        demand[0] = 0
        demand = demand.astype(int)

        service_time = df[['service_time']].values
        service_time = service_time.astype(float)
        self.service_time = service_time/60           #convert seconds to minutes
        self.service_time = self.service_time.astype(int) #round to minutes

        time_window['start_hour'] = pd.to_datetime(time_window['time_start']).dt.hour
        time_window['end_hour'] = pd.to_datetime(time_window['time_end']).dt.hour
        time_window = time_window[['start_hour','end_hour']]
        time_window = time_window.fillna(18.0)
        time_window = time_window.astype(int)
        self.time_window = time_window.values
        self.time_window = (self.time_window-9)*60      #convert hours to minutes

        return gps_coordinates, self.time_window, demand, self.service_time

    def visualize_locations(self,gps_coordinates,gps_locations,title):
        gps_array = gps_coordinates

        plt.figure()
        plt.scatter(gps_array[:,1],gps_array[:,0])
        for i in range(len(gps_array)):
            plt.annotate(i, (gps_array[i,1], gps_array[i,0]))

        for i in range(len(gps_locations)):
            locations = gps_locations[i]
            plt.plot(locations[:,1],locations[:,0])
        plt.title(title)
        plt.show()



    def gps_to_distances(self,gps_coordinates):
        D = np.zeros((len(gps_coordinates),len(gps_coordinates)))

        for i in range(len(gps_coordinates)):
            for j in range(len(gps_coordinates)):
                loc1 = gps_coordinates[i]
                loc2 = gps_coordinates[j]
                D[i,j] = hs.haversine(loc1, loc2)

        # Create time matrix
        self.T = D/self.speed               #time in minutes
        self.T = self.T.astype(int)         #round to full minutes
        return D, self.T

    def add_service_times_to_T(self,T,service_times):
        self.T = T+service_times
        #print(T)
        return self.T

    def get_initial_distance(self,D):
        distance = 0
        for i in range(len(D)):
            distance += 2 * D[i, 0]
        return distance

    def clarke_wright_savings(self, unrouted, i, D):
        # Saving = Di0 + Dj0 - Dij
        savings = [(D[i, 0] + D[0, j] - D[i, j], i, j) for j in unrouted if i != j]
        savings.sort()
        print(savings)
        return savings

    def sequential_savings_init(self, D, initial_distance, savings_callback=clarke_wright_savings):
        # Size of distance matrix
        N = len(D)

        # All the nodes in a list
        unrouted = list(range(1, N))

        total_savings = 0
        savings = None
        initial_node = None

        distance = [initial_distance]
        start_time = time.perf_counter()
        time1 = [0]
        while unrouted:
            # If there is no savings matrix: start a new route
            if not savings:
                if initial_node:
                    # Construct savings matrix
                    print("No possible merges so have to create new savings matrix")

                    savings = savings_callback(self, unrouted, self.route[-1], D)
                    print("Created new Savings matrix")

                if not initial_node:
                    # Select an initial node
                    initial_node = unrouted[8]

                    # Initialise route
                    self.route = [initial_node]
                    print("Initial node:",self.route)

                    # Remove route initialisation from set of unrouted nodes
                    unrouted.remove(initial_node)

                    # Construct savings matrix
                    savings = savings_callback(self, unrouted, initial_node, D)


            while len(savings) > 0 and unrouted:
                # The highest saving is the last one in the savings matrix
                # i is the one to merge with, and j is the merge_candidate
                largest_saving, i, j = savings.pop()
                #print("i:",i)

                # If merge candidate is already merged in the route: continue
                if not j in unrouted:
                    continue

                print('Unrouted nodes:', unrouted)
                print("Possible saving:", largest_saving, i, j)

                # Check where the merge candidate needs to be merged
                merge_on_left = self.route[0] == i
                merge_on_right = self.route[-1] == i and \
                                 len(self.route) > 1
                if i == 0:
                    merge_on_right = True
                    merge_on_left = False


                if merge_on_left:
                    route = [j] + self.route
                    schedule, route_possible = self.create_schedule_from_route(route,self.T,self.time_window)
                    #route_possible = True
                    if route_possible:
                        self.route = route
                        saving = largest_saving
                    else:
                        merge_on_left = False

                if merge_on_right:
                    route = self.route + [j]
                    schedule, route_possible = self.create_schedule_from_route(route,self.T,self.time_window)
                    #route_possible = True
                    if route_possible:
                        self.route = route
                        saving = largest_saving
                    else:
                        merge_on_right = False

                # If both return "False": try next in savings matrix
                if not (merge_on_left or merge_on_right):
                    continue

                # Remove the merge candidate from the list of unmerged nodes
                unrouted.remove(j)

                # Create the new savings matrix
                savings += savings_callback(self, unrouted, j, D)


                # Sort the savings matrix so that the highest savings is the last row
                savings.sort()

                # Add the saving to total savings
                total_savings += saving

                # Current distance = (inital distance) - (savings up till now)
                distance1 = initial_distance - total_savings
                distance.append(distance1)

                # Runtime at current time
                time1.append(float((time.perf_counter() - start_time)))

                print("Route:",self.route)

            # After all savings have been applied, the route is the solution
            solution = [0] + self.route + [0]
            #self.route = None

        return solution, time1, distance1



    def get_objective_distance(self, D, solution):
        """
        Calculates objective value for a solution
        """
        return np.sum(D[solution, np.roll(solution, 1)])

    def add_stopover(self, route, demand, D,T,schedule,schedule_max):
        """
        Adds stopover to makes sure total capacity is not exceeded
        """
        total_load = 0


        for i in range(len(route)):
            current_stop = route[i]
            total_load += demand[current_stop]


        if total_load > self.total_capacity:
            # Brute force calculate where to fit in stop
            best_objective = np.Inf
            load = 0
            for j in range(len(route)):
                current_stop = route[j]
                load += demand[current_stop]
                if np.abs(total_load - load) <= 1000 and load <=1000:
                    new_route = route[:j+1] + [0] + route[j+1:]
                    new_schedule = schedule[:j+1] + [schedule[j] + T[route[j], 0]] + [
                        x - T[route[j], route[j+1]] + T[route[j], 0] + T[0, route[j+1]] for x in
                        schedule[j+1:]]
                    route_possible = self.check_time_windows(new_schedule,new_route,self.time_window)
                    if not route_possible:
                        continue
                    objective = self.get_objective_distance(D, new_route)
                    if objective < best_objective:
                        best_route = new_route
                        best_objective = objective
                        best_j = j

        new_schedule = schedule[:best_j+1] + [schedule[best_j] + T[route[best_j], 0]] + [x - T[route[best_j], route[best_j+1]] + T[route[best_j], 0] + T[0, route[best_j+1]] for x in schedule[best_j+1:]]
        new_schedule_max = schedule_max[:best_j] + [schedule_max[best_j]] + schedule_max[best_j:]

        return best_route, new_schedule, new_schedule_max

    def add_stopover_no_constraints(self, route, demand, D):
        """
        Adds stopover to makes sure total capacity is not exceeded
        """
        total_load = 0


        for i in range(len(route)):
            current_stop = route[i]
            total_load += demand[current_stop]


        if total_load > self.total_capacity:
            # Brute force calculate where to fit in stop
            best_objective = np.Inf
            load = 0
            for j in range(len(route)):
                current_stop = route[j]
                load += demand[current_stop]
                if np.abs(total_load - load) <= 1000 and load <=1000:
                    new_route = route[:j] + [0] + route[j:]
                    objective = self.get_objective_distance(D, new_route)
                    if objective < best_objective:
                        best_route = new_route
                        best_objective = objective
                        best_j = j

        return best_route

    def add_service_times(self,route,schedule,service_times):
        total_service_time = 0

        for i in range(1,len(schedule)-1):
            total_service_time += service_times[route[i]]
            #print(schedule[i+1])
            schedule[i+1] = int(schedule[i+1] + total_service_time)
        return schedule


    def from_schedule_to_time(self, schedule):
        for i in range(len(schedule)):
            schedule[i] = schedule[i]/60 + 9

        return schedule

    def select_available_locations(self, time, locations, service_time, time_windows):
        available_locations = []
        print(time)
        for i in range(len(locations)):
            if time + service_time[locations[i]] >= time_windows[locations[i],0] and time + service_time[locations[i]] <= time_windows[locations[i],1]:
                available_locations = available_locations + [locations[i]]

        return available_locations

    def create_schedule_from_route(self,route,T,time_windows):
        time = self.start_time
        self.schedule = [0]*len(route)
        route_possible = True
        for i in range(len(route)):
            if i == 0:
                if route[i] == 0:
                    travel_time = 0
                else:
                    travel_time = T[0,route[i]]
            else:
                travel_time = T[route[i-1],route[i]]
            added_time = travel_time #+ service_time
            time = time + added_time
            if time >= time_windows[route[i],0] and time <= time_windows[route[i],1]:
                self.schedule[i] = time
            else:
                if 0 < time_windows[route[i],0] - time < 121:
                    time = time + (time_windows[route[i],0] - time)
                    self.schedule[i] = time
                else:
                    route_possible = False
                    break

        return self.schedule, route_possible

    def check_time_windows(self,schedule,route,time_windows):
        route_possible = True
        for i in range(len(route)):
            if schedule[i] >= time_windows[route[i],0] and schedule[i] <= time_windows[route[i],1]:
                continue
            else:
                print("Schedule doesn't match for location:", route[i])
                route_possible = False
                break


        return route_possible

    def make_schedule_possible(self,schedule,route,time_windows):
        for i in range(len(route)):
            if schedule[i] >= time_windows[route[i],0] and schedule[i] <= time_windows[route[i],1]:
                continue
            else:
                print("Schedule doesn't match for location:", route[i])
                if schedule[i] < time_windows[route[i],0]:
                    schedule[i] = schedule[i] + np.ceil(time_windows[route[i],0] - schedule[i])
                if schedule[i] > time_windows[route[i], 1]:
                    schedule[i] = schedule[i] - np.ceil(schedule[i] - time_windows[route[i], 1])

        sorted = np.argsort(schedule[:-1])
        print("Arg_sorted:",sorted)

        new_route = np.zeros(len(sorted))
        new_schedule = np.zeros(len(sorted))
        for i in range(len(sorted)):
            new_route[i] = route[sorted[i]]
            new_schedule[i] = schedule[sorted[i]]

        new_route = new_route.tolist() + [0]
        for i in range(len(new_route)):
            new_route[i] = int(new_route[i])

        print("Old route:", route)
        print("New route:", new_route)
        print("New schedule:", new_schedule)
        print(len(new_schedule))
        print(len(new_route))

        return schedule, new_route

    def local_search(self, initial_solution, T, time_windows):
        """
        Local search improvement.

        Args:
            D - distance matrix (n x n)
            initial_solution - vector (n,)
        """
        n = len(initial_solution)
        best_schedule, route_possible = self.create_schedule_from_route(initial_solution,T,time_windows)
        best_time = best_schedule[-1] - best_schedule[0]
        best_solution = initial_solution
        improved = True
        while improved:
            improved = False
            for i in range(1,n-1):
                for j in range(1,n-1):
                    # Create new solution by swapping 2 nodes
                    new_solution = best_solution.copy()
                    new_solution[j], new_solution[i] = new_solution[i], new_solution[j]
                    #new_solution[i] = new_solution[j]
                    new_schedule, route_possible = self.create_schedule_from_route(new_solution,T,time_windows)
                    new_time = new_schedule[-1] - new_schedule[0]

                    if new_time < best_time and route_possible:
                        best_solution = new_solution
                        best_schedule = new_schedule
                        best_time = new_time
                        improved = True

        return best_solution, best_schedule







