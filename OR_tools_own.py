"""Vehicles Routing Problem (VRP) with Time Windows."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import numpy as np

class ORtool:

    def __init__(self):
        print("Let's go")

    def create_data_model(self,T,D,demand,service_time,time_windows):
        """Stores the data for the problem."""
        data = {}
        data['time_matrix'] = T
        data['distance_matrix'] = D
        data['time_windows'] = [
        (0, 540),  # depot
        (0, 180),  # 1
        (0, 180),  # 2
        (0, 180),  # 3
        (0, 180),  # 4
        (0, 180),  # 5
        (0, 180),  # 6
        (0, 180),  # 7
        (120, 180),  # 8
        (0, 180),  # 9
        (0, 180),  # 10
        (0, 180),  # 11
        (300, 480),  # 12
        (300, 480),  # 13
        (240, 420),  # 14
        (240, 420),  # 15
        (240, 420),  # 16
        (240, 420),  # 17
        (240, 420),  # 18
        (180, 540),  # 19
        (180, 540),  # 20
        (240, 420),  # 21
        (240, 420),  # 22
        (240, 420),  # 23
        (360, 540),  # 24
        (360, 540),  # 25
    ]

        data['service_time'] = service_time
        data['num_vehicles'] = 2
        data['depot'] = 0

        data['demands'] = demand
        data['vehicle_capacities'] = [1000, 1000]
        #print("Service times:",data['service_time'])
        #print("Time matrix:",data['time_matrix'][8])
        #print(data['time_matrix'][8]+data['service_time'][8])

        #for i in range(len(data['time_matrix'])):
            #data['time_matrix'][i] = data['time_matrix'][i] + data['service_time'][i]

        #print(data['time_matrix'][8])
        return data


    def print_solution(self, data, manager, routing, solution):
        """Prints solution on console."""
        print(f'Objective: {solution.ObjectiveValue()}')
        time_dimension = routing.GetDimensionOrDie('Time')
        total_route = []
        total_time = 0
        total_load = 0
        total_schedule = []
        total_schedule_max = []
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route = [0]
            route_load = 0
            schedule = [0]
            schedule_max = [9]
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += '{0} Time({1},{2}) -> '.format(
                    manager.IndexToNode(index), solution.Min(time_var),
                    solution.Max(time_var))
                index = solution.Value(routing.NextVar(index))
                route_load += data['demands'][int(manager.IndexToNode(index))]
                #print("Load of node", index, route_load)
                route.append(int(manager.IndexToNode(index)))
                time_var = time_dimension.CumulVar(index)
                schedule.append(solution.Min(time_var))
                schedule_max.append(solution.Max(time_var))
                #print("Time var:",time_var)
                #print("Timeeee:", solution.Max(time_var))
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                        solution.Min(time_var),
                                                        solution.Max(time_var))
            plan_output += 'Time of the route: {}min\n'.format(
                solution.Min(time_var))
            print(plan_output)
            print('Route vehicle',vehicle_id,route)
            print('Schedule:',schedule)
            total_time += solution.Min(time_var)
            total_route.append(route)
            total_schedule.append(schedule)
            total_schedule_max.append(schedule_max)
            total_load += route_load
            print("Total load of route:",total_load)
        print('Total time of all routes: {}min'.format(total_time))
        return total_route, total_schedule, total_schedule_max


    def main(self,T,D,demand,service_time,time_windows):
        """Solve the VRP with time windows."""
        # Instantiate the data problem.
        data = self.create_data_model(T,D,demand,service_time,time_windows)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                               data['num_vehicles'], data['depot'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)


        # Create and register a transit callback.
        def time_callback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            # Convert from routing variable Index to time matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['time_matrix'][from_node][to_node]

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]


        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = manager.IndexToNode(from_index)
            return data['demands'][from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(
            demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            data['vehicle_capacities'],  # vehicle maximum capacities
            True,  # start cumul to zero
            'Capacity')

        # Add Time Windows constraint.
        time = 'Time'
        routing.AddDimension(
            transit_callback_index,
            200,  # allow waiting time
            400,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time)
        time_dimension = routing.GetDimensionOrDie(time)
        # Add time window constraints for each location except depot.
        for location_idx, time_window in enumerate(data['time_windows']):
            if location_idx == data['depot']:
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.
        depot_idx = data['depot']
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                data['time_windows'][depot_idx][0],
                data['time_windows'][depot_idx][1])

        # Instantiate route start and end times to produce feasible times.
        for i in range(data['num_vehicles']):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i)))
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(i)))

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.FromSeconds(1)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            total_route = self.print_solution(data, manager, routing, solution)

        return total_route


# if __name__ == '__main__':
#     main()