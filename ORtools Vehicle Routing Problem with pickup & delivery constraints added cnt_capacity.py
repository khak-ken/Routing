from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    # distance in km (straight line):
    data['distance_matrix']=[
[0,12,18,12,38,33,19,36,38,11,14,54,50],
[12,0,25,24,49,21,29,42,44,22,25,62,54],
[18,25,0,17,27,44,30,18,20,14,15,36,32],
[12,24,17,0,27,45,12,33,35,3,3,47,48],
[38,49,27,27,0,69,33,27,28,27,24,28,39],
[33,21,44,45,69,0,49,58,60,44,47,78,67],
[19,29,30,12,33,49,0,45,47,15,15,57,60],
[36,42,18,33,27,58,45,0,2,31,30,20,15],
[38,44,20,35,28,60,47,2,0,32,32,19,13],
[11,22,14,3,27,44,15,31,32,0,3,46,45],
[14,25,15,3,24,47,15,30,32,3,0,44,44],
[54,62,36,47,28,78,57,20,19,46,44,0,20],
[50,54,32,48,39,67,60,15,13,45,44,20,0],
]
    
    
#    data['distance_matrix'] = [
#        [
#            0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354,
#            468, 776, 662
#        ],
#        [
#            548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
#            1016, 868, 1210
#        ],
#        [
#            776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164,
#            1130, 788, 1552, 754
#        ],
#        [
#            696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
#            1164, 560, 1358
#        ],
#        [
#            582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
#            1050, 674, 1244
#        ],
#        [
#            274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628,
#            514, 1050, 708
#        ],
#        [
#            502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856,
#            514, 1278, 480
#        ],
#        [
#            194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320,
#            662, 742, 856
#        ],
#        [
#            308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662,
#            320, 1084, 514
#        ],
#        [
#            194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388,
#            274, 810, 468
#        ],
#        [
#            536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764,
#            730, 388, 1152, 354
#        ],
#        [
#            502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114,
#            308, 650, 274, 844
#        ],
#        [
#            388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194,
#            536, 388, 730
#        ],
#        [
#            354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0,
#            342, 422, 536
#        ],
#        [
#            468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536,
#            342, 0, 764, 194
#        ],
#        [
#            776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274,
#            388, 422, 764, 0, 798
#        ],
#        [
#            662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730,
#            536, 194, 798, 0
#        ],
#    ]
    
    #data['demands'] = [0, 4, 2, -2, 2, 2, -4, 3, -3, -2, -2, -3, -4]
    data['demands'] = [0, 2, 2, -2, 2, 2, -2, 3, -3, -2, -2, -2, 2]
    data['pickups_deliveries'] = [
        [1, 2],
        [2, 10],
        [4, 3],
        [5, 9],
        [7, 8],
        [12, 11],
#        [15, 11],
#        [13, 12],
#        [16, 14],
    ]
    data['vehicle_capacities'] = [16,16,16,16,16]
    data['demands_cnt'] = [6, -6, 2, -2, 2, 2, -2, 3, -3, -2, -2, -2, 2]
    data['vehicle_capacities_cnt'] = [6, 6, 6, 6,6]
    data['num_vehicles'] = 5
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    total_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            print(node_index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                    
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Total Distance of all routes: {0}m, {1} minutes'.format(total_distance,round(((total_distance/260)*60),2)))


def main():
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        300,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    
    #adding additional capacity constrain: cnt
    def demand_callback_cnt(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands_cnt'][from_node]

    demand_callback_cnt_index = routing.RegisterUnaryTransitCallback(demand_callback_cnt)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_cnt_index,
        0,  # null capacity slack
        data['vehicle_capacities_cnt'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity_cnt')

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
        routing.solver().Add(distance_dimension.CumulVar(pickup_index) <= distance_dimension.CumulVar(delivery_index))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.solution_limit = 1
    search_parameters.time_limit.seconds = 300

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if assignment:
        print_solution(data, manager, routing, assignment)


if __name__ == '__main__':
    main()
