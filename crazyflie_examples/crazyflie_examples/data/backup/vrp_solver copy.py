from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import os
import csv

def create_data_model(case_number):
    """Crea il dizionario dei dati in base al case_number."""
    data = {}
    Q = 10
    m = 4
    
    if case_number == 1:
        stopsLon = [0, 3, 3, 6, 15, 9, 6, 15, 3, 3, -3, -3, -12, -3, -3]
        stopsLat = [0, 6, 12, 3, 6, 3, 0, 3,-6, -12, -12, -6,  0,  6,  12]
        qi = [0, 2, 5, 6, 3, 2, 1, 1, 1, 1, 1, 3, 9, 2, 1]
    elif case_number == 2:
        stopsLon = [0, 1, 1, 2, 5, 3, 2, 5, 1, 1, -1]
        stopsLat = [0, 2, 4, 1, 2, 1, 0, 1,-2, -4, -4]
        qi = [1, 2, 5, 6, 3, 2, 0, 1, 1, 1, 1]
    elif case_number == 3:
        stopsLon = [0, 1, 1, 2, 5, 3, 2, 5, 1, 1, -1, -1, -4, -1, -1]
        stopsLat = [0, 2, 4, 1, 2, 1, 0, 1,-2, -4, -4, -2,  0,  2,  4]
        qi = [1, 2, 5, 6, 3, 2, 1, 1, 0, 1, 1, 3, 9, 2, 1]
    else:
        print(f"Case number {case_number} non valido. Uso default (1).")
        # Fallback default per evitare crash
        stopsLon = [0, 1, 1, 2, 5, 3, 2, 5, 1, 1, -1, -1, -4, -1, -1]
        stopsLat = [0, 2, 4, 1, 2, 1, 0, 1,-2, -4, -4, -2,  0,  2,  4]
        qi = [0, 2, 5, 6, 3, 2, 1, 1, 1, 1, 1, 3, 9, 2, 1]

    try:
        depot_index = qi.index(0)
    except ValueError:
        print("Nessun deposito (qi=0)! Uso indice 0.")
        depot_index = 0

    data['locations'] = list(zip(stopsLon, stopsLat))
    data['demands'] = qi
    data['vehicle_capacities'] = [Q] * m
    data['num_vehicles'] = m
    data['depot'] = depot_index
    
    return data

def compute_euclidean_distance_matrix(locations):
    """Calcola la matrice delle distanze."""
    size = len(locations)
    matrix = {}
    for from_node in range(size):
        matrix[from_node] = {}
        for to_node in range(size):
            if from_node == to_node:
                matrix[from_node][to_node] = 0
            else:
                x1, y1 = locations[from_node]
                x2, y2 = locations[to_node]
                dist = math.hypot(x1 - x2, y1 - y2)
                # Scaliamo per usare int
                matrix[from_node][to_node] = int(dist * 1000)
    return matrix

def print_solution_console(data, manager, routing, solution):
    """Stampa a console per verifica."""
    total_distance = 0
    print(f"--- Soluzione trovata ---")
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = f'Route {vehicle_id}: '
        route_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            plan_output += f'{node_index} -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        
        node_index = manager.IndexToNode(index)
        plan_output += f'{node_index}'
        if route_distance > 0:
            print(plan_output)
        total_distance += route_distance

def save_routes_to_csv(data, manager, routing, solution):
    """Salva CSV contenenti SOLO le coordinate x,y,z per ogni path."""
    
    # Percorso specifico richiesto
    output_dir = '/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data'
    
    # Controllo se il path esiste, altrimenti provo a crearlo (o fallback su cartella locale)
    try:
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"Creata cartella '{output_dir}'")
    except OSError:
        print(f"Impossibile creare percorso {output_dir}. Salvo nella cartella corrente 'data'.")
        output_dir = 'data'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    saved_files_count = 0

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        
        route_coords = []
        route_distance = 0
        
        # --- Raccolta Coordinate ---
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            x_coord, y_coord = data['locations'][node_index]

            # Logica Z: 0 se siamo all'origine (deposito/terra), 3 se siamo in volo
            if x_coord == 0 and y_coord == 0:
                z_coord = 1.0
            else:
                z_coord = 1.0
            
            # Aggiungi coordinate alla lista
            route_coords.append((x_coord, y_coord, z_coord))
            
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        # Aggiungi il punto finale (ritorno al deposito/punto finale)
        node_index = manager.IndexToNode(index)
        x_coord, y_coord = data['locations'][node_index]
        
        # Calcolo Z anche per l'ultimo punto
        if x_coord == 0 and y_coord == 0:
             z_coord = 1.0
        else:
             z_coord = 1.0
             
        route_coords.append((x_coord, y_coord, z_coord))

        # --- Scrittura CSV ---
        if route_distance > 0:
            filename = os.path.join(output_dir, f'route_vehicle_{vehicle_id}.csv')
            
            with open(filename, mode='w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                
                # Scrivi solo le coppie di coordinate (senza header come richiesto implicitamente)
                for coord in route_coords:
                    writer.writerow(coord)
            
            print(f"Salvato: {filename}")
            saved_files_count += 1

    if saved_files_count == 0:
        print("Tutti i veicoli sono fermi (nessun CSV generato).")

def solve_vrp(case_number):
    """
    Funzione principale per risolvere il VRP.
    Accetta case_number come intero (1, 2, o 3).
    """
    print(f"Avvio risoluzione VRP per caso numero: {case_number}")
    
    data = create_data_model(case_number)
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], 
                                           data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        data['vehicle_capacities'],
        True,
        'Capacity')

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 2

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution_console(data, manager, routing, solution)
        save_routes_to_csv(data, manager, routing, solution)
    else:
        print('Nessuna soluzione trovata!')

# Esempio di utilizzo se lo script viene eseguito direttamente
if __name__ == '__main__':
    # Puoi cambiare questo numero per testare
    CASE_TO_RUN = 1 
    solve_vrp(CASE_TO_RUN)