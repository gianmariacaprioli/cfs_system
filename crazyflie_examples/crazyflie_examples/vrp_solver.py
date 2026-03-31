from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import os
import csv
import sys

# --- CONFIGURAZIONE PERCORSI ---
BASE_PATH = "/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples"
INPUT_DIR = os.path.join(BASE_PATH, "data/pillars_config") # Dove cerca coords e qi
OUTPUT_DIR = os.path.join(BASE_PATH, "data")               # Dove salva route_vehicle_x.csv

def load_data_from_csv(coords_filename, demands_filename):
    """
    Legge i CSV dalla cartella INPUT_DIR.
    Accetta solo i NOMI dei file.
    """
    # Costruzione path completi
    coords_path = os.path.join(INPUT_DIR, coords_filename)
    demands_path = os.path.join(INPUT_DIR, demands_filename)

    stopsLon = []
    stopsLat = []
    qi = []

    print(f"[VRP Lib] Lettura coordinate da: {coords_path}")
    print(f"[VRP Lib] Lettura domande da: {demands_path}")

    # Lettura Coordinate
    try:
        with open(coords_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                try:
                    x = float(row[0])
                    y = float(row[1])
                    stopsLon.append(x)
                    stopsLat.append(y)
                except ValueError:
                    continue 
    except FileNotFoundError:
        print(f"ERRORE: File non trovato: {coords_path}")
        return None, None, None

    # Lettura Domande
    try:
        with open(demands_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                try:
                    q = int(float(row[0]))
                    qi.append(q)
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"ERRORE: File non trovato: {demands_path}")
        return None, None, None

    return stopsLon, stopsLat, qi

def create_data_model(stopsLon, stopsLat, qi, Q, m):
    data = {}
    try:
        depot_index = qi.index(0)
    except ValueError:
        print("ATTENZIONE: Nessun deposito (qi=0). Uso indice 0.")
        depot_index = 0

    data['locations'] = list(zip(stopsLon, stopsLat))
    data['demands'] = qi
    data['vehicle_capacities'] = [Q] * m
    data['num_vehicles'] = m
    data['depot'] = depot_index
    return data

def compute_euclidean_distance_matrix(locations):
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
                matrix[from_node][to_node] = int(dist * 1000)
    return matrix

def save_routes_to_csv(data, manager, routing, solution):
    """Salva i risultati nella cartella OUTPUT_DIR."""
    
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    # Pulizia vecchi file route_vehicle per evitare mix con run precedenti
    for f in os.listdir(OUTPUT_DIR):
        if f.startswith("route_vehicle_") and f.endswith(".csv"):
            os.remove(os.path.join(OUTPUT_DIR, f))

    saved_count = 0

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_coords = []
        route_distance = 0
        
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            x_coord, y_coord = data['locations'][node_index]
            z_coord = 1.0 # Z fisso
            route_coords.append((x_coord, y_coord, z_coord))
            
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        # Punto finale
        node_index = manager.IndexToNode(index)
        x_coord, y_coord = data['locations'][node_index]
        route_coords.append((x_coord, y_coord, 1.0))

        if route_distance > 0:
            filename = os.path.join(OUTPUT_DIR, f'route_vehicle_{vehicle_id}.csv')
            with open(filename, mode='w', newline='') as csv_file:
                writer = csv.writer(csv_file)
                for coord in route_coords:
                    writer.writerow(coord)
            # print(f"Salvato: {filename}")
            saved_count += 1
    
    print(f"[VRP Lib] Salvati {saved_count} percorsi in {OUTPUT_DIR}")

def solve_vrp(coords_filename, demands_filename, Q, m):
    """
    Funzione principale.
    Accetta NOMI FILE (es. 'coords.csv'), Q e m.
    """
    # 1. Carica dati (gestione path interna)
    stopsLon, stopsLat, qi = load_data_from_csv(coords_filename, demands_filename)
    
    if stopsLon is None:
        print("Interruzione risoluzione: Dati non validi.")
        return

    # 2. Modello
    data = create_data_model(stopsLon, stopsLat, qi, Q, m)
    
    # 3. OR-Tools Setup
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
        demand_callback_index, 0, data['vehicle_capacities'], True, 'Capacity')

    # 4. Parametri Ricerca
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 2

    # 5. Risoluzione
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        save_routes_to_csv(data, manager, routing, solution)
    else:
        print('Nessuna soluzione trovata!')