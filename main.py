"""
Drone Teslimat Rota Optimizasyonu - Ana Uygulama
Bu sistem, enerji limitleri ve uçuş yasağı bölgeleri altında
çoklu drone'lar için optimal teslimat rotalarını belirler.
"""

import time
import matplotlib.pyplot as plt
from drone_system import DroneFleet, DeliveryPoint, NoFlyZone
from pathfinding import AStarPathfinder
from genetic_algorithm import GeneticAlgorithm
from csp_solver import CSPSolver
from visualization import DroneVisualizer
from data_generator import DataGenerator
import ast

def load_data_from_file(filename):
    """Veri dosyasından drone, teslimat ve no-fly zone verilerini yükler"""
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Python dictionary formatındaki veriyi parse et
    local_vars = {}
    exec(content, {}, local_vars)
    
    return local_vars['drones'], local_vars['deliveries'], local_vars['no_fly_zones']

def create_drone_fleet(drone_data):
    """Drone verilerinden DroneFleet oluşturur"""
    fleet = DroneFleet()
    for drone in drone_data:
        fleet.add_drone(
            drone_id=drone['id'],
            max_weight=drone['max_weight'],
            battery=drone['battery'],
            speed=drone['speed'],
            start_pos=drone['start_pos']
        )
    return fleet

def create_delivery_points(delivery_data):
    """Teslimat verilerinden DeliveryPoint listesi oluşturur"""
    deliveries = []
    for delivery in delivery_data:
        deliveries.append(DeliveryPoint(
            delivery_id=delivery['id'],
            position=delivery['pos'],
            weight=delivery['weight'],
            priority=delivery['priority'],
            time_window=delivery['time_window']
        ))
    return deliveries

def create_no_fly_zones(no_fly_data):
    """No-fly zone verilerinden NoFlyZone listesi oluşturur"""
    zones = []
    for zone in no_fly_data:
        zones.append(NoFlyZone(
            zone_id=zone['id'],
            coordinates=zone['coordinates'],
            active_time=zone['active_time']
        ))
    return zones

def run_scenario_1():
    """Senaryo 1: 5 drone, 20 teslimat, 3 no-fly zone"""
    print("=" * 50)
    print("SENARYO 1: 5 Drone, 20 Teslimat, 3 No-Fly Zone")
    print("=" * 50)
    
    # Veri dosyasından yükle
    drone_data, delivery_data, no_fly_data = load_data_from_file('verisetitxt.txt')
    
    # Nesneleri oluştur
    fleet = create_drone_fleet(drone_data)
    deliveries = create_delivery_points(delivery_data)
    no_fly_zones = create_no_fly_zones(no_fly_data)
    
    # A* ile rota bulma
    print("\n1. A* Algoritması ile Rota Bulma...")
    start_time = time.time()
    pathfinder = AStarPathfinder(fleet, deliveries, no_fly_zones)
    astar_routes = pathfinder.find_optimal_routes()
    astar_time = time.time() - start_time
    
    # CSP çözümü
    print("2. CSP ile Kısıt Denetimi...")
    start_time = time.time()
    csp_solver = CSPSolver(fleet, deliveries, no_fly_zones)
    csp_solution = csp_solver.solve()
    csp_time = time.time() - start_time
    
    # Genetic Algorithm optimizasyonu
    print("3. Genetic Algorithm ile Optimizasyon...")
    start_time = time.time()
    ga = GeneticAlgorithm(fleet, deliveries, no_fly_zones, population_size=50, generations=100)
    ga_solution = ga.evolve()
    ga_time = time.time() - start_time
    
    # Sonuçları raporla
    print_results("A* Algoritması", astar_routes, astar_time)
    print_results("CSP Çözümü", csp_solution, csp_time)
    print_results("Genetic Algorithm", ga_solution, ga_time)
    
    # Görselleştirme
    visualizer = DroneVisualizer(fleet, deliveries, no_fly_zones)
    visualizer.plot_routes(ga_solution, "Senaryo 1 - GA Optimizasyonu")
    
    return ga_solution

def run_scenario_2():
    """Senaryo 2: 10 drone, 50 teslimat, 5 dinamik no-fly zone"""
    print("=" * 50)
    print("SENARYO 2: 10 Drone, 50 Teslimat, 5 No-Fly Zone")
    print("=" * 50)
    
    # Rastgele veri üret
    generator = DataGenerator()
    drone_data = generator.generate_drones(10)
    delivery_data = generator.generate_deliveries(50)
    no_fly_data = generator.generate_no_fly_zones(5)
    
    # Nesneleri oluştur
    fleet = create_drone_fleet(drone_data)
    deliveries = create_delivery_points(delivery_data)
    no_fly_zones = create_no_fly_zones(no_fly_data)
    
    # Sadece en iyi algoritma ile çöz (performans için)
    print("Genetic Algorithm ile Optimizasyon...")
    start_time = time.time()
    ga = GeneticAlgorithm(fleet, deliveries, no_fly_zones, population_size=100, generations=150)
    ga_solution = ga.evolve()
    ga_time = time.time() - start_time
    
    print_results("Genetic Algorithm (Büyük Senaryo)", ga_solution, ga_time)
    
    # Görselleştirme
    visualizer = DroneVisualizer(fleet, deliveries, no_fly_zones)
    visualizer.plot_routes(ga_solution, "Senaryo 2 - Büyük Ölçekli Optimizasyon")
    
    return ga_solution

def print_results(algorithm_name, solution, execution_time):
    """Algoritma sonuçlarını yazdırır"""
    print(f"\n{algorithm_name} Sonuçları:")
    print(f"  Çalışma Süresi: {execution_time:.2f} saniye")
    
    if solution:
        completed_deliveries = sum(len(routes) for routes in solution.values())
        total_distance = calculate_total_distance(solution)
        total_energy = calculate_total_energy(solution)
        
        print(f"  Tamamlanan Teslimat: {completed_deliveries}")
        print(f"  Toplam Mesafe: {total_distance:.2f} metre")
        print(f"  Toplam Enerji Tüketimi: {total_energy:.2f} mAh")
    else:
        print("  Çözüm bulunamadı!")

def calculate_total_distance(solution):
    """Toplam mesafeyi hesaplar"""
    total = 0
    for drone_routes in solution.values():
        for i in range(len(drone_routes) - 1):
            pos1 = drone_routes[i]
            pos2 = drone_routes[i + 1]
            distance = ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5
            total += distance
    return total

def calculate_total_energy(solution):
    """Toplam enerji tüketimini hesaplar"""
    # Basit enerji modeli: mesafe * enerji katsayısı
    return calculate_total_distance(solution) * 0.1

def main():
    """Ana fonksiyon"""
    print("DRONE TESLİMAT ROTA OPTİMİZASYONU")
    print("=" * 50)
    
    try:
        # Senaryo 1
        solution1 = run_scenario_1()
        
        # Senaryo 2
        solution2 = run_scenario_2()
        
        print("\n" + "=" * 50)
        print("TÜM SENARYOLAR TAMAMLANDI!")
        print("Görselleştirmeler matplotlib ile gösterildi.")
        
    except Exception as e:
        print(f"Hata oluştu: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 