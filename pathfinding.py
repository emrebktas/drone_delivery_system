"""
A* Algoritması ile Rota Bulma Modülü
Bu modül drone'lar için en optimal teslimat rotalarını
A* algoritması kullanarak hesaplar.
"""

import heapq
import math
from typing import List, Dict, Tuple, Optional, Set
from drone_system import DroneFleet, DeliveryPoint, NoFlyZone, Drone

class Node:
    """Graf düğümü sınıfı"""
    
    def __init__(self, position: Tuple[float, float], delivery_id: Optional[int] = None):
        self.position = position
        self.delivery_id = delivery_id  # None ise başlangıç noktası
        self.g_cost = float('inf')      # Başlangıçtan bu düğüme maliyet
        self.h_cost = 0                 # Bu düğümden hedefe tahmini maliyet
        self.f_cost = float('inf')      # g_cost + h_cost
        self.parent = None
        
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class Graph:
    """Teslimat noktaları için graf yapısı"""
    
    def __init__(self, deliveries: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]):
        self.nodes = {}  # position -> Node
        self.deliveries = {d.delivery_id: d for d in deliveries}
        self.no_fly_zones = no_fly_zones
        self.adjacency_list = {}  # Komşuluk listesi
        
        self._build_graph()
    
    def _build_graph(self):
        """Graf yapısını oluşturur"""
        # Her teslimat noktası için düğüm oluştur
        for delivery in self.deliveries.values():
            node = Node(delivery.position, delivery.delivery_id)
            self.nodes[delivery.position] = node
            self.adjacency_list[delivery.position] = []
        
        # Komşuluk ilişkilerini kurup kenar ağırlıklarını hesapla
        positions = list(self.nodes.keys())
        for i, pos1 in enumerate(positions):
            for j, pos2 in enumerate(positions):
                if i != j and self._is_valid_connection(pos1, pos2):
                    self.adjacency_list[pos1].append(pos2)
    
    def _is_valid_connection(self, pos1: Tuple[float, float], 
                           pos2: Tuple[float, float]) -> bool:
        """İki nokta arasında geçerli bağlantı olup olmadığını kontrol eder"""
        # No-fly zone kontrolü
        for zone in self.no_fly_zones:
            if zone.intersects_path(pos1, pos2):
                return False
        return True
    
    def get_neighbors(self, position: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Bir pozisyonun komşularını döndürür"""
        return self.adjacency_list.get(position, [])

class AStarPathfinder:
    """A* algoritması ile rota bulma sınıfı"""
    
    def __init__(self, fleet: DroneFleet, deliveries: List[DeliveryPoint], 
                 no_fly_zones: List[NoFlyZone]):
        self.fleet = fleet
        self.deliveries = deliveries
        self.no_fly_zones = no_fly_zones
        self.graph = Graph(deliveries, no_fly_zones)
        self.current_time = 0
    
    def find_optimal_routes(self) -> Dict[int, List[Tuple[float, float]]]:
        """Tüm drone'lar için optimal rotaları bulur"""
        routes = {}
        unassigned_deliveries = self.deliveries.copy()
        
        # Her drone için en iyi rotayı bul
        for drone in self.fleet.drones.values():
            route = self._find_route_for_drone(drone, unassigned_deliveries)
            if route:
                routes[drone.drone_id] = route
                # Atanan teslimatları listeden çıkar
                for pos in route[1:]:  # İlk pozisyon başlangıç noktası
                    delivery = self._get_delivery_at_position(pos, unassigned_deliveries)
                    if delivery:
                        unassigned_deliveries.remove(delivery)
        
        return routes
    
    def _find_route_for_drone(self, drone: Drone, 
                             available_deliveries: List[DeliveryPoint]) -> Optional[List[Tuple[float, float]]]:
        """Belirli bir drone için en iyi rotayı bulur"""
        if not available_deliveries:
            return None
        
        # Drone kapasitesine uygun teslimatları filtrele
        suitable_deliveries = [d for d in available_deliveries 
                              if d.weight <= drone.max_weight]
        
        if not suitable_deliveries:
            return None
        
        # En yüksek öncelikli teslimatlardan başla
        suitable_deliveries.sort(key=lambda d: d.priority, reverse=True)
        
        route = [drone.start_pos]
        current_pos = drone.start_pos
        current_load = 0
        current_battery = drone.battery
        assigned_deliveries = []
        
        # Greedy yaklaşım ile rotayı oluştur
        while suitable_deliveries:
            best_delivery = None
            best_cost = float('inf')
            
            for delivery in suitable_deliveries:
                if current_load + delivery.weight <= drone.max_weight:
                    distance = Drone.calculate_distance(current_pos, delivery.position)
                    energy_needed = drone.calculate_energy_consumption(distance)
                    
                    if current_battery >= energy_needed:
                        cost = self._calculate_cost(current_pos, delivery, distance)
                        if cost < best_cost:
                            best_cost = cost
                            best_delivery = delivery
            
            if best_delivery:
                route.append(best_delivery.position)
                current_pos = best_delivery.position
                current_load += best_delivery.weight
                
                distance = Drone.calculate_distance(route[-2], current_pos)
                current_battery -= drone.calculate_energy_consumption(distance)
                
                suitable_deliveries.remove(best_delivery)
                assigned_deliveries.append(best_delivery)
            else:
                break
        
        return route if len(route) > 1 else None
    
    def _calculate_cost(self, current_pos: Tuple[float, float], 
                       delivery: DeliveryPoint, distance: float) -> float:
        """Teslimat maliyetini hesaplar"""
        # Maliyet fonksiyonu: distance × weight + (priority × 100)
        base_cost = distance * delivery.weight + (delivery.priority * 100)
        
        # No-fly zone cezası
        penalty = 0
        for zone in self.no_fly_zones:
            if zone.is_active(self.current_time):
                if zone.intersects_path(current_pos, delivery.position):
                    penalty += zone.get_penalty_score()
        
        return base_cost + penalty
    
    def _calculate_heuristic(self, current_pos: Tuple[float, float], 
                           target_pos: Tuple[float, float]) -> float:
        """A* için heuristic fonksiyonunu hesaplar"""
        # Heuristic = distance + no_fly_zone_penalty
        distance = Drone.calculate_distance(current_pos, target_pos)
        
        penalty = 0
        for zone in self.no_fly_zones:
            if zone.is_active(self.current_time):
                if zone.intersects_path(current_pos, target_pos):
                    penalty += 500  # No-fly zone cezası
        
        return distance + penalty
    
    def find_path_astar(self, start: Tuple[float, float], 
                       goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """A* algoritması ile iki nokta arasında optimal yol bulur"""
        open_set = []
        closed_set = set()
        
        # Başlangıç düğümü
        start_node = Node(start)
        start_node.g_cost = 0
        start_node.h_cost = self._calculate_heuristic(start, goal)
        start_node.f_cost = start_node.g_cost + start_node.h_cost
        
        heapq.heappush(open_set, start_node)
        node_dict = {start: start_node}
        
        while open_set:
            current_node = heapq.heappop(open_set)
            
            if current_node.position in closed_set:
                continue
                
            closed_set.add(current_node.position)
            
            # Hedefe ulaştık mı?
            if current_node.position == goal:
                return self._reconstruct_path(current_node)
            
            # Komşuları kontrol et
            for neighbor_pos in self.graph.get_neighbors(current_node.position):
                if neighbor_pos in closed_set:
                    continue
                
                distance = Drone.calculate_distance(current_node.position, neighbor_pos)
                tentative_g = current_node.g_cost + distance
                
                if neighbor_pos not in node_dict:
                    neighbor_node = Node(neighbor_pos)
                    node_dict[neighbor_pos] = neighbor_node
                else:
                    neighbor_node = node_dict[neighbor_pos]
                
                if tentative_g < neighbor_node.g_cost:
                    neighbor_node.parent = current_node
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.h_cost = self._calculate_heuristic(neighbor_pos, goal)
                    neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost
                    
                    heapq.heappush(open_set, neighbor_node)
        
        return None  # Yol bulunamadı
    
    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """Düğümden geriye doğru yolu yeniden oluşturur"""
        path = []
        current = node
        while current:
            path.append(current.position)
            current = current.parent
        return path[::-1]  # Ters çevir
    
    def _get_delivery_at_position(self, position: Tuple[float, float], 
                                 deliveries: List[DeliveryPoint]) -> Optional[DeliveryPoint]:
        """Belirli pozisyondaki teslimat noktasını bulur"""
        for delivery in deliveries:
            if delivery.position == position:
                return delivery
        return None
    
    def calculate_route_metrics(self, route: List[Tuple[float, float]]) -> Dict:
        """Rota metriklerini hesaplar"""
        if len(route) < 2:
            return {"distance": 0, "energy": 0, "deliveries": 0}
        
        total_distance = 0
        for i in range(len(route) - 1):
            distance = Drone.calculate_distance(route[i], route[i + 1])
            total_distance += distance
        
        # Basit enerji modeli
        total_energy = total_distance * 0.1
        deliveries_count = len(route) - 1  # Başlangıç noktası hariç
        
        return {
            "distance": total_distance,
            "energy": total_energy,
            "deliveries": deliveries_count
        } 