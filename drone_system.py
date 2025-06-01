"""
Drone Sistemi - Temel sınıflar ve veri yapıları
Bu modül drone'lar, teslimat noktaları ve uçuş yasağı bölgeleri için
temel sınıfları içerir.
"""

import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

@dataclass
class Drone:
    """Drone sınıfı - bir drone'un özelliklerini temsil eder"""
    
    def __init__(self, drone_id: int, max_weight: float, battery: int, 
                 speed: float, start_pos: Tuple[float, float]):
        self.drone_id = drone_id
        self.max_weight = max_weight  # kg
        self.battery = battery        # mAh
        self.speed = speed           # m/s
        self.start_pos = start_pos   # (x, y)
        self.current_pos = start_pos
        self.current_battery = battery
        self.current_load = 0.0      # Şu anki yük
        self.assigned_deliveries = []
        self.route = [start_pos]     # Takip edilen rota
        
    def can_carry(self, weight: float) -> bool:
        """Drone'un belirli bir ağırlığı taşıyıp taşıyamayacağını kontrol eder"""
        return (self.current_load + weight) <= self.max_weight
    
    def add_delivery(self, delivery_point):
        """Drone'a teslimat noktası atar"""
        if self.can_carry(delivery_point.weight):
            self.assigned_deliveries.append(delivery_point)
            self.current_load += delivery_point.weight
            return True
        return False
    
    def calculate_energy_consumption(self, distance: float) -> float:
        """Mesafeye göre enerji tüketimini hesaplar"""
        # Temel enerji tüketimi + yük faktörü
        base_consumption = distance * 0.1  # mAh/metre
        load_factor = 1 + (self.current_load / self.max_weight) * 0.5
        return base_consumption * load_factor
    
    def can_reach(self, position: Tuple[float, float]) -> bool:
        """Drone'un belirli bir pozisyona ulaşıp ulaşamayacağını kontrol eder"""
        distance = self.calculate_distance(self.current_pos, position)
        required_energy = self.calculate_energy_consumption(distance)
        return self.current_battery >= required_energy
    
    @staticmethod
    def calculate_distance(pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """İki nokta arasındaki Öklid mesafesini hesaplar"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def move_to(self, position: Tuple[float, float]) -> bool:
        """Drone'u belirli bir pozisyona taşır"""
        if self.can_reach(position):
            distance = self.calculate_distance(self.current_pos, position)
            energy_consumed = self.calculate_energy_consumption(distance)
            
            self.current_pos = position
            self.current_battery -= energy_consumed
            self.route.append(position)
            return True
        return False
    
    def reset(self):
        """Drone'u başlangıç durumuna sıfırlar"""
        self.current_pos = self.start_pos
        self.current_battery = self.battery
        self.current_load = 0.0
        self.assigned_deliveries = []
        self.route = [self.start_pos]

class DeliveryPoint:
    """Teslimat noktası sınıfı"""
    
    def __init__(self, delivery_id: int, position: Tuple[float, float], 
                 weight: float, priority: int, time_window: Tuple[int, int]):
        self.delivery_id = delivery_id
        self.position = position      # (x, y)
        self.weight = weight         # kg
        self.priority = priority     # 1-5 arası (5 en yüksek)
        self.time_window = time_window  # (başlangıç, bitiş) dakika
        self.is_delivered = False
        self.delivery_time = None
        
    def calculate_cost(self, distance: float) -> float:
        """Teslimat maliyetini hesaplar"""
        # Maliyet = mesafe × ağırlık + (öncelik × 100)
        return distance * self.weight + (self.priority * 100)
    
    def is_within_time_window(self, current_time: int) -> bool:
        """Teslimatın zaman penceresi içinde olup olmadığını kontrol eder"""
        return self.time_window[0] <= current_time <= self.time_window[1]
    
    def get_priority_multiplier(self) -> float:
        """Öncelik çarpanını döndürür"""
        return 1.0 + (self.priority - 1) * 0.2

class NoFlyZone:
    """Uçuş yasağı bölgesi sınıfı"""
    
    def __init__(self, zone_id: int, coordinates: List[Tuple[float, float]], 
                 active_time: Tuple[int, int]):
        self.zone_id = zone_id
        self.coordinates = coordinates  # Çokgen köşe noktaları
        self.active_time = active_time  # (başlangıç, bitiş) dakika
        
    def is_active(self, current_time: int) -> bool:
        """Bölgenin belirli zamanda aktif olup olmadığını kontrol eder"""
        return self.active_time[0] <= current_time <= self.active_time[1]
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """Bir noktanın bölge içinde olup olmadığını kontrol eder (Ray casting algoritması)"""
        x, y = point
        n = len(self.coordinates)
        inside = False
        
        p1x, p1y = self.coordinates[0]
        for i in range(1, n + 1):
            p2x, p2y = self.coordinates[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def intersects_path(self, start: Tuple[float, float], 
                       end: Tuple[float, float]) -> bool:
        """Bir yolun bölge ile kesişip kesişmediğini kontrol eder"""
        # Basit bir yaklaşım: başlangıç ve bitiş noktalarını kontrol et
        # Gerçek uygulamada çizgi-çokgen kesişimi algoritması kullanılmalı
        return self.contains_point(start) or self.contains_point(end)
    
    def get_penalty_score(self) -> float:
        """Bölge ihlali ceza puanını döndürür"""
        return 1000.0  # Yüksek ceza

class DroneFleet:
    """Drone filosu yönetim sınıfı"""
    
    def __init__(self):
        self.drones: Dict[int, Drone] = {}
        
    def add_drone(self, drone_id: int, max_weight: float, battery: int, 
                  speed: float, start_pos: Tuple[float, float]):
        """Filoya drone ekler"""
        drone = Drone(drone_id, max_weight, battery, speed, start_pos)
        self.drones[drone_id] = drone
        
    def get_drone(self, drone_id: int) -> Optional[Drone]:
        """Belirli ID'ye sahip drone'u döndürür"""
        return self.drones.get(drone_id)
    
    def get_available_drones(self) -> List[Drone]:
        """Müsait drone'ları döndürür"""
        return [drone for drone in self.drones.values() 
                if len(drone.assigned_deliveries) == 0]
    
    def reset_all_drones(self):
        """Tüm drone'ları sıfırlar"""
        for drone in self.drones.values():
            drone.reset()
    
    def get_fleet_status(self) -> Dict:
        """Filo durumunu döndürür"""
        total_capacity = sum(drone.max_weight for drone in self.drones.values())
        total_battery = sum(drone.battery for drone in self.drones.values())
        active_drones = len([d for d in self.drones.values() 
                           if len(d.assigned_deliveries) > 0])
        
        return {
            "total_drones": len(self.drones),
            "active_drones": active_drones,
            "total_capacity": total_capacity,
            "total_battery": total_battery,
            "average_speed": sum(d.speed for d in self.drones.values()) / len(self.drones)
        } 