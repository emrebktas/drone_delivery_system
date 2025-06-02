import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass

@dataclass
class Drone:
    
    def __init__(self, drone_id: int, max_weight: float, battery: int, 
                 speed: float, start_pos: Tuple[float, float]):
        self.drone_id = drone_id
        self.max_weight = max_weight
        self.battery = battery
        self.speed = speed
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.current_battery = battery
        self.current_load = 0.0
        self.assigned_deliveries = []
        self.route = [start_pos]

    def can_carry(self, weight: float) -> bool:
        return (self.current_load + weight) <= self.max_weight
    
    def add_delivery(self, delivery_point):
        if self.can_carry(delivery_point.weight):
            self.assigned_deliveries.append(delivery_point)
            self.current_load += delivery_point.weight
            return True
        return False
    
    def calculate_energy_consumption(self, distance: float) -> float:
        base_consumption = distance * 0.1
        load_factor = 1 + (self.current_load / self.max_weight) * 0.5
        return base_consumption * load_factor

    def can_reach(self, position: Tuple[float, float]) -> bool:
        distance = self.calculate_distance(self.current_pos, position)
        required_energy = self.calculate_energy_consumption(distance)
        return self.current_battery >= required_energy
    
    @staticmethod
    def calculate_distance(pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def move_to(self, position: Tuple[float, float]) -> bool:
        if self.can_reach(position):
            distance = self.calculate_distance(self.current_pos, position)
            energy_consumed = self.calculate_energy_consumption(distance)
            
            self.current_pos = position
            self.current_battery -= energy_consumed
            self.route.append(position)
            return True
        return False
    
    def reset(self):
        self.current_pos = self.start_pos
        self.current_battery = self.battery
        self.current_load = 0.0
        self.assigned_deliveries = []
        self.route = [self.start_pos]

class DeliveryPoint:
    
    def __init__(self, delivery_id: int, position: Tuple[float, float], 
                 weight: float, priority: int, time_window: Tuple[int, int]):
        self.delivery_id = delivery_id
        self.position = position
        self.weight = weight
        self.priority = priority
        self.time_window = time_window
        self.is_delivered = False
        self.delivery_time = None

    def calculate_cost(self, distance: float) -> float:
        return distance * self.weight + (self.priority * 100)
    
    def is_within_time_window(self, current_time: int) -> bool:
        return self.time_window[0] <= current_time <= self.time_window[1]
    
    def get_priority_multiplier(self) -> float:
        return 1.0 + (self.priority - 1) * 0.2

class NoFlyZone:
    
    def __init__(self, zone_id: int, coordinates: List[Tuple[float, float]], 
                 active_time: Tuple[int, int]):
        self.zone_id = zone_id
        self.coordinates = coordinates
        self.active_time = active_time

    def is_active(self, current_time: int) -> bool:
        return self.active_time[0] <= current_time <= self.active_time[1]
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
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
        return self.contains_point(start) or self.contains_point(end)

    def get_penalty_score(self) -> float:
        return 1000.0

class DroneFleet:

    def __init__(self):
        self.drones: Dict[int, Drone] = {}
        
    def add_drone(self, drone_id: int, max_weight: float, battery: int, 
                  speed: float, start_pos: Tuple[float, float]):
        drone = Drone(drone_id, max_weight, battery, speed, start_pos)
        self.drones[drone_id] = drone
        
    def get_drone(self, drone_id: int) -> Optional[Drone]:
        return self.drones.get(drone_id)
    
    def get_available_drones(self) -> List[Drone]:
        return [drone for drone in self.drones.values() 
                if len(drone.assigned_deliveries) == 0]
    
    def reset_all_drones(self):
        for drone in self.drones.values():
            drone.reset()
    
    def get_fleet_status(self) -> Dict:
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