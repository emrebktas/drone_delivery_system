import copy
from typing import List, Dict, Tuple, Set, Optional
from drone_system import DroneFleet, DeliveryPoint, NoFlyZone, Drone

class CSPVariable:
    
    def __init__(self, delivery: DeliveryPoint):
        self.delivery = delivery
        self.domain = []
        self.assigned_drone = None
        
    def assign(self, drone_id: int):
        if drone_id in self.domain:
            self.assigned_drone = drone_id
            return True
        return False
    
    def unassign(self):
        self.assigned_drone = None

class CSPConstraint:
    
    def __init__(self, constraint_type: str, variables: List[CSPVariable], 
                 additional_data: Dict = None):
        self.constraint_type = constraint_type
        self.variables = variables
        self.additional_data = additional_data or {}
    
    def is_satisfied(self, fleet: DroneFleet, no_fly_zones: List[NoFlyZone], 
                    current_time: int = 0) -> bool:
        
        if self.constraint_type == "weight_capacity":
            return self._check_weight_capacity(fleet)
        elif self.constraint_type == "battery_capacity":
            return self._check_battery_capacity(fleet)
        elif self.constraint_type == "time_window":
            return self._check_time_window(current_time)
        elif self.constraint_type == "no_fly_zone":
            return self._check_no_fly_zone(no_fly_zones, current_time)
        elif self.constraint_type == "unique_assignment":
            return self._check_unique_assignment()
        
        return True
    
    def _check_weight_capacity(self, fleet: DroneFleet) -> bool:
        drone_loads = {}
        
        for variable in self.variables:
            if variable.assigned_drone is not None:
                drone_id = variable.assigned_drone
                if drone_id not in drone_loads:
                    drone_loads[drone_id] = 0
                drone_loads[drone_id] += variable.delivery.weight
        
        for drone_id, total_load in drone_loads.items():
            drone = fleet.get_drone(drone_id)
            if drone and total_load > drone.max_weight:
                return False
        
        return True
    
    def _check_battery_capacity(self, fleet: DroneFleet) -> bool:
        drone_routes = {}
        
        for variable in self.variables:
            if variable.assigned_drone is not None:
                drone_id = variable.assigned_drone
                if drone_id not in drone_routes:
                    drone = fleet.get_drone(drone_id)
                    drone_routes[drone_id] = [drone.start_pos] if drone else []
                
                drone_routes[drone_id].append(variable.delivery.position)
        
        for drone_id, route in drone_routes.items():
            drone = fleet.get_drone(drone_id)
            if not drone:
                continue
                
            total_energy = 0
            for i in range(len(route) - 1):
                distance = Drone.calculate_distance(route[i], route[i + 1])
                total_energy += distance * 0.1
            
            if total_energy > drone.battery:
                return False
        
        return True
    
    def _check_time_window(self, current_time: int) -> bool:
        for variable in self.variables:
            if variable.assigned_drone is not None:
                delivery = variable.delivery
                if not delivery.is_within_time_window(current_time):
                    return False
        return True
    
    def _check_no_fly_zone(self, no_fly_zones: List[NoFlyZone], current_time: int) -> bool:
        for variable in self.variables:
            if variable.assigned_drone is not None:
                for zone in no_fly_zones:
                    if zone.is_active(current_time):
                        if zone.contains_point(variable.delivery.position):
                            return False
        return True
    
    def _check_unique_assignment(self) -> bool:
        assigned_deliveries = set()
        
        for variable in self.variables:
            if variable.assigned_drone is not None:
                delivery_id = variable.delivery.delivery_id
                if delivery_id in assigned_deliveries:
                    return False
                assigned_deliveries.add(delivery_id)
        
        return True

class CSPSolver:
    
    def __init__(self, fleet: DroneFleet, deliveries: List[DeliveryPoint], 
                 no_fly_zones: List[NoFlyZone]):
        self.fleet = fleet
        self.deliveries = deliveries
        self.no_fly_zones = no_fly_zones
        self.variables = []
        self.constraints = []
        self.current_time = 0
        
        self._initialize_variables()
        self._initialize_constraints()
    
    def _initialize_variables(self):
        self.variables = []
        
        for delivery in self.deliveries:
            variable = CSPVariable(delivery)
            
            for drone in self.fleet.drones.values():
                if self._is_drone_suitable(drone, delivery):
                    variable.domain.append(drone.drone_id)
            
            self.variables.append(variable)
    
    def _initialize_constraints(self):
        self.constraints = []
        
        self.constraints.append(CSPConstraint("weight_capacity", self.variables))
        
        self.constraints.append(CSPConstraint("battery_capacity", self.variables))
        
        self.constraints.append(CSPConstraint("time_window", self.variables))
        
        self.constraints.append(CSPConstraint("no_fly_zone", self.variables))
        
        self.constraints.append(CSPConstraint("unique_assignment", self.variables))
    
    def _is_drone_suitable(self, drone: Drone, delivery: DeliveryPoint) -> bool:
        if delivery.weight > drone.max_weight:
            return False
        
        distance = Drone.calculate_distance(drone.start_pos, delivery.position)
        energy_needed = distance * 0.1
        if energy_needed > drone.battery:
            return False
        
        return True
    
    def solve(self) -> Optional[Dict[int, List[Tuple[float, float]]]]:
        print("CSP çözümü başlatılıyor...")
        
        if self._backtrack(0):
            print("CSP çözümü bulundu!")
            return self._convert_to_routes()
        else:
            print("CSP çözümü bulunamadı!")
            return None
    
    def _backtrack(self, variable_index: int) -> bool:
        if variable_index >= len(self.variables):
            return self._check_all_constraints()
        
        variable = self.variables[variable_index]
        
        for drone_id in variable.domain:
            variable.assign(drone_id)
            
            if self._is_consistent():
                if self._backtrack(variable_index + 1):
                    return True
            
            variable.unassign()
        
        return False
    
    def _is_consistent(self) -> bool:
        for constraint in self.constraints:
            if constraint.constraint_type in ["weight_capacity", "unique_assignment"]:
                if not constraint.is_satisfied(self.fleet, self.no_fly_zones, self.current_time):
                    return False
        return True
    
    def _check_all_constraints(self) -> bool:
        for constraint in self.constraints:
            if not constraint.is_satisfied(self.fleet, self.no_fly_zones, self.current_time):
                return False
        return True
    
    def _convert_to_routes(self) -> Dict[int, List[Tuple[float, float]]]:
        routes = {}
        
        for drone in self.fleet.drones.values():
            routes[drone.drone_id] = [drone.start_pos]
        
        for variable in self.variables:
            if variable.assigned_drone is not None:
                drone_id = variable.assigned_drone
                routes[drone_id].append(variable.delivery.position)
        
        return {drone_id: route for drone_id, route in routes.items() if len(route) > 1}
    
    def solve_with_forward_checking(self) -> Optional[Dict[int, List[Tuple[float, float]]]]:
        print("CSP çözümü (Forward Checking) başlatılıyor...")
        
        original_domains = {}
        for i, variable in enumerate(self.variables):
            original_domains[i] = variable.domain.copy()
        
        if self._backtrack_with_fc(0, original_domains):
            print("CSP çözümü (FC) bulundu!")
            return self._convert_to_routes()
        else:
            print("CSP çözümü (FC) bulunamadı!")
            return None
    
    def _backtrack_with_fc(self, variable_index: int, domains: Dict) -> bool:
        if variable_index >= len(self.variables):
            return self._check_all_constraints()
        
        variable = self.variables[variable_index]
        
        for drone_id in domains[variable_index]:
            variable.assign(drone_id)
            
            if self._is_consistent():
                new_domains = self._forward_check(variable_index, drone_id, domains)
                
                if new_domains is not None:
                    if self._backtrack_with_fc(variable_index + 1, new_domains):
                        return True
            
            variable.unassign()
        
        return False
    
    def _forward_check(self, assigned_var_index: int, assigned_drone_id: int, 
                      current_domains: Dict) -> Optional[Dict]:
        new_domains = {}
        
        for i, variable in enumerate(self.variables):
            if i <= assigned_var_index:
                new_domains[i] = current_domains[i].copy()
            else:
                new_domains[i] = []
                
                for drone_id in current_domains[i]:
                    temp_assignment = variable.assigned_drone
                    variable.assign(drone_id)
                    
                    if self._is_consistent():
                        new_domains[i].append(drone_id)
                    
                    if temp_assignment is not None:
                        variable.assign(temp_assignment)
                    else:
                        variable.unassign()
                
                if not new_domains[i]:
                    return None
        
        return new_domains
    
    def get_solution_metrics(self) -> Dict:
        if not self.variables:
            return {}
        
        assigned_count = sum(1 for var in self.variables if var.assigned_drone is not None)
        total_count = len(self.variables)
        
        drone_usage = {}
        for var in self.variables:
            if var.assigned_drone is not None:
                drone_id = var.assigned_drone
                drone_usage[drone_id] = drone_usage.get(drone_id, 0) + 1
        
        return {
            "assigned_deliveries": assigned_count,
            "total_deliveries": total_count,
            "assignment_ratio": assigned_count / total_count if total_count > 0 else 0,
            "drone_usage": drone_usage,
            "active_drones": len(drone_usage)
        }