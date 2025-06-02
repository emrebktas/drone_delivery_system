import random
import copy
from typing import List, Dict, Tuple, Optional
from drone_system import DroneFleet, DeliveryPoint, NoFlyZone, Drone

class Individual:
    
    def __init__(self, routes: Dict[int, List[Tuple[float, float]]] = None):
        self.routes = routes if routes else {}
        self.fitness = 0.0
        self.completed_deliveries = 0
        self.total_energy = 0.0
        self.constraint_violations = 0
    
    def calculate_fitness(self, fleet: DroneFleet, deliveries: List[DeliveryPoint], 
                         no_fly_zones: List[NoFlyZone]) -> float:
        
        self.completed_deliveries = 0
        self.total_energy = 0.0
        self.constraint_violations = 0
        
        for drone_id, route in self.routes.items():
            if len(route) > 1:
                self.completed_deliveries += len(route) - 1
                
                for i in range(len(route) - 1):
                    distance = Drone.calculate_distance(route[i], route[i + 1])
                    self.total_energy += distance * 0.1
                
                drone = fleet.get_drone(drone_id)
                if drone:
                    violations = self._check_constraints(drone, route, deliveries, no_fly_zones)
                    self.constraint_violations += violations
        
        self.fitness = (self.completed_deliveries * 50) - (self.total_energy * 0.1) - (self.constraint_violations * 1000)
        return self.fitness
    
    def _check_constraints(self, drone: Drone, route: List[Tuple[float, float]], 
                          deliveries: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]) -> int:
        violations = 0
        current_load = 0
        current_battery = drone.battery
        current_time = 0
        
        for i, position in enumerate(route):
            if i == 0:
                continue
                
            delivery = self._find_delivery_at_position(position, deliveries)
            if not delivery:
                violations += 1
                continue
            
            if current_load + delivery.weight > drone.max_weight:
                violations += 1
            else:
                current_load += delivery.weight
            
            if i > 0:
                distance = Drone.calculate_distance(route[i-1], position)
                energy_needed = distance * 0.1
                if current_battery < energy_needed:
                    violations += 1
                else:
                    current_battery -= energy_needed
                    current_time += distance / drone.speed
            
            if not delivery.is_within_time_window(int(current_time)):
                violations += 1
            
            if i > 0:
                for zone in no_fly_zones:
                    if zone.is_active(int(current_time)):
                        if zone.intersects_path(route[i-1], position):
                            violations += 1
        
        return violations
    
    def _find_delivery_at_position(self, position: Tuple[float, float], 
                                  deliveries: List[DeliveryPoint]) -> Optional[DeliveryPoint]:
        for delivery in deliveries:
            if delivery.position == position:
                return delivery
        return None

class GeneticAlgorithm:
    
    def __init__(self, fleet: DroneFleet, deliveries: List[DeliveryPoint], 
                 no_fly_zones: List[NoFlyZone], population_size: int = 50, 
                 generations: int = 100, mutation_rate: float = 0.1, 
                 crossover_rate: float = 0.8):
        self.fleet = fleet
        self.deliveries = deliveries
        self.no_fly_zones = no_fly_zones
        self.population_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.population = []
        self.best_individual = None
        self.fitness_history = []
    
    def evolve(self) -> Dict[int, List[Tuple[float, float]]]:
        print(f"GA başlatılıyor: Popülasyon={self.population_size}, Nesil={self.generations}")
        
        self._initialize_population()
        
        for generation in range(self.generations):
            self._evaluate_population()
            
            current_best = max(self.population, key=lambda x: x.fitness)
            if not self.best_individual or current_best.fitness > self.best_individual.fitness:
                self.best_individual = copy.deepcopy(current_best)
            
            self.fitness_history.append(self.best_individual.fitness)
            
            if generation % 20 == 0:
                print(f"Nesil {generation}: En İyi Fitness = {self.best_individual.fitness:.2f}")
            
            new_population = []
            
            elite_count = max(1, self.population_size // 10)
            sorted_population = sorted(self.population, key=lambda x: x.fitness, reverse=True)
            new_population.extend(copy.deepcopy(sorted_population[:elite_count]))
            
            while len(new_population) < self.population_size:
                if random.random() < self.crossover_rate:
                    parent1 = self._tournament_selection()
                    parent2 = self._tournament_selection()
                    child1, child2 = self._crossover(parent1, parent2)
                    
                    if random.random() < self.mutation_rate:
                        self._mutate(child1)
                    if random.random() < self.mutation_rate:
                        self._mutate(child2)
                    
                    new_population.extend([child1, child2])
                else:
                    individual = copy.deepcopy(self._tournament_selection())
                    if random.random() < self.mutation_rate:
                        self._mutate(individual)
                    new_population.append(individual)
            
            self.population = new_population[:self.population_size]
        
        print(f"GA tamamlandı. En iyi fitness: {self.best_individual.fitness:.2f}")
        return self.best_individual.routes if self.best_individual else {}
    
    def _initialize_population(self):
        self.population = []
        
        for _ in range(self.population_size):
            individual = self._create_random_individual()
            self.population.append(individual)
    
    def _create_random_individual(self) -> Individual:
        routes = {}
        available_deliveries = self.deliveries.copy()
        random.shuffle(available_deliveries)
        
        drone_list = list(self.fleet.drones.values())
        
        for drone in drone_list:
            route = [drone.start_pos]
            current_load = 0
            current_battery = drone.battery
            
            deliveries_to_remove = []
            for delivery in available_deliveries:
                if current_load + delivery.weight <= drone.max_weight:
                    distance = Drone.calculate_distance(route[-1], delivery.position)
                    energy_needed = distance * 0.1
                    
                    if current_battery >= energy_needed:
                        route.append(delivery.position)
                        current_load += delivery.weight
                        current_battery -= energy_needed
                        deliveries_to_remove.append(delivery)
                        
                        if len(route) > 8:
                            break
            
            for delivery in deliveries_to_remove:
                available_deliveries.remove(delivery)
            
            routes[drone.drone_id] = route
        
        return Individual(routes)
    
    def _evaluate_population(self):
        for individual in self.population:
            individual.calculate_fitness(self.fleet, self.deliveries, self.no_fly_zones)
    
    def _tournament_selection(self) -> Individual:
        tournament_size = 3
        tournament = random.sample(self.population, tournament_size)
        return max(tournament, key=lambda x: x.fitness)
    
    def _crossover(self, parent1: Individual, parent2: Individual) -> Tuple[Individual, Individual]:
        child1_routes = {}
        child2_routes = {}
        
        drone_ids = list(self.fleet.drones.keys())
        
        for drone_id in drone_ids:
            if random.random() < 0.5:
                child1_routes[drone_id] = copy.deepcopy(parent1.routes.get(drone_id, []))
                child2_routes[drone_id] = copy.deepcopy(parent2.routes.get(drone_id, []))
            else:
                child1_routes[drone_id] = copy.deepcopy(parent2.routes.get(drone_id, []))
                child2_routes[drone_id] = copy.deepcopy(parent1.routes.get(drone_id, []))
        
        self._fix_duplicate_deliveries(child1_routes)
        self._fix_duplicate_deliveries(child2_routes)
        
        return Individual(child1_routes), Individual(child2_routes)
    
    def _fix_duplicate_deliveries(self, routes: Dict[int, List[Tuple[float, float]]]):
        seen_positions = set()
        
        for drone_id, route in routes.items():
            new_route = [route[0]] if route else []
            
            for position in route[1:]:
                if position not in seen_positions:
                    new_route.append(position)
                    seen_positions.add(position)
            
            routes[drone_id] = new_route
    
    def _mutate(self, individual: Individual):
        if not individual.routes:
            return
        
        mutation_type = random.choice(['swap', 'insert', 'remove'])
        
        if mutation_type == 'swap':
            self._swap_mutation(individual)
        elif mutation_type == 'insert':
            self._insert_mutation(individual)
        elif mutation_type == 'remove':
            self._remove_mutation(individual)
    
    def _swap_mutation(self, individual: Individual):
        all_positions = []
        position_to_drone = {}
        
        for drone_id, route in individual.routes.items():
            for i, pos in enumerate(route[1:], 1):
                all_positions.append((drone_id, i, pos))
                position_to_drone[pos] = drone_id
        
        if len(all_positions) >= 2:
            pos1_info, pos2_info = random.sample(all_positions, 2)
            
            drone1_id, idx1, pos1 = pos1_info
            drone2_id, idx2, pos2 = pos2_info
            
            individual.routes[drone1_id][idx1] = pos2
            individual.routes[drone2_id][idx2] = pos1
    
    def _insert_mutation(self, individual: Individual):
        available_deliveries = []
        used_positions = set()
        
        for route in individual.routes.values():
            used_positions.update(route[1:])
        
        for delivery in self.deliveries:
            if delivery.position not in used_positions:
                available_deliveries.append(delivery)
        
        if available_deliveries and individual.routes:
            delivery = random.choice(available_deliveries)
            drone_id = random.choice(list(individual.routes.keys()))
            
            drone = self.fleet.get_drone(drone_id)
            if drone and len(individual.routes[drone_id]) < 8:
                individual.routes[drone_id].append(delivery.position)
    
    def _remove_mutation(self, individual: Individual):
        non_empty_routes = [(drone_id, route) for drone_id, route in individual.routes.items() 
                           if len(route) > 1]
        
        if non_empty_routes:
            drone_id, route = random.choice(non_empty_routes)
            if len(route) > 1:
                idx_to_remove = random.randint(1, len(route) - 1)
                individual.routes[drone_id].pop(idx_to_remove)
    
    def get_statistics(self) -> Dict:
        if not self.best_individual:
            return {}
        
        return {
            "best_fitness": self.best_individual.fitness,
            "completed_deliveries": self.best_individual.completed_deliveries,
            "total_energy": self.best_individual.total_energy,
            "constraint_violations": self.best_individual.constraint_violations,
            "fitness_history": self.fitness_history
        } 