"""
Rastgele Veri Üreteci Modülü
Bu modül test senaryoları için rastgele drone, teslimat noktası ve
no-fly zone verisi üretir.
"""

import random
import math
from typing import List, Dict, Tuple

class DataGenerator:
    """Rastgele veri üreteci sınıfı"""
    
    def __init__(self, seed: int = None):
        if seed:
            random.seed(seed)
        
        # Veri üretimi için parametreler
        self.map_width = 100
        self.map_height = 100
        self.min_zone_size = 10
        self.max_zone_size = 25
    
    def generate_drones(self, count: int) -> List[Dict]:
        """Rastgele drone verisi üretir"""
        drones = []
        
        for i in range(1, count + 1):
            drone = {
                "id": i,
                "max_weight": round(random.uniform(2.0, 8.0), 1),  # 2-8 kg
                "battery": random.randint(8000, 25000),           # 8000-25000 mAh
                "speed": round(random.uniform(5.0, 15.0), 1),     # 5-15 m/s
                "start_pos": self._generate_random_position()
            }
            drones.append(drone)
        
        return drones
    
    def generate_deliveries(self, count: int) -> List[Dict]:
        """Rastgele teslimat noktası verisi üretir"""
        deliveries = []
        
        for i in range(1, count + 1):
            delivery = {
                "id": i,
                "pos": self._generate_random_position(),
                "weight": round(random.uniform(0.5, 5.0), 1),     # 0.5-5 kg
                "priority": random.randint(1, 5),                 # 1-5 öncelik
                "time_window": self._generate_time_window()
            }
            deliveries.append(delivery)
        
        return deliveries
    
    def generate_no_fly_zones(self, count: int) -> List[Dict]:
        """Rastgele no-fly zone verisi üretir"""
        zones = []
        
        for i in range(1, count + 1):
            zone = {
                "id": i,
                "coordinates": self._generate_zone_coordinates(),
                "active_time": self._generate_active_time()
            }
            zones.append(zone)
        
        return zones
    
    def _generate_random_position(self) -> Tuple[float, float]:
        """Rastgele koordinat üretir"""
        x = round(random.uniform(0, self.map_width), 1)
        y = round(random.uniform(0, self.map_height), 1)
        return (x, y)
    
    def _generate_time_window(self) -> Tuple[int, int]:
        """Rastgele zaman penceresi üretir"""
        start_time = random.randint(0, 60)  # 0-60 dakika
        duration = random.randint(20, 100)   # 20-100 dakika süre
        end_time = start_time + duration
        return (start_time, end_time)
    
    def _generate_active_time(self) -> Tuple[int, int]:
        """No-fly zone için aktif zaman üretir"""
        start_time = random.randint(0, 80)   # 0-80 dakika
        duration = random.randint(30, 60)    # 30-60 dakika süre
        end_time = start_time + duration
        return (start_time, end_time)
    
    def _generate_zone_coordinates(self) -> List[Tuple[float, float]]:
        """Rastgele no-fly zone koordinatları üretir (dikdörtgen)"""
        # Merkez nokta
        center_x = random.uniform(self.min_zone_size, self.map_width - self.min_zone_size)
        center_y = random.uniform(self.min_zone_size, self.map_height - self.min_zone_size)
        
        # Boyutlar
        width = random.uniform(self.min_zone_size, self.max_zone_size)
        height = random.uniform(self.min_zone_size, self.max_zone_size)
        
        # Dikdörtgen köşeleri
        half_width = width / 2
        half_height = height / 2
        
        coordinates = [
            (center_x - half_width, center_y - half_height),  # Sol alt
            (center_x + half_width, center_y - half_height),  # Sağ alt
            (center_x + half_width, center_y + half_height),  # Sağ üst
            (center_x - half_width, center_y + half_height)   # Sol üst
        ]
        
        return coordinates
    
    def generate_scenario_data(self, drone_count: int, delivery_count: int, 
                             zone_count: int) -> Tuple[List[Dict], List[Dict], List[Dict]]:
        """Tam bir senaryo için veri üretir"""
        drones = self.generate_drones(drone_count)
        deliveries = self.generate_deliveries(delivery_count)
        zones = self.generate_no_fly_zones(zone_count)
        
        return drones, deliveries, zones
    
    def generate_clustered_deliveries(self, count: int, cluster_count: int = 3) -> List[Dict]:
        """Kümelenmiş teslimat noktaları üretir (daha gerçekçi)"""
        deliveries = []
        
        # Küme merkezleri oluştur
        cluster_centers = []
        for _ in range(cluster_count):
            center = self._generate_random_position()
            cluster_centers.append(center)
        
        deliveries_per_cluster = count // cluster_count
        remaining = count % cluster_count
        
        delivery_id = 1
        
        for i, center in enumerate(cluster_centers):
            cluster_size = deliveries_per_cluster
            if i < remaining:
                cluster_size += 1
            
            for _ in range(cluster_size):
                # Küme merkezi etrafında rastgele pozisyon
                angle = random.uniform(0, 2 * math.pi)
                radius = random.uniform(5, 20)  # 5-20 metre yarıçap
                
                x = center[0] + radius * math.cos(angle)
                y = center[1] + radius * math.sin(angle)
                
                # Harita sınırları içinde tut
                x = max(0, min(self.map_width, x))
                y = max(0, min(self.map_height, y))
                
                delivery = {
                    "id": delivery_id,
                    "pos": (round(x, 1), round(y, 1)),
                    "weight": round(random.uniform(0.5, 4.0), 1),
                    "priority": random.randint(1, 5),
                    "time_window": self._generate_time_window()
                }
                
                deliveries.append(delivery)
                delivery_id += 1
        
        return deliveries
    
    def generate_high_priority_scenario(self, drone_count: int, delivery_count: int) -> Tuple[List[Dict], List[Dict]]:
        """Yüksek öncelikli teslimatlar içeren senaryo"""
        drones = self.generate_drones(drone_count)
        deliveries = []
        
        # %30'u yüksek öncelikli (4-5)
        high_priority_count = int(delivery_count * 0.3)
        normal_priority_count = delivery_count - high_priority_count
        
        # Yüksek öncelikli teslimatlar
        for i in range(1, high_priority_count + 1):
            delivery = {
                "id": i,
                "pos": self._generate_random_position(),
                "weight": round(random.uniform(0.5, 3.0), 1),  # Daha hafif
                "priority": random.randint(4, 5),              # Yüksek öncelik
                "time_window": self._generate_urgent_time_window()
            }
            deliveries.append(delivery)
        
        # Normal öncelikli teslimatlar
        for i in range(high_priority_count + 1, delivery_count + 1):
            delivery = {
                "id": i,
                "pos": self._generate_random_position(),
                "weight": round(random.uniform(1.0, 5.0), 1),
                "priority": random.randint(1, 3),
                "time_window": self._generate_time_window()
            }
            deliveries.append(delivery)
        
        return drones, deliveries
    
    def _generate_urgent_time_window(self) -> Tuple[int, int]:
        """Acil teslimat için dar zaman penceresi"""
        start_time = random.randint(0, 20)   # Erkenden başla
        duration = random.randint(15, 40)     # Kısa süre
        end_time = start_time + duration
        return (start_time, end_time)
    
    def generate_dynamic_zones(self, count: int, max_time: int = 120) -> List[Dict]:
        """Dinamik no-fly zone'lar (zaman içinde değişen)"""
        zones = []
        
        for i in range(1, count + 1):
            # Rastgele aktivasyon zamanları
            activations = []
            current_time = 0
            
            while current_time < max_time:
                start_time = current_time + random.randint(5, 20)
                if start_time >= max_time:
                    break
                    
                duration = random.randint(10, 30)
                end_time = min(start_time + duration, max_time)
                
                activations.append((start_time, end_time))
                current_time = end_time + random.randint(5, 15)  # Ara verme
            
            # Her aktivasyon için ayrı zone oluştur
            for j, (start, end) in enumerate(activations):
                zone = {
                    "id": i * 100 + j,  # Benzersiz ID
                    "coordinates": self._generate_zone_coordinates(),
                    "active_time": (start, end)
                }
                zones.append(zone)
        
        return zones
    
    def save_scenario_to_file(self, filename: str, drones: List[Dict], 
                             deliveries: List[Dict], zones: List[Dict]):
        """Senaryo verilerini dosyaya kaydeder"""
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("# Rastgele üretilmiş drone teslimat senaryosu\n\n")
            
            f.write("drones = [\n")
            for drone in drones:
                f.write(f"    {drone},\n")
            f.write("]\n\n")
            
            f.write("deliveries = [\n")
            for delivery in deliveries:
                f.write(f"    {delivery},\n")
            f.write("]\n\n")
            
            f.write("no_fly_zones = [\n")
            for zone in zones:
                f.write(f"    {zone},\n")
            f.write("]\n")
    
    def generate_test_scenarios(self):
        """Çeşitli test senaryoları üretir"""
        scenarios = {}
        
        # Küçük senaryo
        scenarios["small"] = self.generate_scenario_data(3, 10, 2)
        
        # Orta senaryo
        scenarios["medium"] = self.generate_scenario_data(7, 25, 4)
        
        # Büyük senaryo
        scenarios["large"] = self.generate_scenario_data(15, 60, 8)
        
        # Kümelenmiş teslimatlar
        drones = self.generate_drones(8)
        clustered_deliveries = self.generate_clustered_deliveries(30, 4)
        zones = self.generate_no_fly_zones(5)
        scenarios["clustered"] = (drones, clustered_deliveries, zones)
        
        # Yüksek öncelikli senaryo
        high_priority_drones, high_priority_deliveries = self.generate_high_priority_scenario(6, 20)
        high_priority_zones = self.generate_no_fly_zones(3)
        scenarios["high_priority"] = (high_priority_drones, high_priority_deliveries, high_priority_zones)
        
        return scenarios

# Test ve kullanım örneği
if __name__ == "__main__":
    generator = DataGenerator(seed=42)  # Tekrarlanabilir sonuçlar için seed
    
    # Test senaryoları üret
    scenarios = generator.generate_test_scenarios()
    
    print("Üretilen test senaryoları:")
    for name, (drones, deliveries, zones) in scenarios.items():
        print(f"{name}: {len(drones)} drone, {len(deliveries)} teslimat, {len(zones)} no-fly zone")
    
    # Örnek dosyaya kaydetme
    drones, deliveries, zones = scenarios["medium"]
    generator.save_scenario_to_file("test_scenario.txt", drones, deliveries, zones)
    print("Örnek senaryo 'test_scenario.txt' dosyasına kaydedildi.") 