"""
Görselleştirme Modülü
Bu modül drone rotalarını, teslimat noktalarının ve no-fly zone'ları
matplotlib kullanarak görselleştirir.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import numpy as np
from typing import List, Dict, Tuple
from drone_system import DroneFleet, DeliveryPoint, NoFlyZone

class DroneVisualizer:
    """Drone sistemi görselleştirme sınıfı"""
    
    def __init__(self, fleet: DroneFleet, deliveries: List[DeliveryPoint], 
                 no_fly_zones: List[NoFlyZone]):
        self.fleet = fleet
        self.deliveries = deliveries
        self.no_fly_zones = no_fly_zones
        
        # Renk paleti
        self.colors = [
            '#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7',
            '#DDA0DD', '#98D8C8', '#F7DC6F', '#BB8FCE', '#85C1E9'
        ]
        
        # Matplotlib ayarları
        plt.style.use('default')
        plt.rcParams['figure.figsize'] = (12, 10)
        plt.rcParams['font.size'] = 10
    
    def plot_routes(self, routes: Dict[int, List[Tuple[float, float]]], 
                   title: str = "Drone Teslimat Rotaları"):
        """Drone rotalarını görselleştirir"""
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # No-fly zone'ları çiz
        self._draw_no_fly_zones(ax)
        
        # Teslimat noktalarını çiz
        self._draw_delivery_points(ax)
        
        # Drone başlangıç noktalarını çiz
        self._draw_drone_start_positions(ax)
        
        # Rotaları çiz
        self._draw_routes(ax, routes)
        
        # Harita düzenlemeleri
        self._setup_map(ax, title)
        
        # Legend ekle
        self._add_legend(ax)
        
        plt.tight_layout()
        plt.show()
    
    def _draw_no_fly_zones(self, ax):
        """No-fly zone'ları çizer"""
        for i, zone in enumerate(self.no_fly_zones):
            # Çokgen oluştur
            polygon = Polygon(zone.coordinates, alpha=0.3, 
                            facecolor='red', edgecolor='darkred', linewidth=2)
            ax.add_patch(polygon)
            
            # Zone ID'sini merkeze yaz
            center_x = sum(coord[0] for coord in zone.coordinates) / len(zone.coordinates)
            center_y = sum(coord[1] for coord in zone.coordinates) / len(zone.coordinates)
            ax.text(center_x, center_y, f'NFZ{zone.zone_id}', 
                   ha='center', va='center', fontweight='bold', 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
    
    def _draw_delivery_points(self, ax):
        """Teslimat noktalarını çizer"""
        for delivery in self.deliveries:
            x, y = delivery.position
            
            # Öncelik seviyesine göre renk ve boyut
            if delivery.priority >= 4:
                color = 'orange'
                size = 120
                marker = '^'  # Üçgen
            elif delivery.priority >= 3:
                color = 'gold'
                size = 100
                marker = 's'  # Kare
            else:
                color = 'lightblue'
                size = 80
                marker = 'o'  # Daire
            
            ax.scatter(x, y, c=color, s=size, marker=marker, 
                      edgecolors='black', linewidth=1, alpha=0.8, zorder=5)
            
            # Teslimat ID'sini yaz
            ax.annotate(f'D{delivery.delivery_id}', (x, y), 
                       xytext=(5, 5), textcoords='offset points',
                       fontsize=8, fontweight='bold')
    
    def _draw_drone_start_positions(self, ax):
        """Drone başlangıç pozisyonlarını çizer"""
        for drone in self.fleet.drones.values():
            x, y = drone.start_pos
            ax.scatter(x, y, c='green', s=200, marker='H', 
                      edgecolors='darkgreen', linewidth=2, zorder=10)
            
            # Drone ID'sini yaz
            ax.annotate(f'Drone{drone.drone_id}', (x, y), 
                       xytext=(0, -15), textcoords='offset points',
                       ha='center', fontsize=9, fontweight='bold',
                       bbox=dict(boxstyle="round,pad=0.2", facecolor='lightgreen', alpha=0.7))
    
    def _draw_routes(self, ax, routes: Dict[int, List[Tuple[float, float]]]):
        """Drone rotalarını çizer"""
        for i, (drone_id, route) in enumerate(routes.items()):
            if len(route) < 2:
                continue
                
            color = self.colors[i % len(self.colors)]
            
            # Rota çizgilerini çiz
            x_coords = [pos[0] for pos in route]
            y_coords = [pos[1] for pos in route]
            
            ax.plot(x_coords, y_coords, color=color, linewidth=2.5, 
                   alpha=0.8, marker='o', markersize=4, 
                   label=f'Drone {drone_id} ({len(route)-1} teslimat)')
            
            # Ok işaretleri ekle (yön göstermek için)
            for j in range(len(route) - 1):
                start = route[j]
                end = route[j + 1]
                
                # Orta noktada ok çiz
                mid_x = (start[0] + end[0]) / 2
                mid_y = (start[1] + end[1]) / 2
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                
                ax.annotate('', xy=(mid_x + dx*0.1, mid_y + dy*0.1), 
                           xytext=(mid_x - dx*0.1, mid_y - dy*0.1),
                           arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
    
    def _setup_map(self, ax, title: str):
        """Harita ayarlarını yapar"""
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        ax.set_xlabel('X Koordinatı (metre)', fontsize=12)
        ax.set_ylabel('Y Koordinatı (metre)', fontsize=12)
        
        # Grid ekle
        ax.grid(True, alpha=0.3, linestyle='--')
        
        # Eşit ölçek
        ax.set_aspect('equal')
        
        # Kenar boşlukları
        ax.margins(0.05)
        
        # Arka plan rengi
        ax.set_facecolor('#f8f9fa')
    
    def _add_legend(self, ax):
        """Legend ekler"""
        # Özel legend elemanları
        legend_elements = [
            plt.Line2D([0], [0], marker='H', color='w', markerfacecolor='green', 
                      markersize=12, label='Drone Başlangıç', markeredgecolor='darkgreen'),
            plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='orange', 
                      markersize=10, label='Yüksek Öncelik (4-5)'),
            plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='gold', 
                      markersize=10, label='Orta Öncelik (3)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                      markersize=10, label='Düşük Öncelik (1-2)'),
            patches.Patch(color='red', alpha=0.3, label='No-Fly Zone')
        ]
        
        # Mevcut rotalar legend'ına ekle
        handles, labels = ax.get_legend_handles_labels()
        legend_elements.extend(handles)
        
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1))
    
    def plot_comparison(self, results: Dict[str, Dict]):
        """Farklı algoritmaların sonuçlarını karşılaştırır"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        
        algorithms = list(results.keys())
        colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
        
        # 1. Tamamlanan teslimat sayısı
        deliveries = [results[alg].get('completed_deliveries', 0) for alg in algorithms]
        bars1 = ax1.bar(algorithms, deliveries, color=colors[:len(algorithms)])
        ax1.set_title('Tamamlanan Teslimat Sayısı', fontweight='bold')
        ax1.set_ylabel('Teslimat Sayısı')
        
        # Değerleri çubukların üstüne yaz
        for bar, value in zip(bars1, deliveries):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                    str(value), ha='center', va='bottom', fontweight='bold')
        
        # 2. Toplam enerji tüketimi
        energies = [results[alg].get('total_energy', 0) for alg in algorithms]
        bars2 = ax2.bar(algorithms, energies, color=colors[:len(algorithms)])
        ax2.set_title('Toplam Enerji Tüketimi', fontweight='bold')
        ax2.set_ylabel('Enerji (mAh)')
        
        for bar, value in zip(bars2, energies):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(energies)*0.01, 
                    f'{value:.1f}', ha='center', va='bottom', fontweight='bold')
        
        # 3. Çalışma süresi
        times = [results[alg].get('execution_time', 0) for alg in algorithms]
        bars3 = ax3.bar(algorithms, times, color=colors[:len(algorithms)])
        ax3.set_title('Algoritma Çalışma Süresi', fontweight='bold')
        ax3.set_ylabel('Süre (saniye)')
        
        for bar, value in zip(bars3, times):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(times)*0.01, 
                    f'{value:.2f}', ha='center', va='bottom', fontweight='bold')
        
        # 4. Verimlilik skoru (teslimat/enerji oranı)
        efficiency = [d/e if e > 0 else 0 for d, e in zip(deliveries, energies)]
        bars4 = ax4.bar(algorithms, efficiency, color=colors[:len(algorithms)])
        ax4.set_title('Verimlilik Skoru (Teslimat/Enerji)', fontweight='bold')
        ax4.set_ylabel('Skor')
        
        for bar, value in zip(bars4, efficiency):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(efficiency)*0.01, 
                    f'{value:.3f}', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        plt.show()
    
    def plot_fitness_evolution(self, fitness_history: List[float], title: str = "GA Fitness Evrimi"):
        """Genetic Algorithm fitness evrimini gösterir"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        generations = range(len(fitness_history))
        ax.plot(generations, fitness_history, linewidth=2, color='#4ECDC4', marker='o', markersize=3)
        
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.set_xlabel('Nesil', fontsize=12)
        ax.set_ylabel('En İyi Fitness Değeri', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        # En iyi değeri vurgula
        best_fitness = max(fitness_history)
        best_generation = fitness_history.index(best_fitness)
        ax.annotate(f'En İyi: {best_fitness:.2f}', 
                   xy=(best_generation, best_fitness),
                   xytext=(best_generation + len(fitness_history)*0.1, best_fitness),
                   arrowprops=dict(arrowstyle='->', color='red', lw=2),
                   fontsize=12, fontweight='bold',
                   bbox=dict(boxstyle="round,pad=0.3", facecolor='yellow', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
    
    def plot_drone_utilization(self, routes: Dict[int, List[Tuple[float, float]]]):
        """Drone kullanım oranlarını gösterir"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Drone başına teslimat sayısı
        drone_deliveries = {}
        for drone_id, route in routes.items():
            drone_deliveries[drone_id] = len(route) - 1 if len(route) > 1 else 0
        
        drones = list(drone_deliveries.keys())
        deliveries = list(drone_deliveries.values())
        
        bars = ax1.bar([f'Drone {d}' for d in drones], deliveries, 
                      color=self.colors[:len(drones)])
        ax1.set_title('Drone Başına Teslimat Sayısı', fontweight='bold')
        ax1.set_ylabel('Teslimat Sayısı')
        ax1.tick_params(axis='x', rotation=45)
        
        # Değerleri çubukların üstüne yaz
        for bar, value in zip(bars, deliveries):
            if value > 0:
                ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                        str(value), ha='center', va='bottom', fontweight='bold')
        
        # Pasta grafiği (aktif vs pasif drone'lar)
        active_drones = sum(1 for d in deliveries if d > 0)
        inactive_drones = len(drones) - active_drones
        
        labels = ['Aktif Drone\'lar', 'Pasif Drone\'lar']
        sizes = [active_drones, inactive_drones]
        colors = ['#4ECDC4', '#FFB6C1']
        
        ax2.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', 
               startangle=90, textprops={'fontweight': 'bold'})
        ax2.set_title('Drone Kullanım Oranı', fontweight='bold')
        
        plt.tight_layout()
        plt.show()
    
    def save_route_map(self, routes: Dict[int, List[Tuple[float, float]]], 
                      filename: str, dpi: int = 300):
        """Rota haritasını dosyaya kaydeder"""
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # Haritayı çiz
        self._draw_no_fly_zones(ax)
        self._draw_delivery_points(ax)
        self._draw_drone_start_positions(ax)
        self._draw_routes(ax, routes)
        self._setup_map(ax, "Drone Teslimat Rotaları")
        self._add_legend(ax)
        
        plt.tight_layout()
        plt.savefig(filename, dpi=dpi, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        print(f"Harita '{filename}' dosyasına kaydedildi.")
        plt.close()

# Kullanım örneği
if __name__ == "__main__":
    # Test verisi ile örnek görselleştirme
    from drone_system import DroneFleet, DeliveryPoint, NoFlyZone
    
    # Basit test verisi
    fleet = DroneFleet()
    fleet.add_drone(1, 5.0, 15000, 10.0, (10, 10))
    fleet.add_drone(2, 4.0, 12000, 8.0, (80, 20))
    
    deliveries = [
        DeliveryPoint(1, (30, 40), 2.0, 4, (0, 60)),
        DeliveryPoint(2, (60, 70), 1.5, 3, (10, 50)),
        DeliveryPoint(3, (20, 80), 3.0, 5, (5, 45))
    ]
    
    zones = [
        NoFlyZone(1, [(40, 30), (60, 30), (60, 50), (40, 50)], (0, 120))
    ]
    
    # Test rotaları
    test_routes = {
        1: [(10, 10), (30, 40), (20, 80)],
        2: [(80, 20), (60, 70)]
    }
    
    # Görselleştir
    visualizer = DroneVisualizer(fleet, deliveries, zones)
    visualizer.plot_routes(test_routes, "Test Rotaları") 