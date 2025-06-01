# Drone Teslimat Rota Optimizasyonu

Bu proje, enerji limitleri ve uçuş yasağı bölgeleri (no-fly zone) gibi dinamik kısıtlar altında çalışan drone'lar için en uygun teslimat rotalarının belirlenmesini sağlayan gelişmiş algoritmaları içerir.

## 🚁 Proje Özellikleri

- **A* Algoritması**: Graf tabanlı optimal yol bulma
- **Genetic Algorithm**: Evrimsel optimizasyon ile en iyi çözüm arama
- **CSP (Constraint Satisfaction Problem)**: Dinamik kısıtlar altında atama problemi çözümü
- **Görselleştirme**: Matplotlib ile interaktif harita ve grafik gösterimi
- **Rastgele Veri Üreteci**: Test senaryoları için esnek veri oluşturma
- **Performans Analizi**: Algoritmaların karşılaştırmalı değerlendirmesi

## 📋 Gereksinimler

- Python 3.8+
- matplotlib>=3.5.0
- numpy>=1.21.0
- typing-extensions>=4.0.0

## 🛠️ Kurulum

1. **Projeyi klonlayın:**
   ```bash
   git clone <repo-url>
   cd drone_uyg3
   ```

2. **Sanal ortam oluşturun (önerilen):**
   ```bash
   python -m venv venv
   source venv/bin/activate  # Linux/Mac
   # veya
   venv\Scripts\activate     # Windows
   ```

3. **Bağımlılıkları yükleyin:**
   ```bash
   pip install -r requirements.txt
   ```

## 🚀 Hızlı Başlangıç

Ana uygulamayı çalıştırın:

```bash
python main.py
```

Bu komut iki ana senaryoyu çalıştırır:
- **Senaryo 1**: 5 drone, 20 teslimat, 3 no-fly zone
- **Senaryo 2**: 10 drone, 50 teslimat, 5 no-fly zone

## 📊 Algoritma Detayları

### 1. A* Algoritması (pathfinding.py)

**Özellikler:**
- Graf tabanlı optimal yol bulma
- Heuristic fonksiyon: `distance + no_fly_zone_penalty`
- Maliyet fonksiyonu: `distance × weight + (priority × 100)`

**Kullanım:**
```python
from pathfinding import AStarPathfinder

pathfinder = AStarPathfinder(fleet, deliveries, no_fly_zones)
routes = pathfinder.find_optimal_routes()
```

### 2. Genetic Algorithm (genetic_algorithm.py)

**Özellikler:**
- Popülasyon tabanlı evrimsel optimizasyon
- Fitness fonksiyonu: `(teslimat sayısı × 50) - (enerji × 0.1) - (ihlal × 1000)`
- Çaprazlama, mutasyon ve elitizm

**Parametreler:**
- Popülasyon boyutu: 50-100
- Nesil sayısı: 100-150
- Mutasyon oranı: 0.1
- Çaprazlama oranı: 0.8

**Kullanım:**
```python
from genetic_algorithm import GeneticAlgorithm

ga = GeneticAlgorithm(fleet, deliveries, no_fly_zones, 
                      population_size=50, generations=100)
solution = ga.evolve()
```

### 3. CSP Çözücü (csp_solver.py)

**Kısıtlar:**
- Ağırlık kapasitesi
- Batarya kapasitesi
- Zaman penceresi
- No-fly zone ihlali
- Tekil atama (her teslimat sadece bir drone'a)

**Kullanım:**
```python
from csp_solver import CSPSolver

csp_solver = CSPSolver(fleet, deliveries, no_fly_zones)
solution = csp_solver.solve()
```

## 📁 Proje Yapısı

```
drone_uyg3/
├── main.py                 # Ana uygulama
├── drone_system.py         # Temel sınıflar (Drone, DeliveryPoint, NoFlyZone)
├── pathfinding.py          # A* algoritması
├── genetic_algorithm.py    # Genetik algoritma
├── csp_solver.py          # CSP çözücü
├── visualization.py        # Görselleştirme
├── data_generator.py       # Rastgele veri üreteci
├── verisetitxt.txt        # Örnek veri seti
├── requirements.txt        # Bağımlılıklar
└── README.md              # Bu dosya
```

## 📊 Veri Yapıları

### Drone Özellikleri
```python
{
    "id": 1,
    "max_weight": 4.0,        # kg
    "battery": 12000,         # mAh
    "speed": 8.0,            # m/s
    "start_pos": (10, 10)    # (x, y) koordinatları
}
```

### Teslimat Noktaları
```python
{
    "id": 1,
    "pos": (15, 25),         # (x, y) koordinatları
    "weight": 1.5,           # kg
    "priority": 3,           # 1-5 arası (5 en yüksek)
    "time_window": (0, 60)   # (başlangıç, bitiş) dakika
}
```

### No-Fly Zone'lar
```python
{
    "id": 1,
    "coordinates": [(40, 30), (60, 30), (60, 50), (40, 50)],  # Çokgen köşeleri
    "active_time": (0, 120)  # (başlangıç, bitiş) dakika
}
```

## 🎯 Kullanım Örnekleri

### Rastgele Veri Üretme

```python
from data_generator import DataGenerator

generator = DataGenerator(seed=42)
drones, deliveries, zones = generator.generate_scenario_data(
    drone_count=5, 
    delivery_count=20, 
    zone_count=3
)
```

### Görselleştirme

```python
from visualization import DroneVisualizer

visualizer = DroneVisualizer(fleet, deliveries, no_fly_zones)
visualizer.plot_routes(routes, "Optimized Routes")
visualizer.plot_comparison(algorithm_results)
```

### Fitness Evrimi

```python
ga = GeneticAlgorithm(fleet, deliveries, no_fly_zones)
solution = ga.evolve()

# Fitness gelişimini görselleştir
visualizer.plot_fitness_evolution(ga.fitness_history)
```

## 📈 Performans Metrikleri

Sistem aşağıdaki metrikleri ölçer:

- **Tamamlanan Teslimat Yüzdesi**: Başarıyla teslim edilen paket oranı
- **Ortalama Enerji Tüketimi**: Drone başına ortalama enerji kullanımı
- **Algoritma Çalışma Süresi**: Çözüm bulma süresi
- **Kısıt İhlali Sayısı**: Ağırlık, enerji, zaman penceresi ihlalleri
- **Drone Kullanım Oranı**: Aktif drone'ların toplam drone'lara oranı

## 🔧 Özelleştirme

### Yeni Algoritma Ekleme

1. `pathfinding.py` modelini takip ederek yeni algoritma dosyası oluşturun
2. `main.py`'de algoritmanızı test senaryolarına ekleyin
3. Görselleştirme için gerekli metrikleri tanımlayın

### Veri Seti Özelleştirme

```python
# Özel senaryo oluşturma
generator = DataGenerator()
custom_drones = generator.generate_drones(8)
custom_deliveries = generator.generate_clustered_deliveries(30, 4)
custom_zones = generator.generate_dynamic_zones(6)
```

## 🐛 Sorun Giderme

### Yaygın Hatalar

1. **ModuleNotFoundError**: `pip install -r requirements.txt` komutu çalıştırın
2. **Matplotlib gösterim sorunu**: GUI backend yüklü olduğundan emin olun
3. **Bellek hatası**: Büyük senaryolarda popülasyon boyutunu azaltın

### Performans Optimizasyonu

- Büyük veri setleri için `numba` kurulumunu yapın
- CSP çözücüde forward checking kullanın
- GA parametrelerini veri boyutuna göre ayarlayın

## 📝 Katkıda Bulunma

1. Projeyi fork edin
2. Feature branch oluşturun (`git checkout -b feature/AmazingFeature`)
3. Değişikliklerinizi commit edin (`git commit -m 'Add some AmazingFeature'`)
4. Branch'inizi push edin (`git push origin feature/AmazingFeature`)
5. Pull Request oluşturun

## 📄 Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Detaylar için `LICENSE` dosyasına bakın.

## 🤝 İletişim

Proje sorumlusu: [İsim Soyisim]
Email: [email@example.com]

Proje Link: [https://github.com/username/drone_uyg3](https://github.com/username/drone_uyg3)

## 🙏 Teşekkürler

- Algoritma tasarımında ilham veren akademik çalışmalar
- Matplotlib ve NumPy geliştiricileri
- Açık kaynak topluluk katkıları