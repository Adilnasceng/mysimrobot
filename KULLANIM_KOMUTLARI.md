# MySimRobot Komut Rehberi

Bu dosya simülasyonu başlatmak, Nav2 ile hedefe gitmek, rota kaydetmek ve kayıtlı rotayı tekrar çalıştırmak için gerekli terminal komutlarını toplar.

## 1. Her Terminalde Ortamı Hazırla

Her yeni terminalde önce:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
```

Kod veya config değiştirdikten sonra build:

```bash
cd /home/adil/Masaüstü/mysimrobot
colcon build --packages-select mysimrobot
source install/setup.bash
```

## 2. Simülasyonu Başlat

Terminal 1:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 launch mysimrobot launch_sim.launch.py
```

Boş dünyada başlatmak için:

```bash
ros2 launch mysimrobot launch_sim.launch.py world:=/home/adil/Masaüstü/mysimrobot/install/mysimrobot/share/mysimrobot/worlds/empty.world
```

## 3. Harita ile Localization Başlat

Kayıtlı harita üzerinde Nav2 kullanacaksan Terminal 2:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 launch mysimrobot localization_launch.py use_sim_time:=true
```

RViz'de `2D Pose Estimate` ile robotun başlangıç pozunu haritada işaretle.

## 4. Nav2 Navigation Başlat

Terminal 3:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 launch mysimrobot navigation_launch.py use_sim_time:=true
```

## 5. RViz Aç

Terminal 4:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
rviz2 -d install/mysimrobot/share/mysimrobot/config/map.rviz
```

RViz'de faydalı kontroller:

- `Fixed Frame`: `map`
- Robot başlangıcı için: `2D Pose Estimate`
- Hedef göndermek için: `Nav2 Goal`
- Footprint görmek için topic ekle: `/local_costmap/published_footprint`

## 6. Manuel Sürüş

Joystick launch dosyası simülasyonla birlikte başlıyor. Klavye ile sürmek istersen ayrı terminalde:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

Not: `twist_mux` içinde joystick önceliği yüksektir. Manuel komut varsa Nav2 komutlarını bastırabilir.

## 7. Rota Kaydetme

Rota kaydetmeden önce şunlar çalışıyor olmalı:

- Gazebo robot
- Localization veya SLAM
- `map -> base_link` TF'i

Robotu manuel olarak istediğin noktaya götür. Sonra mevcut robot pozunu kaydet:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 run mysimrobot route_tool.py save baslangic --point hedef
```

Bu komut `baslangic` adlı rotaya tek hedef kaydeder.

Kayıt dosyası:

```bash
config/routes.yaml
```

## 8. Kayıtlı Rotaları Listele

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 run mysimrobot route_tool.py list
```

## 9. Kayıtlı Rotaya Git

Nav2 çalışırken kayıtlı rotayı çalıştır:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 run mysimrobot route_tool.py run baslangic
```

Robot `baslangic` rotasındaki hedefe Nav2 ile gider.

## 10. Çok Noktalı Rota Kaydetme

İlk waypoint'i kaydet:

```bash
ros2 run mysimrobot route_tool.py save rota1 --point wp1
```

Robotu manuel olarak ikinci noktaya götür, sonra append ile ekle:

```bash
ros2 run mysimrobot route_tool.py save rota1 --point wp2 --append
```

Üçüncü noktayı ekle:

```bash
ros2 run mysimrobot route_tool.py save rota1 --point wp3 --append
```

Çalıştır:

```bash
ros2 run mysimrobot route_tool.py run rota1
```

Robot `wp1 -> wp2 -> wp3` sırasıyla hedeflere gider.

## 11. SLAM ile Yeni Harita Çıkarma

Localization yerine haritalama yapacaksan Terminal 2'de:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 launch mysimrobot online_async_launch.py use_sim_time:=true
```

Haritayı kaydet:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/adil/Masaüstü/mysimrobot/maps/yeni_harita
```

Sonra `localization_launch.py` içinde varsayılan harita yerine yeni YAML dosyasını kullanabilirsin:

```bash
ros2 launch mysimrobot localization_launch.py use_sim_time:=true map:=/home/adil/Masaüstü/mysimrobot/maps/yeni_harita.yaml
```

## 12. Fork Kontrolü

Fork teleop:

```bash
cd /home/adil/Masaüstü/mysimrobot
source install/setup.bash
ros2 run mysimrobot fork_teleop.py
```

Tuşlar:

- `u`: çatal yukarı
- `d`: çatal aşağı
- `q`: çıkış

## 13. Faydalı Kontrol ve Debug Komutları

Aktif topic'leri gör:

```bash
ros2 topic list
```

TF ağacını kontrol et:

```bash
ros2 run tf2_tools view_frames
```

Robotun harita pozunu kontrol et:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Nav2 hedef action'ını kontrol et:

```bash
ros2 action list | grep navigate
```

Hız komutlarını izle:

```bash
ros2 topic echo /cmd_vel
```

Nav2'nin ürettiği ham komutu izle:

```bash
ros2 topic echo /cmd_vel_nav
```

Lidar verisini izle:

```bash
ros2 topic echo /scan --once
```

Footprint'i izle:

```bash
ros2 topic echo /local_costmap/published_footprint
```

Route dosyasını terminalde göster:

```bash
cat /home/adil/Masaüstü/mysimrobot/config/routes.yaml
```

## 14. Önerilen Terminal Sırası

Tipik kayıt ve rota çalıştırma sırası:

```text
Terminal 1: launch_sim.launch.py
Terminal 2: localization_launch.py
Terminal 3: navigation_launch.py
Terminal 4: rviz2
Terminal 5: manuel sürüş veya route_tool.py komutları
```

Rota kaydet:

```bash
ros2 run mysimrobot route_tool.py save baslangic --point hedef
```

Rota çalıştır:

```bash
ros2 run mysimrobot route_tool.py run baslangic
```

