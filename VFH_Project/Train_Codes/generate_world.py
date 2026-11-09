import random
import os
import math
import stat
import sys

# --- CONFIGURAZIONE ARENA (ESPANSIONE PER TESTING/TRAINING) ---
ARENA_SIZE = 200.0      # Espansione totale del campo ostacoli
SPAWN_LIMIT = 150.0     # Il drone e i WP restano in un'area vasta ma sicura dai bordi
MAX_HEIGHT = 280.0      
NUM_SPHERES = 100       
NUM_PILLARS = 40        
NUM_BOXES = 120         
NUM_BEAMS = 100         
SAFE_ZONE_RADIUS = 20.0 

# --- LETTURA MODALITA' DA RIGA DI COMANDO ---
# Se non viene specificato nulla, di default fa "random"
mode = "random"
if len(sys.argv) > 1:
    mode = sys.argv[1].lower()

# --- EURISTICA DEL FONDALE AGGIORNATA ---
def get_min_z(x, y):
    # 1. Argini perimetrali massicci (centrati a +/-220)
    # Hanno un picco Z massimo attorno ai 55 metri a causa della rotazione.
    if abs(x) > 170 or abs(y) > 170:
        return 60.0 
        
    # 2. Controllo a raggio sulle singole dune fisse (Coordinate da vfh_base.sdf)
    # math.hypot calcola la distanza 2D dal centro dell'ostacolo
    if math.hypot(x - (-80), y - 80) < 85.0:   # dune_huge_1
        return 25.0
    if math.hypot(x - 90, y - (-70)) < 90.0:   # dune_huge_2
        return 30.0
    if math.hypot(x - (-40), y - (-100)) < 85.0: # dune_steep_3
        return 20.0
    if math.hypot(x - 100, y - 100) < 80.0:    # dune_steep_4
        return 35.0
    
    # 3. Ostacoli centrali
    if math.hypot(x - 10, y - 20) < 35.0:      # spire_center
        return 5.0
    
    # 4. Spazio vuoto nel canyon
    return -40.0

# 1. GENERAZIONE SPAWN ALL'INTERNO DEL LIMITE DI SICUREZZA
start_x = random.uniform(-SPAWN_LIMIT, SPAWN_LIMIT)
start_y = random.uniform(-SPAWN_LIMIT, SPAWN_LIMIT)
start_z = random.uniform(get_min_z(start_x, start_y) + 10.0, 100.0)

# Calcolo dello Yaw affinché il muso del drone punti verso il centro (0, 0)
start_yaw = math.atan2(0.0 - start_y, 0.0 - start_x)

waypoints = []

# --- 2. GENERAZIONE SINGOLO OBIETTIVO BASATA SUL MODO ---
if mode == "canyon":
    # Obiettivo nel crepaccio centrale
    wx = random.uniform(-15.0, 15.0)
    wy = random.uniform(-15.0, 15.0)
    # Lo forziamo a stare in basso (tra +5 e +20 metri dal fondo), ma al sicuro dalla guglia
    wz = random.uniform(get_min_z(wx, wy) + 5.0, get_min_z(wx, wy) + 20.0)
else:
    # Obiettivo ad altitudine e posizione randomica
    wx = random.uniform(-SPAWN_LIMIT, SPAWN_LIMIT)
    wy = random.uniform(-SPAWN_LIMIT, SPAWN_LIMIT)
    # Grazie al nuovo get_min_z, possiamo farlo spawnare anche a soli 10 metri dal suolo ovunque
    wz = random.uniform(get_min_z(wx, wy) + 10.0, 200.0)

waypoints.append((wx, wy, wz))

safe_points = [(start_x, start_y, start_z)] + waypoints

# Scrittura waypoint per il nodo C++
with open("current_waypoints.txt", "w") as f:
    for wp in waypoints:
        f.write(f"{wp[0]:.2f} {wp[1]:.2f} {wp[2]:.2f}\n")

# --- GENERAZIONE SCRIPT DI SPAWN AUTOMATICO ---
spawn_cmd = f"ros2 run ros_gz_sim create -world vfh_training_world -file tethys_equipped.sdf -name tethys -x {start_x:.2f} -y {start_y:.2f} -z {start_z:.2f} -Y {start_yaw:.2f}"
with open("spawn_drone.sh", "w") as f:
    f.write("#!/bin/bash\n")
    f.write("source /opt/ros/humble/setup.bash\n")
    f.write(spawn_cmd + "\n")
os.chmod("spawn_drone.sh", os.stat("spawn_drone.sh").st_mode | stat.S_IEXEC)

def is_in_safe_zone(x, y, z):
    for px, py, pz in safe_points:
        dist = math.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)
        if dist < SAFE_ZONE_RADIUS:
            return True
    return False

def generate_obstacles_and_markers():
    xml = '\n    <model name="procedural_obstacles">\n      <static>true</static>\n'
    
    # SFERE
    for i in range(NUM_SPHERES):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            r = random.uniform(1.0, 5.0)
            z = random.uniform(get_min_z(x, y) + r + 2.0, MAX_HEIGHT)
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="sphere_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose><collision name="col"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry></collision><visual name="vis"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry><material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual></link>\n'

    # PILONI
    for i in range(NUM_PILLARS):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            l = random.uniform(30.0, 120.0)
            z = get_min_z(x, y) + (l / 2.0) - 5.0
            if z + (l / 2.0) > MAX_HEIGHT:
                l = (MAX_HEIGHT - get_min_z(x, y)) * 0.8
                z = get_min_z(x, y) + (l / 2.0) - 5.0
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="pillar_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose><collision name="col"><geometry><cylinder><radius>1.5</radius><length>{l:.2f}</length></cylinder></geometry></collision><visual name="vis"><geometry><cylinder><radius>1.5</radius><length>{l:.2f}</length></cylinder></geometry><material><ambient>0.2 0.8 0.2 1</ambient><diffuse>0.2 0.8 0.2 1</diffuse></material></visual></link>\n'

    # CUBI
    for i in range(NUM_BOXES):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            sx, sy, sz = random.uniform(2.0, 10.0), random.uniform(2.0, 10.0), random.uniform(2.0, 10.0)
            z = random.uniform(get_min_z(x, y) + (sz/2) + 2.0, MAX_HEIGHT)
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="box_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {random.uniform(0, 3.14):.2f}</pose><collision name="col"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry></collision><visual name="vis"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry><material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material></visual></link>\n'

    # TRAVI
    for i in range(NUM_BEAMS):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            z = random.uniform(get_min_z(x, y) + 15.0, MAX_HEIGHT)
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="beam_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 1.5707 {random.uniform(0, 3.14):.2f}</pose><collision name="col"><geometry><cylinder><radius>1.0</radius><length>{random.uniform(15.0, 40.0):.2f}</length></cylinder></geometry></collision><visual name="vis"><geometry><cylinder><radius>1.0</radius><length>{random.uniform(15.0, 40.0):.2f}</length></cylinder></geometry><material><ambient>0.9 0.8 0.1 1</ambient><diffuse>0.9 0.8 0.1 1</diffuse></material></visual></link>\n'

    # WAYPOINT VISIVI
    for i, wp in enumerate(waypoints):
        xml += f"""
      <link name="wp_marker_{i}">
        <pose>{wp[0]:.2f} {wp[1]:.2f} {wp[2]:.2f} 0 0 0</pose>
        <visual name="vis">
          <geometry><sphere><radius>5.0</radius></sphere></geometry>
          <material>
            <ambient>0.7 1.0 0.0 0.5</ambient>
            <diffuse>0.7 1.0 0.0 0.5</diffuse>
            <specular>0.7 1.0 0.0 1.0</specular>
          </material>
        </visual>
      </link>"""
    
    xml += '    </model>\n'
    return xml

if __name__ == "__main__":
    with open("vfh_base.sdf", "r") as f:
        content = f.read()
        
    marker_start = ""
    marker_end = ""
    
    s_idx = content.find(marker_start)
    e_idx = content.find(marker_end)
    
    if s_idx != -1 and e_idx != -1:
        content = content[:s_idx] + content[e_idx + len(marker_end):]
        
    new_block = f"{marker_start}{generate_obstacles_and_markers()}{marker_end}"
    final_content = content.replace("</world>", f"{new_block}\n  </world>")
    
    with open("vfh_ready.sdf", "w") as f:
        f.write(final_content)
        
    print(f"Mondo generato in modalità '{mode}'. Yaw Drone = {start_yaw:.2f} rad.")
