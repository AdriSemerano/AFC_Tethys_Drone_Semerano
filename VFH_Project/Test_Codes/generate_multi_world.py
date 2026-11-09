import random
import os
import math
import stat
import sys

# --- ARENA CONFIGURATION ---
ARENA_SIZE = 200.0      
SPAWN_RADIUS = 120.0    
MAX_HEIGHT = 280.0      
NUM_SPHERES = 100       
NUM_PILLARS = 40        
NUM_BOXES = 120         
NUM_BEAMS = 100         
SAFE_ZONE_RADIUS = 20.0 

num_drones = 3
if len(sys.argv) > 1:
    try: num_drones = int(sys.argv[1])
    except: pass

def get_min_z(x, y):
    if abs(x) > 170 or abs(y) > 170: return 60.0 
    if math.hypot(x - (-80), y - 80) < 85.0:   return 25.0
    if math.hypot(x - 90, y - (-70)) < 90.0:   return 30.0
    if math.hypot(x - (-40), y - (-100)) < 85.0: return 20.0
    if math.hypot(x - 100, y - 100) < 80.0:    return 35.0
    if math.hypot(x - 10, y - 20) < 35.0:      return 5.0
    return -40.0

drones = []
safe_points = []
waypoints_visuals = []

angle_step = (2 * math.pi) / num_drones
current_radius = random.uniform(90.0, 130.0)

with open("tethys_equipped.sdf", "r") as f:
    base_tethys_sdf = f.read()

for i in range(num_drones):
    angle = i * angle_step
    jitter_x = random.uniform(-15.0, 15.0)
    jitter_y = random.uniform(-15.0, 15.0)
    
    start_x = current_radius * math.cos(angle) + jitter_x
    start_y = current_radius * math.sin(angle) + jitter_y
    base_z = get_min_z(start_x, start_y)
    start_z = random.uniform(base_z + 40.0, base_z + 60.0) if i % 2 == 0 else random.uniform(base_z + 70.0, base_z + 90.0)
    
    target_x = -start_x
    target_y = -start_y
    target_base_z = get_min_z(target_x, target_y)
    target_z = random.uniform(target_base_z + 70.0, target_base_z + 90.0) if i % 2 == 0 else random.uniform(target_base_z + 40.0, target_base_z + 60.0)
    
    start_yaw = math.atan2(target_y - start_y, target_x - start_x) + math.pi
    
    drone_name = f"tethys_{i}"
    drones.append({'name': drone_name, 'x': start_x, 'y': start_y, 'z': start_z, 'yaw': start_yaw})
    
    safe_points.append((start_x, start_y, start_z))
    safe_points.append((target_x, target_y, target_z))
    waypoints_visuals.append((target_x, target_y, target_z))
    
    with open(f"wp_{drone_name}.txt", "w") as f:
        f.write(f"{target_x:.2f} {target_y:.2f} {target_z:.2f}\n")
        
    custom_sdf = base_tethys_sdf.replace("/tethys/", f"/{drone_name}/")
    custom_sdf = custom_sdf.replace("<namespace>tethys</namespace>", f"<namespace>{drone_name}</namespace>")
    
    with open(f"tethys_equipped_{i}.sdf", "w") as f:
        f.write(custom_sdf)

spawn_script = "#!/bin/bash\nsource /opt/ros/humble/setup.bash\n"
for i, d in enumerate(drones):
    spawn_script += f"ros2 run ros_gz_sim create -world vfh_training_world -file tethys_equipped_{i}.sdf -name {d['name']} -x {d['x']:.2f} -y {d['y']:.2f} -z {d['z']:.2f} -Y {d['yaw']:.2f} &\n"

with open("spawn_multi.sh", "w") as f:
    f.write(spawn_script)
os.chmod("spawn_multi.sh", os.stat("spawn_multi.sh").st_mode | stat.S_IEXEC)

def is_in_safe_zone(x, y, z):
    for px, py, pz in safe_points:
        if math.hypot(x - px, y - py) < SAFE_ZONE_RADIUS and abs(z - pz) < SAFE_ZONE_RADIUS: return True
    return False

def generate_obstacles_and_markers():
    xml = '\n    <model name="procedural_obstacles">\n      <static>true</static>\n'
    
    # SPHERES
    for i in range(NUM_SPHERES):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            r = random.uniform(1.0, 5.0)
            z = random.uniform(get_min_z(x, y) + r + 2.0, MAX_HEIGHT)
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="sphere_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose><collision name="col"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry></collision><visual name="vis"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry><material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual></link>\n'

    # PILLARS
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

    # BOXES (ORANGE CUBES)
    for i in range(NUM_BOXES):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            sx, sy, sz = random.uniform(2.0, 10.0), random.uniform(2.0, 10.0), random.uniform(2.0, 10.0)
            z = random.uniform(get_min_z(x, y) + (sz/2) + 2.0, MAX_HEIGHT)
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="box_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {random.uniform(0, 3.14):.2f}</pose><collision name="col"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry></collision><visual name="vis"><geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry><material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material></visual></link>\n'

    # BEAMS (YELLOW BEAMS)
    for i in range(NUM_BEAMS):
        while True:
            x, y = random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE)
            z = random.uniform(get_min_z(x, y) + 15.0, MAX_HEIGHT)
            if not is_in_safe_zone(x, y, z): break
        xml += f'      <link name="beam_{i}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 1.5707 {random.uniform(0, 3.14):.2f}</pose><collision name="col"><geometry><cylinder><radius>1.0</radius><length>{random.uniform(15.0, 40.0):.2f}</length></cylinder></geometry></collision><visual name="vis"><geometry><cylinder><radius>1.0</radius><length>{random.uniform(15.0, 40.0):.2f}</length></cylinder></geometry><material><ambient>0.9 0.8 0.1 1</ambient><diffuse>0.9 0.8 0.1 1</diffuse></material></visual></link>\n'

    # WAYPOINT MARKERS
    for i, wp in enumerate(waypoints_visuals):
        xml += f"""
      <link name="wp_marker_{i}">
        <pose>{wp[0]:.2f} {wp[1]:.2f} {wp[2]:.2f} 0 0 0</pose>
        <visual name="vis"><geometry><sphere><radius>5.0</radius></sphere></geometry><material><ambient>0.7 1.0 0.0 0.5</ambient><diffuse>0.7 1.0 0.0 0.5</diffuse></material></visual>
      </link>"""
    xml += '    </model>\n'
    return xml

if __name__ == "__main__":
    with open("vfh_base.sdf", "r") as f:
        content = f.read()
    
    final_content = content.replace("</world>", f"{generate_obstacles_and_markers()}\n  </world>")
    
    with open("vfh_ready.sdf", "w") as f:
        f.write(final_content)
    print(f"Multi-Agent World Generated. Drones: {num_drones}.")
