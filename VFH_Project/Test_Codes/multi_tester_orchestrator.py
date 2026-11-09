import time
import subprocess
import os
import signal
import sys
import glob

class MultiTesterOrchestrator:
    def __init__(self, num_drones=3, iterations=5, use_gui=False):
        self.num_drones = num_drones
        self.iterations = iterations
        self.use_gui = use_gui
        self.home_dir = os.path.expanduser('~')
        self.proj_dir = f"{self.home_dir}/VFH_Project"
        
        self.gazebo_process = None
        self.bridge_process = None
        self.vfh_processes = []

    def generate_bridge_yaml(self):
        yaml_content = ""
        for i in range(self.num_drones):
            name = f"tethys_{i}"
            yaml_content += f"""
- ros_topic_name: "/model/{name}/joint/propeller_joint/cmd_thrust"
  gz_topic_name: "/model/{name}/joint/propeller_joint/cmd_thrust"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/model/{name}/joint/horizontal_fins_joint/fin_0/cmd_pos"
  gz_topic_name: "/model/{name}/joint/horizontal_fins_joint/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/model/{name}/joint/vertical_fins_joint/fin_0/cmd_pos"
  gz_topic_name: "/model/{name}/joint/vertical_fins_joint/0/cmd_pos"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/{name}/sonar_3d/points"
  gz_topic_name: "/{name}/sonar_3d/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/{name}/altimeter"
  gz_topic_name: "/{name}/altimeter"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/{name}/imu"
  gz_topic_name: "/{name}/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
  
- ros_topic_name: "/model/{name}/odometry"
  gz_topic_name: "/model/{name}/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
 
- ros_topic_name: "/{name}/bumper"
  gz_topic_name: "/world/vfh_training_world/model/{name}/link/base_link/sensor/tethys_bumper/contact"
  ros_type_name: "ros_gz_interfaces/msg/Contacts"
  gz_type_name: "gz.msgs.Contacts"
  direction: GZ_TO_ROS
"""
        with open(f"{self.proj_dir}/bridge_multi.yaml", "w") as f:
            f.write(yaml_content)

    def clean_result_files(self):
        files = glob.glob("/tmp/result_tethys_*.txt") + glob.glob("/tmp/status_tethys_*.txt")
        for f in files:
            try: os.remove(f)
            except: pass

    def get_weights(self):
        try:
            with open(f"{self.proj_dir}/vfh_rl_weights.txt", "r") as f:
                return f.read().strip()
        except: return "Not Found"

    def start_simulation(self):
        print("     [Orchestrator] Generating procedural world...")
        subprocess.run(["python3", "generate_multi_world.py", str(self.num_drones)], cwd=self.proj_dir, stdout=subprocess.DEVNULL)
        self.generate_bridge_yaml()

        print(f"     [Orchestrator] Starting Gazebo in background (GUI: {'ON' if self.use_gui else 'OFF'})...")
        
        gz_cmd = ["gz", "sim", "-r", "vfh_ready.sdf"]
        if not self.use_gui:
            gz_cmd.insert(2, "-s") # Aggiunge la flag server-only se la GUI non è richiesta
            
        self.gazebo_process = subprocess.Popen(
            gz_cmd, cwd=self.proj_dir, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        
        print("     [Orchestrator] Waiting for physics engine to load (15 seconds)...")
        time.sleep(15.0)
        
        print("     [Orchestrator] Executing swarm spawn script...")
        subprocess.run(["./spawn_multi.sh"], cwd=self.proj_dir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print("     [Orchestrator] Waiting for swarm stabilization (5 seconds)...")
        time.sleep(5.0)
        
        print("     [Orchestrator] Starting ROS-Gazebo Bridge...")
        bridge_cmd = f"source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:={self.proj_dir}/bridge_multi.yaml"
        self.bridge_process = subprocess.Popen(
            bridge_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid, 
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        
        spawn_success = False
        for attempt in range(10): 
            time.sleep(2.0)
            try:
                check_cmd = "source /opt/ros/humble/setup.bash && ros2 topic list"
                result = subprocess.run(check_cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, text=True, timeout=5)
                if "/model/tethys_0/odometry" in result.stdout:
                    print("     [+] Swarm detected! Sensors online and ROS 2 bridge established.")
                    spawn_success = True
                    break
            except subprocess.TimeoutExpired:
                pass
                
        if not spawn_success:
            print("     [!] CRITICAL ERROR: Swarm spawn failed. Restarting...")
            self.stop_all()
            time.sleep(2)
            return self.start_simulation()

    def start_vfh_nodes(self):
        print("     [Orchestrator] Starting VFH Nodes...")
        self.vfh_processes = []
        for i in range(self.num_drones):
            cmd = f"source /opt/ros/humble/setup.bash && source {self.home_dir}/ros2_ws/install/setup.bash && ros2 run uuv_random_policy vfh_multi_node --ros-args -p robot_name:=tethys_{i}"
            p = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self.vfh_processes.append(p)

    def remove_drone_from_gazebo(self, i, state_reason):
        drone_name = f"tethys_{i}"
        print(f"     [!] Drone {drone_name} reached final state [{state_reason}]. Removing from simulation...")
        try: os.killpg(os.getpgid(self.vfh_processes[i].pid), signal.SIGKILL)
        except: pass
        rm_cmd = f"gz service -s /world/vfh_training_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: \"{drone_name}\" type: MODEL'"
        subprocess.run(rm_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def log_and_print(self, text, log_file):
        print(text)
        with open(log_file, "a") as log: log.write(text + "\n")

    def write_live_telemetry_file(self, telemetry_text, live_log_file):
        with open(live_log_file, "a") as f:
            f.write(telemetry_text + "\n")

    def parse_cumulative_history(self, log_file):
        c_succ, c_obs, c_drone, c_time = 0, 0, 0, 0
        if os.path.exists(log_file):
            with open(log_file, "r") as f:
                for line in f:
                    if "Result (Iter" in line and "Success:" in line:
                        parts = line.split("|")
                        for part in parts:
                            if "Success:" in part:
                                c_succ += int(part.split("Success:")[1].strip())
                            elif "Obs_Crash:" in part:
                                c_obs += int(part.split("Obs_Crash:")[1].strip())
                            elif "Drone_Crash:" in part:
                                c_drone += int(part.split("Drone_Crash:")[1].strip())
                            elif "Timeout:" in part:
                                c_time += int(part.split("Timeout:")[1].strip())
        return c_succ, c_obs, c_drone, c_time

    def print_cumulative_summary(self, log_file):
        c_succ, c_obs, c_drone, c_time = self.parse_cumulative_history(log_file)
        total_agents = c_succ + c_obs + c_drone + c_time
        
        if total_agents == 0:
            print("\n[!] No historical test data found in log.")
            return

        final_summary = (
            f"\n=== CUMULATIVE HISTORICAL SWARM TEST RESULTS ===\n"
            f"Total Drones Deployed Across All Runs : {total_agents}\n"
            f"Goal Reached                          : {c_succ} ({(c_succ/total_agents)*100:.1f}%)\n"
            f"Obstacle Collisions                   : {c_obs} ({(c_obs/total_agents)*100:.1f}%)\n"
            f"Drone-Drone Collisions                : {c_drone} ({(c_drone/total_agents)*100:.1f}%)\n"
            f"Timeouts                              : {c_time} ({(c_time/total_agents)*100:.1f}%)\n"
            f"----------------------------------------"
        )
        self.log_and_print(final_summary, log_file)

    def stop_all(self):
        for p in self.vfh_processes:
            try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except: pass
        self.vfh_processes.clear()

        if self.bridge_process:
            try: os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGKILL)
            except: pass
        if self.gazebo_process:
            try: os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGKILL)
            except: pass
        
        subprocess.run(["killall", "-9", "ruby", "gz", "_gz_sim", "ros_gz_bridge", "parameter_bridge", "vfh_multi_node"], stderr=subprocess.DEVNULL)
        time.sleep(8.0)

    def run(self):
        log_path = f"{self.proj_dir}/swarm_test_history.log"
        live_log_path = f"{self.proj_dir}/swarm_live_status.log"
        weights = self.get_weights()
        
        start_msg = f"\n--- SWARM TEST INITIATED | Drones: {self.num_drones} | Iterations: {self.iterations} | GUI: {'ON' if self.use_gui else 'OFF'} ---\nLoaded RL Weights: [{weights}]"
        self.log_and_print(start_msg, log_path)
        
        print(f"  [i] Live telemetry stream written to: {live_log_path}")
        print(f"      (View in second terminal: tail -f {live_log_path})")

        for it in range(self.iterations):
            self.log_and_print(f"\nITERATION {it+1}/{self.iterations}", log_path)
            self.clean_result_files()
            self.start_simulation()
            self.start_vfh_nodes()
            
            start_time = time.time()
            last_telemetry_time = start_time
            max_duration = 420.0
            finished_drones = set()
            
            print("     [~] Testing... (Timeout: 7 min)")
            
            while time.time() - start_time < max_duration:
                current_time = time.time()
                elapsed = current_time - start_time
                
                for i in range(self.num_drones):
                    if i not in finished_drones:
                        res_file = f"/tmp/result_tethys_{i}.txt"
                        if os.path.exists(res_file):
                            with open(res_file, "r") as f:
                                state_reason = f.read().strip()
                            self.remove_drone_from_gazebo(i, state_reason)
                            finished_drones.add(i)
                
                if current_time - last_telemetry_time >= 15.0:
                    mins = int(elapsed // 60)
                    secs = int(elapsed % 60)
                    telemetry_block = f"\nMinute {mins:02d}:{secs:02d}, Iteration {it+1}/{self.iterations}, drones state:"
                    for i in range(self.num_drones):
                        stat_file = f"/tmp/status_tethys_{i}.txt"
                        if os.path.exists(stat_file):
                            with open(stat_file, "r") as f:
                                data = f.read().strip().split()
                                if len(data) >= 5:
                                    state, dist, x, y, z = data[0], float(data[1]), float(data[2]), float(data[3]), float(data[4])
                                    telemetry_block += f"\n  tethys_{i}: state {state}, wp distance {dist:.1f}, position: {x:.1f}, {y:.1f}, {z:.1f}"
                        else:
                            telemetry_block += f"\n  tethys_{i}: state UNKNOWN/INITIALIZING"
                    
                    self.write_live_telemetry_file(telemetry_block, live_log_path)
                    last_telemetry_time = current_time
                
                if len(finished_drones) == self.num_drones:
                    break
                time.sleep(1.0)
            
            duration = time.time() - start_time
            self.log_and_print(f"     [!] Iteration ended in {duration/60.0:.2f} min.", log_path)
            
            i_succ = 0; i_obs = 0; i_drone = 0; i_time = 0
            
            for i in range(self.num_drones):
                res_file = f"/tmp/result_tethys_{i}.txt"
                if os.path.exists(res_file):
                    with open(res_file, "r") as f:
                        res = f.read().strip()
                        if res == "SUCCESS": i_succ += 1
                        elif res == "CRASH_OBSTACLE": i_obs += 1
                        elif res == "CRASH_DRONE": i_drone += 1
                else:
                    i_time += 1
            
            res_str = f"  Result (Iter {it+1}): Success: {i_succ} | Obs_Crash: {i_obs} | Drone_Crash: {i_drone} | Timeout: {i_time}"
            self.log_and_print(res_str, log_path)
            
            self.print_cumulative_summary(log_path)
            
            self.stop_all()

        self.print_cumulative_summary(log_path)

if __name__ == '__main__':
    drones = 3
    iters = 5
    use_gui = False
    
    if len(sys.argv) > 1: drones = int(sys.argv[1])
    if len(sys.argv) > 2: iters = int(sys.argv[2])
    if len(sys.argv) > 3:
        arg3 = sys.argv[3].lower()
        if arg3 in ['gui', 'true', '1', '--gui']:
            use_gui = True
            
    orchestrator = MultiTesterOrchestrator(num_drones=drones, iterations=iters, use_gui=use_gui)
    log_file = f"{orchestrator.proj_dir}/swarm_test_history.log"
    try: 
        orchestrator.run()
    except KeyboardInterrupt:
        print("\n[!] Test interrupted by user. Cleaning up and parsing cumulative history...")
        orchestrator.stop_all()
        orchestrator.print_cumulative_summary(log_file)
