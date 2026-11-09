import numpy as np
import time
import subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
import threading
import os
import signal
import math
import gc  

class SimulationManager:
    def __init__(self):
        self.gazebo_process = None
        self.bridge_process = None
        self.vfh_process = None
        self.home_dir = os.path.expanduser('~')
        
    def generate_world(self, mode="random"):
        print(f"\n[Orchestrator] Generating procedural world (Mode: {mode.upper()})...")
        subprocess.run(["python3", "generate_world.py", mode], cwd=f"{self.home_dir}/VFH_Project")

    def start_simulation(self):
        print("     [Orchestrator] Starting Gazebo in background...")
        self.gazebo_process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "vfh_ready.sdf"], 
            cwd=f"{self.home_dir}/VFH_Project",
            preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        
        print("     [Orchestrator] Waiting for physics engine to load (15 seconds)...")
        time.sleep(15.0)
        
        print("     [Orchestrator] Executing spawn script...")
        subprocess.run(["./spawn_drone.sh"], cwd=f"{self.home_dir}/VFH_Project",
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print("     [Orchestrator] Waiting for drone stabilization (5 seconds)...")
        time.sleep(5.0)
        
        print("     [Orchestrator] Attaching camera to Tethys...")
        subprocess.run(["gz", "topic", "-t", "/gui/follow", "-m", "gz.msgs.StringMsg", "-p", 'data: "tethys"'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print("     [Orchestrator] Starting ROS-Gazebo Bridge...")
        bridge_cmd = f"source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:={self.home_dir}/bridge_config.yaml"
        self.bridge_process = subprocess.Popen(
            bridge_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid, 
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        
        spawn_success = False
        for attempt in range(10): 
            time.sleep(2.0)
            try:
                result = subprocess.run(["ros2", "topic", "list"], stdout=subprocess.PIPE, text=True, timeout=5)
                if "/model/tethys/odometry" in result.stdout:
                    print("     [+] Tethys detected! Sensors online and ROS 2 bridge established.")
                    spawn_success = True
                    break
            except subprocess.TimeoutExpired:
                pass
                
        if not spawn_success:
            print("     [!] CRITICAL ERROR: Drone spawn failed. Restarting...")
            self.stop_simulation()
            time.sleep(2)
            return self.start_simulation() 

    def stop_simulation(self):
        if self.bridge_process:
            try: os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGKILL)
            except: pass
        if self.gazebo_process:
            try: os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGKILL)
            except: pass
        time.sleep(1)
        subprocess.run(["killall", "-9", "ruby", "gz", "_gz_sim", "ros_gz_bridge", "parameter_bridge", "vfh_node"], stderr=subprocess.DEVNULL)
        time.sleep(8.0)

    def start_vfh_node(self):
        vfh_cmd = f"source /opt/ros/humble/setup.bash && source {self.home_dir}/ros2_ws/install/setup.bash && ros2 run uuv_random_policy vfh_node"
        log_path = f"{self.home_dir}/VFH_Project/vfh_live.log"
        self.vfh_log_file = open(log_path, "w")
        self.vfh_process = subprocess.Popen(
            vfh_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid,
            stdout=self.vfh_log_file, stderr=subprocess.STDOUT
        )

    def stop_vfh_node(self):
        if self.vfh_process:
            try: os.killpg(os.getpgid(self.vfh_process.pid), signal.SIGKILL)
            except: pass
        if hasattr(self, 'vfh_log_file') and self.vfh_log_file:
            try: self.vfh_log_file.close()
            except: pass

class EvolutionStrategyOptimizer:
    def __init__(self, num_params=8, pop_size=8): 
        # Initial optimized parameters with unlocked bounds
        self.theta = np.array([28.0, 3.0, 12.0, 8.0, 15.0, 3.0, 1.5, 5.0]) 
        
        self.min_bounds = np.array([10.0, 1.0, 1.0, 1.0,  3.0,  0.1, 0.1, 1.0])
        self.max_bounds = np.array([35.0, 8.0, 15.0, 15.0, 25.0, 5.0, 2.0, 15.0])
        
        self.sigma = np.array([4.0, 2.0, 2.5, 2.5, 3.0, 0.8, 0.4, 2.0])
        self.alpha = np.array([2.5, 1.0, 1.0, 1.0, 1.5, 0.4, 0.1, 1.0])
        
        self.m = np.zeros(num_params) 
        self.v = np.zeros(num_params) 
        self.beta1 = 0.9
        self.beta2 = 0.999
        self.eps = 1e-8
        self.t = 0 
        
        self.pop_size = pop_size
        self.current_gen = 0
        assert pop_size % 2 == 0
        
        self.load_checkpoint()
        
        if not os.path.exists('training_history.log'):
            with open('training_history.log', 'w') as f:
                f.write("TRAINING HISTORY (OpenAI ES + ADAM)\n\n")

    def save_checkpoint(self):
        np.save('vfh_rl_checkpoint.npy', {
            'theta': self.theta, 
            'gen': self.current_gen,
            'sigma': self.sigma,
            'alpha': self.alpha,
            'm': self.m,
            'v': self.v,
            't': self.t
        })

    def load_checkpoint(self):
        if os.path.exists('vfh_rl_checkpoint.npy'):
            data = np.load('vfh_rl_checkpoint.npy', allow_pickle=True).item()
            self.theta = data['theta']
            self.current_gen = data['gen']
            self.sigma = data.get('sigma', self.sigma) 
            self.alpha = data.get('alpha', self.alpha)
            self.m = data.get('m', np.zeros_like(self.theta))
            self.v = data.get('v', np.zeros_like(self.theta))
            self.t = data.get('t', 0)
            print(f"[*] Checkpoint loaded! Resuming from Generation {self.current_gen}")

    def ask(self):
        half_pop = self.pop_size // 2
        self.epsilon = np.random.randn(half_pop, len(self.theta))
        self.epsilon = np.concatenate([self.epsilon, -self.epsilon])
        
        mutated_thetas = []
        for e in self.epsilon:
            mutant = self.theta + self.sigma * e
            mutant_clipped = np.clip(mutant, self.min_bounds, self.max_bounds)
            mutated_thetas.append(mutant_clipped) 
        return mutated_thetas

    def tell(self, rewards, best_metrics, old_theta):
        rewards = np.array(rewards)
        temp = rewards.argsort()
        ranks = np.empty_like(temp)
        ranks[temp] = np.arange(len(rewards))
        
        unique_rewards, inverse_indices, counts = np.unique(rewards, return_inverse=True, return_counts=True)
        for i, count in enumerate(counts):
            if count > 1:
                indices = np.where(inverse_indices == i)[0]
                ranks[indices] = ranks[indices].mean()

        utilities = (ranks / (self.pop_size - 1)) - 0.5
        if np.all(utilities == utilities[0]):
            utilities = np.zeros_like(utilities)
            
        g = np.dot(utilities, self.epsilon) / (self.pop_size * self.sigma)
        
        self.t += 1
        self.m = self.beta1 * self.m + (1 - self.beta1) * g
        self.v = self.beta2 * self.v + (1 - self.beta2) * (g ** 2)
        
        m_hat = self.m / (1 - self.beta1 ** self.t)
        v_hat = self.v / (1 - self.beta2 ** self.t)
        
        step = self.alpha * m_hat / (np.sqrt(v_hat) + self.eps)
        
        self.theta = np.clip(self.theta + step, self.min_bounds, self.max_bounds)
        self.current_gen += 1
        
        # Apply learning rate decay halfway through training
        if self.current_gen > 17:
            self.sigma = np.maximum(0.1, self.sigma * 0.85)
            self.alpha = np.maximum(0.05, self.alpha * 0.85)
        
        self.save_checkpoint()

        report_str = (
            f"Iteration {self.current_gen}: Best clone reported {best_metrics['impacts']:.1f} impacts, "
            f"reached {best_metrics['waypoints_reached']:.2f} targets in {best_metrics['time']:.2f} minutes.\n"
            f"   from: {np.round(old_theta, 2)}\n"
            f"   to:   {np.round(self.theta, 2)}\n\n"
        )
        print("\n" + report_str)
        with open('training_history.log', 'a') as f:
            f.write(report_str)

class TethysEnvNode(Node):
    def __init__(self, home_dir):
        super().__init__('tethys_rl_env')
        self.odom_sub = self.create_subscription(Odometry, '/model/tethys/odometry', self.odom_cb, 10)
        self.bumper_sub = self.create_subscription(Contacts, '/tethys/bumper', self.bumper_cb, 10)
        self.home_dir = home_dir
        self.current_pos = [0.0, 0.0, 0.0]
        self.impacts = 0
        self.last_impact_time = 0.0 
        self.waypoints = []
        self.current_wp_index = 0
        self.start_dist_to_current_wp = -1.0 
        self.total_distance_traveled = 0.0
        self.last_pos = None

    def load_waypoints(self):
        self.waypoints = []
        wp_file = f"{self.home_dir}/VFH_Project/current_waypoints.txt"
        if os.path.exists(wp_file):
            with open(wp_file, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        self.waypoints.append([float(parts[0]), float(parts[1]), float(parts[2])])

    def update_start_distance(self):
        if self.current_wp_index < len(self.waypoints):
            wp = self.waypoints[self.current_wp_index]
            self.start_dist_to_current_wp = math.sqrt((self.current_pos[0]-wp[0])**2 + (self.current_pos[1]-wp[1])**2 + (self.current_pos[2]-wp[2])**2)

    def odom_cb(self, msg):
        new_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        if self.last_pos is not None:
            step = math.sqrt((new_pos[0]-self.last_pos[0])**2 + (new_pos[1]-self.last_pos[1])**2 + (new_pos[2]-self.last_pos[2])**2)
            self.total_distance_traveled += step
        self.last_pos = new_pos
        self.current_pos = new_pos
        
        if self.start_dist_to_current_wp < 0 and self.current_wp_index < len(self.waypoints):
            self.update_start_distance()
            
        if self.current_wp_index < len(self.waypoints):
            wp = self.waypoints[self.current_wp_index]
            dist = math.sqrt((self.current_pos[0]-wp[0])**2 + (self.current_pos[1]-wp[1])**2 + (self.current_pos[2]-wp[2])**2)
            if dist < 4.0:
                self.current_wp_index += 1
                self.update_start_distance() 

    def bumper_cb(self, msg):
        if msg.contacts:
            curr_time = time.time()
            if curr_time - self.last_impact_time > 5.0:
                self.impacts += 1
                self.last_impact_time = curr_time

def run_episode(env_node, sim_manager, weights):
    with open(f"{sim_manager.home_dir}/VFH_Project/vfh_rl_weights.txt", 'w') as f:
        f.write(" ".join(map(str, weights)))
        
    env_node.load_waypoints()
    env_node.impacts = 0
    env_node.last_impact_time = 0.0 
    env_node.current_wp_index = 0
    env_node.start_dist_to_current_wp = -1.0 
    env_node.total_distance_traveled = 0.0
    env_node.last_pos = None
    
    sim_manager.start_vfh_node()
    start_time = time.time()
    max_impacts_allowed = 1  
    mission_success = False
    timeout_occurred = False 
    
    while True:
        current_time = time.time()
        time.sleep(0.1) 
        elapsed_time = current_time - start_time
        
        if env_node.current_wp_index >= len(env_node.waypoints):
            mission_success = True
            break
        if env_node.impacts >= max_impacts_allowed:
            break
        if elapsed_time > 240.0:
            timeout_occurred = True
            break

    sim_manager.stop_vfh_node()
    time.sleep(1) 
    
    # Core reward calculation
    reward = (env_node.current_wp_index * 15000.0)
    if env_node.impacts >= 1: reward -= 20000.0
    if timeout_occurred: reward -= 10000.0
    if mission_success: reward += 15000.0
    
    reward -= (elapsed_time * 10.0) 
    
    # 3D Euclidean distance approach reward
    if env_node.current_wp_index < len(env_node.waypoints) and env_node.start_dist_to_current_wp > 0:
        wp = env_node.waypoints[env_node.current_wp_index]
        final_dist = math.sqrt((env_node.current_pos[0]-wp[0])**2 + (env_node.current_pos[1]-wp[1])**2 + (env_node.current_pos[2]-wp[2])**2)
        distance_covered = env_node.start_dist_to_current_wp - final_dist
        if distance_covered > 0:
            reward += (distance_covered * 25.0) 
            
    reward -= (env_node.total_distance_traveled * 0.5) 
        
    return {
        'reward': reward,
        'impacts': env_node.impacts,
        'waypoints_reached': env_node.current_wp_index,
        'time': elapsed_time / 60.0 
    }

def main():
    rclpy.init()
    sim_manager = SimulationManager()
    env_node = TethysEnvNode(sim_manager.home_dir)
    
    es = EvolutionStrategyOptimizer(num_params=8, pop_size=8)
    spin_thread = threading.Thread(target=rclpy.spin, args=(env_node,))
    spin_thread.start()
    
    micro_ckpt_path = f"{sim_manager.home_dir}/VFH_Project/vfh_micro_checkpoint.npy"
    resume_e_idx, resume_m_idx = 0, 0
    mutated_thetas = None
    gen_metrics = [] 
    
    if os.path.exists(micro_ckpt_path):
        try:
            data = np.load(micro_ckpt_path, allow_pickle=True).item()
            if data['gen'] == es.current_gen:
                mutated_thetas = data['mutated_thetas']
                gen_metrics = data['gen_metrics']
                es.epsilon = data['epsilon']
                resume_e_idx = data['e_idx']
                resume_m_idx = data['m_idx']
                if resume_e_idx >= 10:
                    resume_e_idx, resume_m_idx, mutated_thetas = 0, 0, None
        except: pass
    
    generations = 35 
    env_modes = ['random', 'canyon', 'random', 'canyon', 'random', 'canyon', 'random', 'canyon', 'random', 'canyon']
    
    try:
        while es.current_gen < generations:
            if mutated_thetas is None:
                print(f"\nGENERATION {es.current_gen+1}/{generations}")
                mutated_thetas = es.ask()
                gen_metrics = [[None for _ in range(len(env_modes))] for _ in range(es.pop_size)]
                resume_e_idx, resume_m_idx = 0, 0
            
            for e_idx in range(resume_e_idx, len(env_modes)):
                mode = env_modes[e_idx]
                if resume_m_idx == 0:
                    sim_manager.generate_world(mode=mode)
                
                for m_idx in range(resume_m_idx, len(mutated_thetas)):
                    weights = mutated_thetas[m_idx]
                    print(f"\n   [Mutant {m_idx+1}/{es.pop_size}] Map {e_idx+1}/10 -> Weights: {np.round(weights, 2)}")
                    
                    sim_manager.start_simulation() 
                    metrics = run_episode(env_node, sim_manager, weights)
                    gen_metrics[m_idx][e_idx] = metrics
                    
                    print(f"     => Test Result: Reward {np.round(metrics['reward'], 2)} | "
                          f"WP: {metrics['waypoints_reached']} | Impacts: {metrics['impacts']} | Time: {metrics['time']:.2f} min")
                    
                    next_m_idx = m_idx + 1
                    next_e_idx = e_idx
                    if next_m_idx == es.pop_size:
                        next_m_idx, next_e_idx = 0, e_idx + 1
                        
                    np.save(micro_ckpt_path, {
                        'gen': es.current_gen, 'mutated_thetas': mutated_thetas, 'epsilon': es.epsilon, 
                        'gen_metrics': gen_metrics, 'e_idx': next_e_idx, 'm_idx': next_m_idx
                    })
                    
                    sim_manager.stop_simulation()
                    gc.collect() 
                
                resume_m_idx = 0
                
            print("\nGENERATION RESULTS")
            episode_metrics = []
            for m_idx in range(es.pop_size):
                m_envs = gen_metrics[m_idx]
                episode_metrics.append({
                    'reward': np.mean([m['reward'] for m in m_envs]),
                    'time': np.mean([m['time'] for m in m_envs]),
                    'impacts': np.mean([m['impacts'] for m in m_envs]),
                    'waypoints_reached': np.mean([m['waypoints_reached'] for m in m_envs])
                })
                print(f" Mutant {m_idx+1} => Avg Reward: {episode_metrics[-1]['reward']:.2f} | WP: {episode_metrics[-1]['waypoints_reached']:.2f}")
                
            best_mutant_idx = np.argmax([m['reward'] for m in episode_metrics])
            es.tell([m['reward'] for m in episode_metrics], episode_metrics[best_mutant_idx], es.theta.copy())
            
            if os.path.exists(micro_ckpt_path): os.remove(micro_ckpt_path)
            mutated_thetas = None
            
    except KeyboardInterrupt:
        print("\n[!] Training interrupted by user.")
    finally:
        sim_manager.stop_simulation()
        sim_manager.stop_vfh_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
