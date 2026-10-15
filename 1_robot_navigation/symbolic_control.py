"""
Symbolic Control of Nonlinear Systems
=====================================
Implementation of the discrete abstraction, symbolic controller synthesis,
and closed-loop simulation based on:
"Approches symboliques pour le contrôle des systèmes non linéaires"
by Antoine Girard, Pierre-Jean Meyer, Adnane Saoud (2024).

System: Discrete-time unicycle mobile robot:
  x1(t+1) = x1(t) + T * u1(t) * cos(x3(t)) + T * w1(t)
  x2(t+1) = x2(t) + T * u1(t) * sin(x3(t)) + T * w2(t)
  x3(t+1) = x3(t) + T * u2(t) + T * w3(t)  (wrapped in [-pi, pi])

Specification:
  "Visit R1 or R2 (exclusive), then reach R3, while avoiding R4 (obstacle)."
"""

import os
import time
import numpy as np
from numba import njit

# ---------------------------------------------------------------------------
# Default Constants matching the tutorial paper (Table 1, Table 2, Table 3)
# ---------------------------------------------------------------------------
DEFAULT_SAMPLING_TIME = 1.0  # T = 1.0 s

DEFAULT_BOUND_X = np.array([
    [0.0, 10.0],       # x1 in [0, 10] m
    [0.0, 10.0],       # x2 in [0, 10] m
    [-np.pi, np.pi]    # x3 (heading) in [-pi, pi] rad
], dtype=np.float64)

DEFAULT_BOUND_U = np.array([
    [0.25, 1.0],       # u1 (linear velocity) in [0.25, 1.0] m/s
    [-1.0, 1.0]        # u2 (angular velocity) in [-1.0, 1.0] rad/s
], dtype=np.float64)

DEFAULT_BOUND_W = np.array([
    [-0.05, 0.05],     # w1 in [-0.05, 0.05] m/s
    [-0.05, 0.05],     # w2 in [-0.05, 0.05] m/s
    [-0.05, 0.05]      # w3 in [-0.05, 0.05] rad/s
], dtype=np.float64)

# Regions: R = [x1_min, x1_max; x2_min, x2_max]
REGIONS = {
    'R1': {'x': (4.0, 5.0), 'y': (8.5, 9.5), 'color': '#EAB308', 'label': 'Waymark 1 (R1)'},
    'R2': {'x': (8.5, 9.5), 'y': (2.0, 3.0), 'color': '#06B6D4', 'label': 'Waymark 2 (R2)'},
    'R3': {'x': (2.0, 3.0), 'y': (0.5, 1.5), 'color': '#10B981', 'label': 'Final Goal (R3)'},
    'R4': {'x': (3.0, 7.0), 'y': (3.0, 7.0), 'color': '#EF4444', 'label': 'Obstacle (R4)'}
}

# Automaton transition relation A_s (1-indexed in paper, 1..5)
# Labels: 1 = Free, 2 = R1, 3 = R2, 4 = R3, 5 = R4
AUTOMATON_TRANSITIONS = np.array([
    [1, 2, 3, 1, 5],  # State 1: Start (neither R1 nor R2 visited)
    [2, 2, 5, 4, 5],  # State 2: R1 visited, now seek R3 (avoid R2 & R4)
    [3, 5, 3, 4, 5],  # State 3: R2 visited, now seek R3 (avoid R1 & R4)
    [4, 4, 4, 4, 4],  # State 4: Target reached (SUCCESS!)
    [5, 5, 5, 5, 5]   # State 5: Sink error state (VIOLATION)
], dtype=np.int32)

AUTOMATON_STATE_DESCRIPTIONS = {
    1: "q1: Initial State (Seeking R1 or R2, avoiding R4)",
    2: "q2: R1 Visited (Seeking R3, avoiding R2 & R4)",
    3: "q3: R2 Visited (Seeking R3, avoiding R1 & R4)",
    4: "q4: Goal Reached! (Specification Satisfied)",
    5: "q5: Sink Trap! (Specification Violated)"
}

TARGET_STATE = 4
SINK_STATE = 5
NUM_AUTOMATON_STATES = 5

CACHE_FILE = os.path.join(os.path.dirname(__file__), 'controller_cache_100x100x30.npz')


# ---------------------------------------------------------------------------
# Numba JIT-Compiled Backward Reachability Synthesis Kernel
# ---------------------------------------------------------------------------
@njit
def _synthesize_step(V, Vp, h2, c1_min, c1_max, c2_min, c2_max, c3_min, c3_max,
                     valid_u, lab_2d, A_s, n1, n2, n3, n_sigma, n_s, iter_num):
    """
    Executes one iteration of controlled backward reachability fixed-point.
    Returns the number of new states proven controllable in this step.
    """
    changes = 0
    for i1 in range(n1):
        for i2 in range(n2):
            lab = lab_2d[i1, i2]
            for i3 in range(n3):
                for psi in range(1, n_s + 1):
                    # Already controllable
                    if V[psi - 1, i1, i2, i3] < 1e8:
                        continue
                    
                    psi_succ = A_s[psi - 1, lab - 1]
                    
                    # If control at successor automaton state already resolved
                    if h2[psi_succ - 1, i1, i2, i3] != 0:
                        V[psi - 1, i1, i2, i3] = iter_num
                        changes += 1
                        continue
                    
                    # Search for an enabling control sigma whose reachable box is safe
                    found_sigma = 0
                    for sigma in range(n_sigma):
                        if not valid_u[i1, i2, i3, sigma]:
                            continue
                        
                        r1_lo = c1_min[i1, i3, sigma] - 1
                        r1_hi = c1_max[i1, i3, sigma] - 1
                        r2_lo = c2_min[i2, i3, sigma] - 1
                        r2_hi = c2_max[i2, i3, sigma] - 1
                        r3_lo = c3_min[i3, sigma] - 1
                        r3_hi = c3_max[i3, sigma] - 1
                        
                        all_ok = True
                        if r3_lo <= r3_hi:
                            for k1 in range(r1_lo, r1_hi + 1):
                                if not all_ok:
                                    break
                                for k2 in range(r2_lo, r2_hi + 1):
                                    if not all_ok:
                                        break
                                    for k3 in range(r3_lo, r3_hi + 1):
                                        if Vp[psi_succ - 1, k1, k2, k3] > 1e7:
                                            all_ok = False
                                            break
                        else:
                            # Angle wraps around [-pi, pi]
                            for k1 in range(r1_lo, r1_hi + 1):
                                if not all_ok:
                                    break
                                for k2 in range(r2_lo, r2_hi + 1):
                                    if not all_ok:
                                        break
                                for k3 in range(r3_lo, n3):
                                    if Vp[psi_succ - 1, k1, k2, k3] > 1e7:
                                        all_ok = False
                                        break
                                if not all_ok:
                                    break
                                for k3 in range(0, r3_hi + 1):
                                    if Vp[psi_succ - 1, k1, k2, k3] > 1e7:
                                        all_ok = False
                                        break
                        
                        if all_ok:
                            found_sigma = sigma + 1
                            break
                    
                    if found_sigma > 0:
                        V[psi - 1, i1, i2, i3] = iter_num
                        h2[psi_succ - 1, i1, i2, i3] = found_sigma
                        changes += 1
                        
    return changes


# ---------------------------------------------------------------------------
# Symbolic Controller Manager Class
# ---------------------------------------------------------------------------
class SymbolicController:
    """
    Manages discretization, abstraction, controller synthesis, and continuous simulation.
    """
    def __init__(self, n_x=None, n_u=None, T=DEFAULT_SAMPLING_TIME):
        self.T = T
        self.bound_x = DEFAULT_BOUND_X.copy()
        self.bound_u = DEFAULT_BOUND_U.copy()
        self.bound_w = DEFAULT_BOUND_W.copy()
        
        # Grid resolutions
        self.n_x = np.array([100, 100, 30], dtype=np.int32) if n_x is None else np.array(n_x, dtype=np.int32)
        self.n_u = np.array([3, 5], dtype=np.int32) if n_u is None else np.array(n_u, dtype=np.int32)
        
        self.d_x = (self.bound_x[:, 1] - self.bound_x[:, 0]) / self.n_x
        self.d_u = (self.bound_u[:, 1] - self.bound_u[:, 0]) / (self.n_u - 1)
        self.d_w = self.bound_w[:, 1] - self.bound_w[:, 0]
        
        # Generate discrete control inputs (n_sigma = 15)
        self.val_u = self._generate_inputs()
        self.n_sigma = self.val_u.shape[1]
        
        # Cell coordinate centers
        self.x1_c = self.bound_x[0, 0] + (np.arange(self.n_x[0]) + 0.5) * self.d_x[0]
        self.x2_c = self.bound_x[1, 0] + (np.arange(self.n_x[1]) + 0.5) * self.d_x[1]
        self.x3_c = self.bound_x[2, 0] + (np.arange(self.n_x[2]) + 0.5) * self.d_x[2]
        
        # Custom regions dictionary
        self.regions = {k: {'x': v['x'], 'y': v['y'], 'color': v['color'], 'label': v['label']}
                        for k, v in REGIONS.items()}
        
        # Compute 2D Label matrix
        self.lab_2d = self._compute_label_matrix()
        
        # Controller structures
        self.V = None   # Value function (n_s, n1, n2, n3)
        self.h2 = None  # Control lookup table (n_s, n1, n2, n3)
        self.is_synthesized = False
        self.synthesis_iterations = 0
        self.synthesis_time = 0.0

    def _generate_inputs(self):
        """Generates grid of discrete input vectors matching MATLAB ordering."""
        val_u = []
        p_u = [1, self.n_u[0], self.n_u[0] * self.n_u[1]]
        for i in range(1, self.n_u[0] * self.n_u[1] + 1):
            c1 = ((i - 1) % p_u[1]) // p_u[0] + 1
            c2 = ((i - 1) % p_u[2]) // p_u[1] + 1
            u_val = self.bound_u[:, 0] + np.array([c1 - 1, c2 - 1]) * self.d_u
            val_u.append(u_val)
        return np.array(val_u).T  # Shape: (2, 15)

    def set_regions(self, new_regions):
        """Allows dynamic repositioning of obstacles, waypoints, and target."""
        self.regions = new_regions
        self.lab_2d = self._compute_label_matrix()
        self.is_synthesized = False

    def _compute_label_matrix(self):
        """Computes 2D label matrix: 1:Free, 2:R1, 3:R2, 4:R3, 5:Obstacles."""
        n1, n2 = self.n_x[0], self.n_x[1]
        lab = np.ones((n1, n2), dtype=np.int32)
        
        r1_x, r1_y = self.regions.get('R1', {}).get('x', (0,0)), self.regions.get('R1', {}).get('y', (0,0))
        r2_x, r2_y = self.regions.get('R2', {}).get('x', (0,0)), self.regions.get('R2', {}).get('y', (0,0))
        r3_x, r3_y = self.regions.get('R3', {}).get('x', (0,0)), self.regions.get('R3', {}).get('y', (0,0))
        
        # Collect all obstacles (R4 and any custom obstacle starting with 'Obs' or 'R4')
        obstacles = [v for k, v in self.regions.items() if k == 'R4' or k.startswith('Obs') or v.get('type') == 'obstacle']
        
        for i1 in range(n1):
            x1 = self.x1_c[i1]
            for i2 in range(n2):
                x2 = self.x2_c[i2]
                # Check obstacles first (label 5)
                is_obs = False
                for obs in obstacles:
                    ox, oy = obs['x'], obs['y']
                    if ox[0] <= x1 <= ox[1] and oy[0] <= x2 <= oy[1]:
                        lab[i1, i2] = 5
                        is_obs = True
                        break
                if is_obs:
                    continue
                
                if r1_x[0] <= x1 <= r1_x[1] and r1_y[0] <= x2 <= r1_y[1]:
                    lab[i1, i2] = 2
                elif r2_x[0] <= x1 <= r2_x[1] and r2_y[0] <= x2 <= r2_y[1]:
                    lab[i1, i2] = 3
                elif r3_x[0] <= x1 <= r3_x[1] and r3_y[0] <= x2 <= r3_y[1]:
                    lab[i1, i2] = 4
        return lab

    def get_controllable_mask(self, psi=1):
        """Returns 2D boolean mask (n1, n2) of controllable positions for given automaton state."""
        if self.V is None:
            return np.zeros((self.n_x[0], self.n_x[1]), dtype=bool)
        # Min cost over all headings theta
        min_v = np.min(self.V[psi - 1], axis=-1)
        return min_v < 1e7

    def load_cached_controller(self):
        """Loads precomputed controller from disk if resolution matches."""
        if os.path.exists(CACHE_FILE) and np.array_equal(self.n_x, [100, 100, 30]):
            try:
                data = np.load(CACHE_FILE)
                self.V = data['V']
                self.h2 = data['h2']
                self.is_synthesized = True
                self.synthesis_iterations = 35
                self.synthesis_time = 0.05
                return True
            except Exception as e:
                print("Failed to load cache:", e)
        return False

    def synthesize(self, progress_callback=None):
        """
        Runs symbolic abstraction and backward reachability synthesis.
        Allows real-time progress callbacks for GUI updates.
        """
        t0 = time.time()
        
        # 1. Reachable set deviation bounds
        d_x_succ = np.zeros((3, self.n_sigma))
        for s in range(self.n_sigma):
            u = self.val_u[:, s]
            Jf_x = np.array([[1.0, 0.0, self.T * abs(u[0])],
                             [0.0, 1.0, self.T * abs(u[0])],
                             [0.0, 0.0, 1.0]])
            Jf_w = np.eye(3) * self.T
            d_x_succ[:, s] = 0.5 * (Jf_x @ self.d_x) + 0.5 * (Jf_w @ self.d_w)

        # 2. Angle bounds
        c3_min = np.zeros((self.n_x[2], self.n_sigma), dtype=np.int32)
        c3_max = np.zeros((self.n_x[2], self.n_sigma), dtype=np.int32)
        for s in range(self.n_sigma):
            for i3 in range(self.n_x[2]):
                r_min = self.x3_c[i3] + self.T * self.val_u[1, s] - d_x_succ[2, s]
                r_max = self.x3_c[i3] + self.T * self.val_u[1, s] + d_x_succ[2, s]
                if r_min < -np.pi and r_max >= -np.pi:
                    r_min += 2 * np.pi
                elif r_min < -np.pi and r_max < -np.pi:
                    r_min += 2 * np.pi
                    r_max += 2 * np.pi
                elif r_max > np.pi and r_min <= np.pi:
                    r_max -= 2 * np.pi
                elif r_max > np.pi and r_min > np.pi:
                    r_max -= 2 * np.pi
                    r_min -= 2 * np.pi
                c3_min[i3, s] = int(np.floor((r_min - self.bound_x[2, 0]) / self.d_x[2])) + 1
                c3_max[i3, s] = int(np.ceil((r_max - self.bound_x[2, 0]) / self.d_x[2]))

        # 3. Coordinate x1 bounds & validity
        c1_min = np.zeros((self.n_x[0], self.n_x[2], self.n_sigma), dtype=np.int32)
        c1_max = np.zeros((self.n_x[0], self.n_x[2], self.n_sigma), dtype=np.int32)
        valid1 = np.zeros((self.n_x[0], self.n_x[2], self.n_sigma), dtype=bool)
        for s in range(self.n_sigma):
            for i3 in range(self.n_x[2]):
                delta = self.T * self.val_u[0, s] * np.cos(self.x3_c[i3])
                r_min = self.x1_c + delta - d_x_succ[0, s]
                r_max = self.x1_c + delta + d_x_succ[0, s]
                c1_min[:, i3, s] = np.floor((r_min - self.bound_x[0, 0]) / self.d_x[0]).astype(np.int32) + 1
                c1_max[:, i3, s] = np.ceil((r_max - self.bound_x[0, 0]) / self.d_x[0]).astype(np.int32)
                valid1[:, i3, s] = (r_min >= self.bound_x[0, 0]) & (r_max <= self.bound_x[0, 1])

        # 4. Coordinate x2 bounds & validity
        c2_min = np.zeros((self.n_x[1], self.n_x[2], self.n_sigma), dtype=np.int32)
        c2_max = np.zeros((self.n_x[1], self.n_x[2], self.n_sigma), dtype=np.int32)
        valid2 = np.zeros((self.n_x[1], self.n_x[2], self.n_sigma), dtype=bool)
        for s in range(self.n_sigma):
            for i3 in range(self.n_x[2]):
                delta = self.T * self.val_u[0, s] * np.sin(self.x3_c[i3])
                r_min = self.x2_c + delta - d_x_succ[1, s]
                r_max = self.x2_c + delta + d_x_succ[1, s]
                c2_min[:, i3, s] = np.floor((r_min - self.bound_x[1, 0]) / self.d_x[1]).astype(np.int32) + 1
                c2_max[:, i3, s] = np.ceil((r_max - self.bound_x[1, 0]) / self.d_x[1]).astype(np.int32)
                valid2[:, i3, s] = (r_min >= self.bound_x[1, 0]) & (r_max <= self.bound_x[1, 1])

        valid_u = valid1[:, None, :, :] & valid2[None, :, :, :]

        # 5. Initialize value function and controller table
        self.V = np.full((NUM_AUTOMATON_STATES, self.n_x[0], self.n_x[1], self.n_x[2]), 1e9, dtype=np.float64)
        self.V[TARGET_STATE - 1, :, :, :] = 0.0  # Final target state V=0
        self.h2 = np.zeros((NUM_AUTOMATON_STATES, self.n_x[0], self.n_x[1], self.n_x[2]), dtype=np.int32)

        # 6. Fixed-Point backward reachability iterations
        it = 0
        while True:
            it += 1
            Vp = self.V.copy()
            changes = _synthesize_step(
                self.V, Vp, self.h2,
                c1_min, c1_max, c2_min, c2_max, c3_min, c3_max,
                valid_u, self.lab_2d, AUTOMATON_TRANSITIONS,
                self.n_x[0], self.n_x[1], self.n_x[2], self.n_sigma, NUM_AUTOMATON_STATES, it
            )
            
            if progress_callback:
                progress_callback(it, changes, int(np.sum(self.V[0] < 1e7)))
                
            if changes == 0 or it > 100:
                break

        self.synthesis_iterations = it
        self.synthesis_time = time.time() - t0
        self.is_synthesized = True

        # Cache if full resolution
        if np.array_equal(self.n_x, [100, 100, 30]):
            np.savez_compressed(CACHE_FILE, V=self.V, h2=self.h2, n_x=self.n_x,
                                n_u=self.n_u, val_u=self.val_u, lab_2d=self.lab_2d)

        return self.synthesis_iterations, self.synthesis_time

    def quantize(self, x):
        """Converts continuous state [x1, x2, x3] into grid cell indices (c1, c2, c3)."""
        c1 = int(np.floor((x[0] - self.bound_x[0, 0]) / self.d_x[0]))
        c2 = int(np.floor((x[1] - self.bound_x[1, 0]) / self.d_x[1]))
        theta = x[2]
        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi
        c3 = int(np.floor((theta - self.bound_x[2, 0]) / self.d_x[2]))
        return np.clip(c1, 0, self.n_x[0] - 1), np.clip(c2, 0, self.n_x[1] - 1), np.clip(c3, 0, self.n_x[2] - 1)

    def get_cell_label(self, c1, c2):
        """Returns 1..5 label for 2D cell coordinate."""
        return self.lab_2d[c1, c2]

    def step_simulation(self, state, psi, with_noise=True, custom_noise=None):
        """
        Executes one discrete-time step of closed-loop continuous dynamics.
        
        Returns:
            next_state (np.ndarray): [x1, x2, x3]
            next_psi (int): Updated automaton state (1..5)
            u (np.ndarray): Control applied [u1, u2]
            w (np.ndarray): Disturbance [w1, w2, w3]
            status (str): "running", "success", "sink", "uncontrollable", "out_of_bounds"
        """
        if not self.is_synthesized:
            raise RuntimeError("Controller has not been synthesized or loaded.")

        # Check bounds
        if not (self.bound_x[0, 0] <= state[0] <= self.bound_x[0, 1] and
                self.bound_x[1, 0] <= state[1] <= self.bound_x[1, 1]):
            return state, SINK_STATE, np.array([0.0, 0.0]), np.array([0.0, 0.0, 0.0]), "out_of_bounds"

        c1, c2, c3 = self.quantize(state)
        lab = self.get_cell_label(c1, c2)
        next_psi = int(AUTOMATON_TRANSITIONS[psi - 1, lab - 1])

        if next_psi == TARGET_STATE:
            return state, TARGET_STATE, np.array([0.0, 0.0]), np.array([0.0, 0.0, 0.0]), "success"
        if next_psi == SINK_STATE:
            return state, SINK_STATE, np.array([0.0, 0.0]), np.array([0.0, 0.0, 0.0]), "sink"

        sigma = int(self.h2[next_psi - 1, c1, c2, c3])
        if sigma == 0:
            return state, next_psi, np.array([0.0, 0.0]), np.array([0.0, 0.0, 0.0]), "uncontrollable"

        u = self.val_u[:, sigma - 1]
        if custom_noise is not None:
            w = np.array(custom_noise, dtype=np.float64)
        elif with_noise:
            w = np.random.uniform(self.bound_w[:, 0], self.bound_w[:, 1])
        else:
            w = np.zeros(3)

        next_state = np.zeros(3)
        next_state[0] = state[0] + self.T * u[0] * np.cos(state[2]) + self.T * w[0]
        next_state[1] = state[1] + self.T * u[0] * np.sin(state[2]) + self.T * w[1]
        next_state[2] = state[2] + self.T * u[1] + self.T * w[2]

        while next_state[2] > np.pi:
            next_state[2] -= 2 * np.pi
        while next_state[2] < -np.pi:
            next_state[2] += 2 * np.pi

        return next_state, next_psi, u, w, "running"

    def simulate_trajectory(self, x0, max_steps=100, with_noise=True, seed=None):
        """Simulates complete closed-loop trajectory from initial state x0."""
        if seed is not None:
            np.random.seed(seed)

        x = np.array(x0, dtype=np.float64)
        psi = 1
        traj = [x.copy()]
        psi_history = [psi]
        controls = []
        noises = []

        status = "running"
        for _ in range(max_steps):
            x_next, psi_next, u, w, status = self.step_simulation(x, psi, with_noise=with_noise)
            if status == "success":
                traj.append(x.copy())
                psi_history.append(TARGET_STATE)
                break
            if status in ("sink", "uncontrollable", "out_of_bounds"):
                traj.append(x_next.copy())
                psi_history.append(psi_next)
                break

            controls.append(u)
            noises.append(w)
            x = x_next
            psi = psi_next
            traj.append(x.copy())
            psi_history.append(psi)

        return {
            'success': (status == "success"),
            'status': status,
            'trajectory': np.array(traj),
            'psi_history': psi_history,
            'controls': np.array(controls) if controls else np.empty((0, 2)),
            'noises': np.array(noises) if noises else np.empty((0, 3)),
            'steps': len(traj) - 1
        }
