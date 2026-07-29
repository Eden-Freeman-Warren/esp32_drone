import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    """Discrete-time PID controller for temperature regulation."""
    def __init__(self, kp, ki, kd, dt, output_limits=(0.0, 100.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.min_out, self.max_out = output_limits
        
        # State tracking
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, current_value):
        # 1. Error calculation
        error = setpoint - current_value
        
        # 2. Discrete terms
        proportional = self.kp * error
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Anti-windup: clamp integral term to prevent saturation lag
        self.integral = max(min(self.integral, self.max_out / (self.ki + 1e-6)), 
                            self.min_out / (self.ki + 1e-6))
        
        # 3. Summing total control effort
        output = proportional + (self.ki * self.integral) + (self.kd * derivative)
        
        # 4. Limit output to physical hardware capability (0-100% power)
        output = max(min(output, self.max_out), self.min_out)
        
        self.prev_error = error
        return output

class ThermalPlant:
    """Simulates physical heat transfer (First-Order Plus Dead Time)."""
    def __init__(self, gain, time_constant, dead_time, initial_temp, ambient_temp, dt):
        self.K = gain
        self.tau = time_constant
        self.theta = int(dead_time / dt)  # convert dead time to step queue size
        self.ambient_temp = ambient_temp
        self.temp = initial_temp
        self.dt = dt
        
        # Input history buffer to handle dead time latency
        self.input_history = [0.0] * (self.theta + 1)

    def update(self, u):
        # Track historical inputs for dead time simulation
        self.input_history.append(u)
        delayed_u = self.input_history.pop(0)
        
        # Euler discretization of the thermal differential equation:
        # dT/dt = (1/tau) * (K * u - (T - T_ambient))
        dT = (1.0 / self.tau) * (self.K * delayed_u - (self.temp - self.ambient_temp)) * self.dt
        self.temp += dT
        return self.temp

# --- Simulation Configurations ---
duration = 400.0   # Total simulation time in seconds
dt = 0.1           # Sampling time interval in seconds
steps = int(duration / dt)

# Instantiate the physical thermal environment
plant = ThermalPlant(gain=0.8, time_constant=40.0, dead_time=5.0, 
                     initial_temp=20.0, ambient_temp=20.0, dt=dt)

# Instantiate the PID loop (tuned to accommodate a 5-second physical delay)
pid = PIDController(kp=4.5, ki=0.12, kd=12.0, dt=dt, output_limits=(0.0, 100.0))

# Pre-allocate data structures for visualization
time_axis = np.linspace(0, duration, steps)
temp_history = np.zeros(steps)
pwm_history = np.zeros(steps)
setpoint_history = np.zeros(steps)

# Run Loop
for k in range(steps):
    t = time_axis[k]
    
    # Define step changes in target temperature profiles
    if t < 10.0:
        setpoint = 20.0
    elif t < 220.0:
        setpoint = 75.0   # Target heat step up
    else:
        setpoint = 50.0   # Target drop cool down
        
    current_temp = plant.temp
    pwm_power = pid.compute(setpoint, current_temp)
    plant.update(pwm_power)
    
    # Store records
    temp_history[k] = current_temp
    pwm_history[k] = pwm_power
    setpoint_history[k] = setpoint

# --- Plotting Results ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

ax1.plot(time_axis, setpoint_history, 'r--', label='Target Temperature (°C)')
ax1.plot(time_axis, temp_history, 'b-', label='Measured Temperature (°C)')
ax1.set_ylabel('Temperature (°C)')
ax1.grid(True)
ax1.legend(loc='upper right')
ax1.set_title('Thermal System PID Control Loop Simulation')

ax2.plot(time_axis, pwm_history, 'g-', label='Heater Power Duty Cycle (%)')
ax2.set_ylabel('PWM Output (%)')
ax2.set_xlabel('Time (seconds)')
ax2.grid(True)
ax2.legend(loc='upper right')

plt.tight_layout()
plt.show()
