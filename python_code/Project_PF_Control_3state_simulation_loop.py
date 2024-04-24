

import numpy as np
#from numpy.random import normal
from numpy.random import rand
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

class CameraParticleFilter:
    def __init__(self, space, num_particles, initial_state, state_cov, measurement_cov):
        self.num_particles = num_particles
        self.space=space
        self.particles = np.array([initial_state + np.random.normal(scale=np.sqrt(state_cov), size=3) for _ in range(num_particles)])
        # Equal Weights
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.state_cov = state_cov #State covariance
        self.measurement_cov = measurement_cov  #Measurment Covariance
        self.N_eff = 0
        self.true_state=initial_state
    
    def predict(self, control_signal, dt):
        for i in range(self.num_particles):
            directed_motion = control_signal * dt
            noise = np.random.normal(scale=np.sqrt(self.state_cov), size=3)
            self.particles[i] +=  directed_motion+ noise
           
           
        # Define the minimum and maximum bounds for each dimension
        min_bounds = np.array([-self.space[0], -self.space[1],-self.space[2]])
        max_bounds = np.array([self.space[0], self.space[1], self.space[2]])

        # Use np.clip() to ensure that all particles are within the specified bounds
        self.particles = np.clip(self.particles, min_bounds, max_bounds)




    def update(self, measurement):
        for i in range(self.num_particles):
            likelihood = multivariate_normal.pdf(measurement, self.particles[i], self.measurement_cov)
            self.weights[i] *= likelihood
        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= np.sum(self.weights)  # normalize
    
    def resample(self):
        #indices = np.random.choice(self.num_particles, size=self.num_particles, p=self.weights)
        #self.particles = self.particles[indices]
        #self.weights = np.ones(self.num_particles) / self.num_particles

        """
        low variance resampling
        """
        W = np.cumsum(self.particles)
        r = rand(1) / self.num_particles
        j = 1
        for i in range(self.num_particles):
            u = r + (i - 1) / self.num_particles
            while u > W[j]:
                j = j + 1
            self.particles[i, :] = self.particles[j, :]
            self.weights[i] = 1 / self.num_particles
    
    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0) #weighted average
    
    def step(self, measurement, control_signal, dt):
        self.predict(control_signal, dt)
        self.update(measurement)
        self.N_eff = 1.0 / np.sum(np.square(self.weights))
        if self.N_eff < self.num_particles / 2:
            self.resample()
        return self.estimate()

# Initialize Particle Filter
num_particles = 1000
space=[1000,1000,1000]
initial_state = np.array([-26.6, -13.2, 64.8])  # Camera starts at this state
state_cov = np.array([8.7, 1.5, 8.7])  # State covariance for position 
measurement_cov = np.diag([9.8, 1.0, 8.8])  # Measurement noise covariance
pf = CameraParticleFilter(space,num_particles, initial_state, state_cov, measurement_cov)

# Control Signal for Moving towards Original Position
target_state = np.array([67.8, -14.3, 115.5]) #The state we want to return to
dt = 1  # Time step
#num_steps=70

estimated_states = []
measurements = []
controls = []
true_states = []
true_controls=[]
i=0

while not np.allclose(pf.estimate(), target_state, atol=2.0): # Run for 100 steps
    # Simulated measurement from camera sensors
    sim_measurement_noise = np.random.normal(scale=np.sqrt(np.diag(measurement_cov)), size=3)
    measurement = pf.estimate() + sim_measurement_noise

    # Compute Control Signal
    estimated_state = pf.estimate()
    estimated_states.append(estimated_state)
    
    control_signal = (target_state - estimated_state) * 0.5  # Proportional control #0.1 control gain
    
    true_control_signal = (target_state - pf.true_state) * 0.5
    pf.true_state=pf.true_state+ true_control_signal
    
    # Particle Filter Step
    estimated_state = pf.step(measurement, control_signal, dt)
    #estimated_states.append(estimated_state)
    measurements.append(measurement)
    true_states.append(pf.true_state)
    true_controls.append(np.linalg.norm(true_control_signal))
    controls.append(np.linalg.norm(control_signal))
    #controls.append((control_signal))

    print("Estimated State:", estimated_state)
    i=i+1


# Convert lists to numpy arrays for plotting
#states = np.array(states)
#measurements = np.array(measurements)
#true_states = np.array(true_states)
controls = np.array(controls)
num_steps=i
#steps = np.arange(num_steps)





# Convert lists to numpy arrays for plotting
estimated_states = np.array(estimated_states)
measurements = np.array(measurements)
controls = np.array(controls)
true_controls = np.array(true_controls)
true_states = np.array(true_states)
#num_steps=i
steps = np.arange(num_steps)


print("controls shape", controls.shape)
print("states shape", estimated_states.shape)
print("measurments shape", measurements.shape)
print("controls", controls)
print("measurements", measurements)
print("states shape", estimated_states)
print("num_steps", num_steps)

# Plotting no 1
fig1, axs = plt.subplots(3, 1, figsize=(10, 15))  # 3 rows for x, y, z components

# X Component Plot
axs[0].plot(steps, estimated_states[:, 0], 'r--', label='Particle Filter Estimated X')
axs[0].plot(steps, true_states[:, 0], 'ro', label='Ground Truth X', alpha=0.6)
axs[0].set_title('X Component-Ground Truth vs Estimated Particle Filter state ')
axs[0].set_xlabel('No of Steps')
axs[0].set_ylabel('X Value')
axs[0].legend()
axs[0].grid(True)

# Y Component Plot
axs[1].plot(steps, estimated_states[:, 1], 'g--', label='Particle Filter Estimated Y')
axs[1].plot(steps, true_states[:, 1], 'go', label='Ground Truth X', alpha=0.6)
axs[1].set_title('Y Component-Ground Truth vs Estimated Particle Filter state')
axs[1].set_xlabel('No of Steps')
axs[1].set_ylabel('Y Value')
axs[1].legend()
axs[1].grid(True)

# Z Component Plot
axs[2].plot(steps, estimated_states[:, 2], 'b--', label='Particle Filter Estimated Z')
axs[2].plot(steps, true_states[:, 2], 'bo', label='Ground Truth X', alpha=0.6)
axs[2].set_title('Z Component-Ground Truth vs Estimated Particle Filter state')
axs[2].set_xlabel('No of Steps')
axs[2].set_ylabel('Z Value')
axs[2].legend()
axs[2].grid(True)

# Display the plots
plt.subplots_adjust(left=0.1, right=0.9, top=0.8, bottom=0.2, hspace=0.6, wspace=0.6)
#plt.tight_layout()
plt.show()

#########################
plt.figure(figsize=(10, 6))

# Plotting the X component of states and measurements
plt.plot(steps, estimated_states[:, 0], 'r--', label='Estimated State X', marker=None)  # Red dashed line for X states
plt.plot(steps, true_states[:, 0], 'rx', label='Ground Truth X', alpha=0.7)  # Red 'x' markers for X measurements

# Plotting the Y component of states and measurements
plt.plot(steps, estimated_states[:, 1], 'g--', label='Estimated State Y', marker=None)  # Green dashed line for Y states
plt.plot(steps, true_states[:, 1], 'gx', label='Ground Truth Y', alpha=0.7)  # Green 'x' markers for Y measurements

# Plotting the Z component of states and measurements
plt.plot(steps, estimated_states[:, 2], 'b--', label='Estimated State Z', marker=None)  # Blue dashed line for Z states
plt.plot(steps, true_states[:, 2], 'bx', label='Ground Truth Z', alpha=0.7)  # Blue 'x' markers for Z measurements

plt.title('Ground Truth vs Particle Filter Estimation')
plt.xlabel('Steps')
plt.ylabel('State Value')
plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
plt.grid(True)
plt.tight_layout() 
plt.show()


####################
# Set the width for the bars
bar_width = 0.35

# Creating a new figure for the bar graph
plt.figure(figsize=(10, 6))

# Creating a bar graph for controls
# Shift each bar to the left by half the width of the bar
plt.bar(steps[1:] - bar_width/2, controls[1:], width=bar_width, color='blue', alpha=0.7, label='Control')

# Creating a bar graph for true_controls
# Shift each bar to the right by half the width of the bar
plt.bar(steps[1:] + bar_width/2, true_controls[1:], width=bar_width, color='green', alpha=0.7, label='Ground Truth Control')

# Adding titles and labels
plt.title('Control Input vs Ground Truth Control Input at Each Step')
plt.xlabel('Steps')
plt.ylabel('Control Magnitude')

# Adding a legend to distinguish between the datasets
plt.legend()
plt.show()
##################



