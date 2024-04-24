
import numpy as np
from scipy.linalg import block_diag
import matplotlib.pyplot as plt


class CameraControlKF:
    def __init__(self, initial_state, process_noise_std, measurement_noise_std): #we can calculate these noises from the simulated measurments we received in .csv
        # Initial state and covariance
        self.state = initial_state
        self.true_state= initial_state      
        self.P = np.eye(3)  #covariance for a start
        
        # motion and measurement noise covariance matrices
        self.Q = np.diag(process_noise_std**2)
        self.R = np.diag(measurement_noise_std**2)
        
        
        #Fand H are state and measurement matrices of Kalman
        self.F = np.eye(3) 
        self.H = np.eye(3)
        
        # control gain
        self.Kp = 0.5
    
    def predict(self,control_input):
        
        self.state = self.F @ self.state+ control_input
        #self.true_state = self.F @ self.true_state + control_input
        self.P = (self.F @self.P@ self.F.T) + self.Q
    
    def correction(self, measurement):
        
        y = measurement - (self.H @ self.state) #innovation
        # Kalman gain
        S = (self.H @self.P @ self.H.T) + self.R
        K = self.P@ self.H.T @ np.linalg.inv(S)
        
        # State update
        
        self.state = self.state + (K@y)
        I=np.eye(self.P.shape[0])
        self.P = I - (K  @ self.H)@ self.P
    
    def control_input(self, target_state):
        
        control = self.Kp * (target_state - self.state)
        return control
    
    def true_control_input(self, target_state):
        
        control = self.Kp * (target_state - self.true_state)
        return control
    
    #def apply_control(self, control):
    #    # Apply control input to the state (this simulates the movement towards the target)
    #    self.state =self.state+ control
    #    self.true_state += control 
    
    def get_state(self):
        return self.state
    
    def get_true_state(self):
        return self.true_state

# Initialization
target_state = np.array([67.8, -14.3, 115.5])  # Target position  Dummy for now
initial_state = np.array([-26.6, -13.2, 64.8])  # New Displaced Position
process_noise_std = np.array([8.7, 1.5, 8.7])  # In form of std.devs
measurement_noise_std = np.array([9.8, 1.0, 8.8])  # In form of Std. devs

camkf = CameraControlKF(initial_state, process_noise_std, measurement_noise_std)

states = []
measurements = []
true_states = []
controls=[]
true_controls=[]

i=0

# Control loop
while not np.allclose(camkf.get_state(), target_state, atol=2.0): #keep comparing the 2 matrices if within tolerance
    
    
    if (i==0):
    
        # Simulate sensor measurement 
        measurement = camkf.get_state() + np.random.normal(0, measurement_noise_std)
        measurements.append(measurement)

        camkf.state=measurement
        camkf.true_state=measurement

        states.append(camkf.get_state())
        true_states.append(camkf.get_true_state())

        control_signal=camkf.control_input(target_state)
        #controls.append(np.linalg.norm(control_signal))
        controls.append((control_signal))

        true_control_signal=camkf.true_control_input(target_state)
        #true_controls.append(np.linalg.norm(true_control_signal))
        true_controls.append((true_control_signal))
        i=i+1
        #control_signal=(target_state-measurement)*camkf.Kp
    else:
        control_signal=camkf.control_input(target_state)
        #controls.append(np.linalg.norm(control_signal))
        controls.append((control_signal))

        true_control_signal=camkf.true_control_input(target_state)
        #true_controls.append(np.linalg.norm(true_control_signal))
        true_controls.append((true_control_signal))

        camkf.predict(control_signal)
        camkf.true_state=camkf.true_state+true_control_signal
       
       # Simulate sensor measurement 
        measurement = camkf.get_state() + np.random.normal(0, measurement_noise_std)
        measurements.append(measurement)

        camkf.correction(measurement)
            # Current state stimation/calculation with control
        print("                                                                                                                 ")
        print("Estimated current Camera state (x,y,z) from Kalman Filter:", camkf.get_state())
        print("                                                                                                                 ")

        states.append(camkf.get_state())
        true_states.append(camkf.get_true_state())

        i=i+1

       


# Convert lists to numpy arrays for plotting
states = np.array(states)
true_states = np.array(true_states)
measurements = np.array(measurements)
controls = np.array(controls)
true_controls = np.array(true_controls)
num_steps=i
steps = np.arange(num_steps)

##testing the sizes and contents of arrays for plotting
#print("controls shape", controls.shape)
#print("states shape", states.shape)
#print("true states shape", true_states.shape)
#print("measurments shape", measurements.shape)

print("controls", controls)
#print("measurements", measurements)
#print("true states", true_states)
#print("states", states)
#print("num_steps", num_steps)


############################################
# Plotting # Plotting true states vs estimated state
fig1, axs = plt.subplots(3, 1, figsize=(10, 15))  # 3 rows for x, y, z components

# X Component Plot
axs[0].plot(steps, states[:, 0], 'r--', label='Kalman Filter Estimated X')
axs[0].plot(steps, true_states[:, 0], 'ro', label='Ground Truth X')
axs[0].set_title('X Component-Ground Truth vs Estimated Kalman state - Simulation ')
axs[0].set_xlabel('No of Steps')
axs[0].set_ylabel('X Value')
axs[0].legend()
axs[0].grid(True)

# Y Component Plot
axs[1].plot(steps, states[:, 1], 'g--', label='Kalman Filter Estimated Y')
axs[1].plot(steps, true_states[:, 1], 'go', label='Ground Truth Y')
axs[1].set_title('Y Component-Ground Truth vs Estimated Kalman state - Simulation')
axs[1].set_xlabel('No of Steps')
axs[1].set_ylabel('Y Value')
axs[1].legend()
axs[1].grid(True)

# Z Component Plot
axs[2].plot(steps, states[:, 2], 'b--', label='Kalman Filter Estimated Z')
axs[2].plot(steps, true_states[:, 2], 'bo', label='Ground Truth Z')
axs[2].set_title('Z Component-Ground Truth vs Estimated Kalman state - Simulation')
axs[2].set_xlabel('No of Steps')
axs[2].set_ylabel('Z Value')
axs[2].legend()
axs[2].grid(True)

# Display the plots
plt.subplots_adjust(left=0.1, right=0.9, top=0.8, bottom=0.2, hspace=0.6, wspace=0.6)
#plt.tight_layout()
plt.show()

#########################
#########################Plotting true state vs Estimated combined

plt.figure(figsize=(10, 6))

# Plotting the X component of states and measurements
plt.plot(steps, states[:, 0], 'r--', label='Estimated State X', alpha=0.9)
plt.plot(steps, true_states[:, 0], 'ro', label='Ground Truth X', alpha=0.9)


# Plotting the Y component of states and measurements
plt.plot(steps, states[:, 1], 'g--', label='Estimated State Y', alpha=0.9)  # Green dashed line for Y states
plt.plot(steps, true_states[:, 1], 'go', label='Ground Truth Y', alpha=0.9)

# Plotting the Z component of states and measurements
plt.plot(steps, states[:, 2], 'b--', label='Estimated State Z', alpha=0.9)  # Blue dashed line for Z states
plt.plot(steps, true_states[:, 2], 'bo', label='Ground Truth Z', alpha=0.9)


plt.title('Ground Truth vs Estimated Kalman state - Simulation')
plt.xlabel('Steps')
plt.ylabel('State Value')
#plt.legend()  # Display a legend to identify each line
plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
plt.grid(True)
plt.tight_layout() 
plt.show()


####################
##Bar Graph True state control input vs Estimated control input



# Set the width for the bars
bar_width = 0.35

# Creating a new figure for the bar graph
plt.figure(figsize=(10, 6))

# Creating a bar graph for controls
# Shift each bar to the left by half the width of the bar
plt.bar(steps[1:] - bar_width/2, controls[1:,0], width=bar_width, color='blue', alpha=0.7, label='Control at X')

# Creating a bar graph for true_controls
# Shift each bar to the right by half the width of the bar
plt.bar(steps[1:] + bar_width/2, true_controls[1:,0], width=bar_width, color='green', alpha=0.7, label='True Control at X')

# Adding titles and labels
plt.title('Control Input vs Ground Truth Control Input at X at Each Step')
plt.xlabel('Steps')
plt.ylabel('Control Magnitude')

# Adding a legend to distinguish between the datasets
plt.legend()
plt.show()
################################################
