import random

# Set the initial value for z
z = 0

# Create an empty list to store the position data
position_data = []

# Generate 100 position data points
for i in range(100):
  # Increment z by 1
  z += 1
  z_temp = z;
  # Add some random noise to x, y, and z
  x = 0 + random.uniform(-1.2, 0.7)
  y = 0 + random.uniform(-0.3, 1)
  z_temp += random.uniform(-1, 1.5)
  # Append the position data point to the list
  print('[',x,',', y,',', z_temp,']',',')
 
