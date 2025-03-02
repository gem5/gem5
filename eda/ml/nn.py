import re

import numpy as np
import tensorflow as tf
from tensorflow.keras.layers import (
    LSTM,
    Dense,
)
from tensorflow.keras.models import Sequential

# Filepath to the address stream file
file_path = '/home/abishek/Desktop/SHELLY/gem5/eda/addr_pc.log'

# Initialize lists to store addresses
addresses = []

# Read the address stream file
with open(file_path, 'r') as file:
    for line in file:
        match = re.match(r'Addr: ([0-9a-fA-F]+)', line)
        if match:
            addr = int(match.group(1), 16)  # Convert hex address to integer
            addresses.append(addr)

# Convert addresses to numpy array
addresses = np.array(addresses)

# Prepare the data for training
sequence_length = 64
X = []
y = []

for i in range(len(addresses) - sequence_length):
    X.append(addresses[i:i + sequence_length])
    y.append(addresses[i + sequence_length])

X = np.array(X)
y = np.array(y)

# Reshape the data for LSTM
X = X.reshape((X.shape[0], X.shape[1], 1))

# Build the LSTM model
model = Sequential()
model.add(LSTM(50, activation='relu', input_shape=(sequence_length, 1)))
model.add(Dense(1))
model.compile(optimizer='adam', loss='mse')

# Print the model summary
model.summary()

# Train the model
model.fit(X, y, epochs=20, batch_size=32, validation_split=0.2)

# Function to predict the next address
def predict_next_address(model, input_sequence):
    input_sequence = np.array(input_sequence).reshape((1, sequence_length, 1))
    predicted_address = model.predict(input_sequence, verbose=0)
    return int(predicted_address[0][0])

# Predict the next address iteratively
input_sequence = addresses[:sequence_length].tolist()

for i in range(100):  # Predict the next 100 addresses
    next_address = predict_next_address(model, input_sequence)
    print(f'Predicted next address: {hex(next_address)}')
    input_sequence.append(next_address)
    input_sequence.pop(0)