# QTR-MD-6A
from pyb import Pin, ADC

sensor_pins = [
    Pin(Pin.cpu.A4),
    Pin(Pin.cpu.C4), 
    Pin(Pin.cpu.A6),
    Pin(Pin.cpu.B0),
    Pin(Pin.cpu.C5),
    Pin(Pin.cpu.B1)
]

def read_and_normalize_data():
    '''!@brief Reads and normalizes sensor values.
        @return Normalized sensor values (0 to 1 range).
    '''
    raw_values = []
    for pin in sensor_pins:
        adc = ADC(pin)
        raw_values.append(adc.read())

    max_val = max(raw_values)
    min_val = min(raw_values)

    if max_val == min_val:
        normalized_data = [0.5 for _ in raw_values]  # Neutral value if all readings are identical
    else:
        normalized_data = [(reading - min_val) / (max_val - min_val) for reading in raw_values]

    return normalized_data

def apply_threshold(values, threshold):
    '''!@brief Converts sensor readings to binary values.
        @param values Normalized sensor values.
        @param threshold Threshold for detecting the line.
        @return Binary array where 1 indicates black (line), and 0 indicates white (background).
    '''
    binary_data = [1 if val > threshold else 0 for val in values]  # Inverted logic
    return binary_data

def compute_centroid(values):
    '''!@brief Computes the centroid of the binary sensor values.
        @param values Binary array of sensor values.
        @return The centroid position or None if no line is detected.
    '''
    # Define sensor positions based on physical spacing
    sensor_spacing = 0.315  # inches
    num_sensors = len(values)
    sensor_positions = [(i - (num_sensors - 1) / 2) * sensor_spacing for i in range(num_sensors)]
    
    sum_product = 0
    sum_values = 0

    # Compute weighted average
    for i, value in enumerate(values):
        sum_product += sensor_positions[i] * value
        sum_values += value

    if sum_values == 0:
        return None  # No line detected

    # Return the centroid position
    return sum_product / sum_values