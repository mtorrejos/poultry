import time
import board
import busio
import numpy as np
from PIL import Image
import adafruit_amg88xx
import RPi.GPIO as GPIO
import adafruit_dht
import BlynkLib
import os
import serial


BLYNK_AUTH = ''  #Replace with Blynk IoT token
blynk = BlynkLib.Blynk(BLYNK_AUTH, server="blynk.cloud", port=80)

arduino_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)  # Adjust port as necessary

# Initialize sensors
dht_sensor = adafruit_dht.DHT22(board.D4)
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

GPIO.setmode(GPIO.BCM)
GPIO.setup(DOUT_PIN, GPIO.IN)

previous_command = None
DEBOUNCE_DELAY = 5000
startTime = time.time() * 1000

#Pin setup
COOLING_FAN_PIN = 3  # V3
EXHAUST_FAN_PIN = 4  # V4
HEAT_LAMP_PIN = 5    # V5
WINDOW_PIN = 6       # V6
DOUT_PIN = 11        # GPIO pin for MQ137 DOUT


menu_values = {
    0: (32.2, 35.0, 60, 70, 2),
    1: (29.4, 32.2, 50, 60, 2),
    2: (26.7, 29.4, 50, 60, 2),
    3: (25, 90, 90, 50, 5),  #Manual Control
}

#Custom threshold values
custom_thresholds = {
    "temp_low": 0,
    "temp_high": 0,
    "humidity_low": 0,
    "humidity_high": 0,
    "ammonia_level": 0
}

manual_control = False  #Flag for manual control mode
custom_control_mode = False  #Flag for custom threshold mode

def send_command_to_arduino(command):
    global previous_command
    global startTime
    currentTime = time.time() * 1000
    try:
        if command != previous_command and currentTime - startTime > DEBOUNCE_DELAY:
            arduino_serial.write(bytes([command]))  #Send command as a byte
            print(f"Sent command {command} to Arduino")
            previous_command = command
            startTime = currentTime
    except Exception as e:
        print(f"Error sending command to Arduino: {e}")

#Function to control components automatically
def control_components(temp_low, temp_high, humidity_low, humidity_high, ammonia_level):
    if manual_control or custom_control_mode:
        return  #Skip if in manual or custom control mode
    
    try:
        temperature = dht_sensor.temperature
        humidity = dht_sensor.humidity

        if arduino_serial.in_waiting > 0:  #Check if data is available to read
            line = arduino_serial.readline().decode('utf-8').rstrip()  #read the line and decode it
            print(f"Received from Arduino: {line}")
        time.sleep(1)

        ammonia_detected = GPIO.input(DOUT_PIN) == GPIO.HIGH
        ammonia_ppm = ammonia_level if ammonia_detected else 0  
        print(f"Ammonia Detected: {'Yes' if ammonia_detected else 'No'} (PPM: {ammonia_ppm})")
        
        if ammonia_ppm > ammonia_level:
            print("Turning on: Exhaust fan and cooling fan, opening window")
            send_command_to_arduino(1)
        elif humidity < humidity_low and temperature < temp_low:
            print("Turning on: Heat lamp")
            send_command_to_arduino(2)
        elif humidity < humidity_low and temperature > temp_high:
            print("Opening window, turning on cooling fan")
            send_command_to_arduino(3)
        elif humidity > humidity_high and temperature > temp_high:
            print("Turning on: Exhaust fan, cooling fan, opening window")
            send_command_to_arduino(1)
        elif humidity > humidity_high and temperature < temp_low:
            print("Turning on: Exhaust fan and heat lamp")
            send_command_to_arduino(4)
        elif humidity < humidity_low:
            print("Opening window")
            send_command_to_arduino(5)
        elif humidity > humidity_high:
            print("Turning on: Exhaust fan")
            send_command_to_arduino(6)
        elif temperature < temp_low:
            print("Turning on: Heat lamp")
            send_command_to_arduino(2)
        elif temperature > temp_high:
            print("Turning on: Cooling fan")
            send_command_to_arduino(7)
        else:
            print("Turning off all components")
            send_command_to_arduino(0)

    except RuntimeError as e:
        print(f"Error reading sensors: {e}")

# Heatmap generation function
def create_heatmap():
    pixels = amg.pixels
    pixel_array = np.array(pixels)
    min_temp = np.min(pixel_array)
    max_temp = np.max(pixel_array)
    normalized_pixels = (pixel_array - min_temp) / (max_temp - min_temp) * 255  #Scale to 0-255
    
    # Create an RGB heatmap
    heatmap_color = np.zeros((8, 8, 3), dtype=np.uint8)
    for i in range(8):
        for j in range(8):
            val = normalized_pixels[i, j]
            if val < 64:
                heatmap_color[i, j] = [0, 0, int(val * 4)]  # Blue
            elif val < 128:
                heatmap_color[i, j] = [0, int((val - 64) * 4), 255]  # Cyan
            elif val < 192:
                heatmap_color[i, j] = [int((val - 128) * 4), 255, 255 - int((val - 128) * 4)]  # Yellow
            else:
                heatmap_color[i, j] = [255, 255 - int((val - 192) * 4), 0]  # Red
    
    image_data = Image.fromarray(heatmap_color)
    heatmap = image_data.resize((64, 64), Image.BICUBIC)
    heatmap.save(os.path.join('/home/pi/heatmap', "heatmap.jpg"))

#lynk virtual pin handlers for manual control

@blynk.VIRTUAL_WRITE(3)  #Cooling Fans control
def v3_write_handler(value):
    if manual_control:  # Only allow manual control if in manual mode
        if int(value[0]) != 0:
            print("V3 triggered: Cooling Fan ON")
            send_command_to_arduino(10)  # Send command to turn on Cooling Fan
        else:
            print("V3 triggered: Cooling Fan OFF")
            send_command_to_arduino(14)  # Send command to turn off all components
    else:
        print("Ignoring Cooling Fan command, not in manual control mode")

@blynk.VIRTUAL_WRITE(4)  # Exhaust Fans control
def v4_write_handler(value):
    if manual_control:  # Only allow manual control if in manual mode
        if int(value[0]) != 0:
            print("V4 triggered: Exhaust Fan ON")
            send_command_to_arduino(9)  # Send command to turn on Exhaust Fan
        else:
            print("V4 triggered: Exhaust Fan OFF")
            send_command_to_arduino(13)  # Send command to turn off all components
    else:
        print("Ignoring Exhaust Fan command, not in manual control mode")

@blynk.VIRTUAL_WRITE(5)  # Heat Lamp control
def v5_write_handler(value):
    if manual_control:  # Only allow manual control if in manual mode
        if int(value[0]) != 0:
            print("V5 triggered: Heat Lamp ON")
            send_command_to_arduino(11)  # Send command to turn on Heat Lamp
        else:
            print("V5 triggered: Heat Lamp OFF")
            send_command_to_arduino(15)  # Send command to turn off all components
    else:
        print("Ignoring Heat Lamp command, not in manual control mode")


@blynk.VIRTUAL_WRITE(6)  #Windows control
def v6_write_handler(value):
    if manual_control:  # Only allow manual control if in manual mode
        if int(value[0]) != 0:
            print("V6 triggered: Window OPEN")
            send_command_to_arduino(8)  # Send command to open Window
        else:
            print("V6 triggered: Window CLOSED")
            send_command_to_arduino(12)  # Send command to close Window
    else:
        print("Ignoring Window command, not in manual control mode")


# Function to update thresholds from Blynk menu or custom sliders
@blynk.VIRTUAL_WRITE(7)  # Dropdown menu to select preset or custom thresholds
def update_thresholds(value):
    global manual_control, custom_control_mode
    current_choice = int(value[0])  # Get the selected menu option
    if current_choice == 3:  # Manual control mode
        manual_control = True
        custom_control_mode = False
        print("Manual control mode enabled")
    elif current_choice == 4:  # Custom threshold mode
        manual_control = False
        custom_control_mode = True
        print("Custom control mode enabled")
    else:  # Preset threshold mode
        manual_control = False
        custom_control_mode = False
        preset_thresholds = menu_values[current_choice]
        temp_low, temp_high, humidity_low, humidity_high, ammonia_level = preset_thresholds
        control_components(temp_low, temp_high, humidity_low, humidity_high, ammonia_level)

#Custom threshold sliders
@blynk.VIRTUAL_WRITE(10)  # Slider for custom low temperature threshold
def v10_write_handler(value):
    if custom_control_mode:
        custom_thresholds["temp_low"] = float(value[0])
        print(f"Custom temperature low threshold set to {custom_thresholds['temp_low']}")

@blynk.VIRTUAL_WRITE(11)  # Slider for custom high temperature threshold
def v11_write_handler(value):
    if custom_control_mode:
        custom_thresholds["temp_high"] = float(value[0])
        print(f"Custom temperature high threshold set to {custom_thresholds['temp_high']}")

@blynk.VIRTUAL_WRITE(12)  # Slider for custom low humidity threshold
def v12_write_handler(value):
    if custom_control_mode:
        custom_thresholds["humidity_low"] = float(value[0])
        print(f"Custom humidity low threshold set to {custom_thresholds['humidity_low']}")

@blynk.VIRTUAL_WRITE(13)  # Slider for custom high humidity threshold
def v13_write_handler(value):
    if custom_control_mode:
        custom_thresholds["humidity_high"] = float(value[0])
        print(f"Custom humidity high threshold set to {custom_thresholds['humidity_high']}")

@blynk.VIRTUAL_WRITE(14)  # Slider for custom ammonia level threshold
def v14_write_handler(value):
    if custom_control_mode:
        custom_thresholds["ammonia_level"] = float(value[0])
        print(f"Custom ammonia level threshold set to {custom_thresholds['ammonia_level']}")

def send_sensor_data():
    try:
        temperature = dht_sensor.temperature
        humidity = dht_sensor.humidity
        if temperature is not None and humidity is not None:
            print(f"Sending to Blynk: Temperature={temperature}°C, Humidity={humidity}%")
            blynk.virtual_write(0, temperature)  # V0 for Temp
            blynk.virtual_write(1, humidity)     # V1 for Humidity
        else:
            print("Failed to get valid readings from DHT22.")
    except RuntimeError as e:
        print(f"Error reading DHT sensor: {e}")

#Main loop
try:
    while True:
        blynk.run()
        if custom_control_mode:
            control_components(custom_thresholds["temp_low"], custom_thresholds["temp_high"],
                               custom_thresholds["humidity_low"], custom_thresholds["humidity_high"],
                               custom_thresholds["ammonia_level"])
        elif not manual_control and not custom_control_mode:
            send_sensor_data()
        create_heatmap()
        time.sleep(2)
        
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    GPIO.cleanup()
    print("GPIO cleanup done")
