## Project Overview

This project involves a microcontroller, a motor driver, two motors, an OLED display, an MPU-9250/6500 sensor, and a 3.7V Li-ion battery. Below is a brief explanation of how it works and the components used.

### Components:
1. **Microcontroller**: The brain of the project, controlling all other components.
2. **Motor Driver (DRV8833)**: Manages the power supplied to the motors, allowing them to run forward or backward.
3. **Motors**: Provide movement for the project, such as wheels for a robot.
4. **OLED Display**: Shows information like sensor readings or status messages.
5. **MPU-9250/6500 Sensor**: Measures orientation and motion, useful for navigation or stabilization.
6. **3.7V Li-ion Battery**: Powers the entire circuit.

### How It Works:
1. **Power Supply**: The 3.7V Li-ion battery provides power to the microcontroller and other components.
2. **Control Signals**: The microcontroller sends control signals to the motor driver, which then adjusts the power to the motors, controlling their speed and direction.
3. **Sensor Data**: The MPU-9250/6500 sensor collects data on orientation and motion, sending it to the microcontroller.
4. **Display Information**: The microcontroller processes the sensor data and displays relevant information on the OLED screen.

This setup is ideal for projects like small robots or automated systems where you need to control motors and display information.
