# Mini Sumo Robot

A PS4-controlled mini sumo robot featuring autonomous opponent detection and line avoidance capabilities. This project combines autonomous behavior with manual control, making it perfect for sumo robot competitions or hobbyist robotics.

## Demo
![Mini Sumo Robot in Action](demo.gif)

## Features
- PS4 controller support via Bluetooth
- Ultrasonic-based opponent detection (up to 400cm range)
- Dual IR line sensors for ring boundary detection
- Advanced movement patterns:
  - Search mode with alternating rotation
  - Aggressive attack mode when opponent detected
  - Quick retreat when boundary line detected
- Real-time status monitoring via Serial
- Safety features (auto-stop on controller disconnect)
- PWM motor control for precise speed adjustment

## Hardware Requirements
- ESP32 development board
- L298N motor driver module
- HC-SR04 ultrasonic sensor
- 2x IR line detection sensors
- 2x DC motors with wheels
- PS4 controller
- Power supply
- Robot chassis and mounting hardware

## Pin Configuration
| Component | Pin | Description |
|-----------|-----|-------------|
| Motor A IN1 | 32 | Direction control 1 |
| Motor A IN2 | 33 | Direction control 2 |
| Motor A Enable | 27 | Speed control (PWM) |
| Motor B IN1 | 25 | Direction control 1 |
| Motor B IN2 | 26 | Direction control 2 |
| Motor B Enable | 14 | Speed control (PWM) |
| Ultrasonic TRIG | 23 | Trigger pin |
| Ultrasonic ECHO | 22 | Echo pin |
| Right Line Sensor | 35 | Ring boundary detection |
| Left Line Sensor | 34 | Ring boundary detection |

## Setup Instructions
1. Install required libraries in Arduino IDE:
   - PS4Controller
   - ESP32 PWM library
   
2. Configure Bluetooth:
   - Update the MAC address in the code: `PS4.begin("84:30:95:78:C7:DB")`
   - Replace with your PS4 controller's MAC address
   
3. Hardware Assembly:
   - Connect motors to L298N driver
   - Wire sensors according to pin configuration
   - Ensure proper power supply connections
   
4. Upload Code:
   - Select appropriate ESP32 board in Arduino IDE
   - Upload the code to your ESP32
   
5. Initial Testing:
   - Power on the robot
   - Pair PS4 controller
   - Verify all sensors are working via Serial monitor

## Operation Guide
- Controls:
  - Press Cross button to start the robot
  - Press Square button to stop
  - Controller disconnection automatically stops the robot
  
- Autonomous Behaviors:
  - Search: Robot rotates searching for opponents
  - Attack: Moves forward at full speed when opponent detected
  - Retreat: Quick backward movement when boundary line detected
  
- LED Indicators:
  - Serial output provides real-time status information
  - Battery level monitoring for PS4 controller

## Code Structure
The code implements several key functions:
- `search()`: Implements search pattern with direction changes
- `attack()`: Handles opponent engagement
- `retreat()`: Manages boundary avoidance
- `getDistance()`: Ultrasonic sensor reading
- `isLineDetected()`: Line sensor monitoring
- Motor control functions for movement and rotation

## Safety Features
- Automatic motor stop on controller disconnect
- Minimum and maximum distance thresholds for sensor readings
- Controlled acceleration in motor functions
- Regular connection status checking
- Battery level monitoring

## Development
This project uses the ESP32's LEDC peripheral for PWM generation and features real-time serial debugging output. Feel free to adjust constants in the code to tune the robot's behavior for your specific needs.
