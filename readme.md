# Serial ADC PID Control Simulation

A ROS 2-based simulation project that mimics receiving analog ADC data via UART, parsing and publishing it, and controlling a simulated BLDC motor using a PID control algorithm. The project simulates both the microcontroller sending data and the ESC + motor response using ROS 2 nodes and mock serial devices (socat).

## Features

- Simulates a 10-bit ADC reading sent via UART in a specific format
- Parses UART input (`!ExxxxD` format) and publishes ADC values to a ROS 2 topic
- Implements a PID controller using ADC values (interpreted as RPM) to generate PWM control output
- Mocks the motor receiving the PWM values and logs simulated output

## Format Specification

ADC values are formatted as:
- Example: 999 → `!E0999D`
- Example: 1011 → `!E1011D`

## Project Structure

- `mock_arduino_serial.py` — Simulates the microcontroller sending UART ADC values
- `serial_adc_publisher.py` — Reads serial data, parses it, and publishes `/adc_values`
- `pid_controller.py` — Subscribes to `/adc_values` and outputs `/motor_pwm` based on PID
- `motor_sim.py` — Subscribes to `/motor_pwm` and simulates a motor response

## Getting Started

These instructions are for Linux/WSL2 users with ROS 2 installed.

### Prerequisites

- ROS 2 (tested on Humble/Foxy)
- Python 3.x
- pyserial
- socat (for virtual UART)

### Installation

1. Install required packages:
   ```bash
   sudo apt update
   sudo apt install socat
   pip3 install pyserial
   ```

2. Clone this repository and build the workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> serial_adc_pid
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### Setup Virtual Serial Ports

1. In a terminal, run:
   ```bash
   socat -d -d pty,raw,echo=0 pty,raw,echo=0
   ```
   
   Note the two virtual ports it gives you, e.g., `/dev/pts/7` and `/dev/pts/8`.

2. Update the following files with the correct serial port paths:
   - `mock_arduino_serial.py` should write to the first port (e.g., `/dev/pts/7`)
   - `serial_adc_publisher.py` should read from the second port (e.g., `/dev/pts/8`)

## Running the Simulation

1. **Run mock serial sender** in a terminal:
   ```bash
   python3 mock_arduino_serial.py
   ```

2. In separate terminals, run the ROS 2 nodes:
   - Serial ADC publisher:
     ```bash
     ros2 run serial_adc_pid serial_adc_publisher
     ```
   - PID controller:
     ```bash
     ros2 run serial_adc_pid pid_controller
     ```
   - Motor simulation:
     ```bash
     ros2 run serial_adc_pid motor_sim
     ```

3. **Monitor topics** (optional):
   ```bash
   # View ADC values
   ros2 topic echo /adc_values
   
   # View motor PWM output
   ros2 topic echo /motor_pwm
   ```

## Dependencies

- ROS 2 (Humble/Foxy recommended)
- Python 3.x
- pyserial
- socat

## Acknowledgments

- ROS 2 documentation
- Python serial library
- socat developers
