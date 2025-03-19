# AI-Powered Garbage Collection and Segregation Robot

This project is an **AI-powered garbage collection and segregation system** designed to automate waste management using a Raspberry Pi. The robot uses computer vision, ultrasonic sensors, servo motors, and DC motors to detect, collect, and segregate waste into categories such as **biodegradable**, **non-biodegradable**, and **recyclable** materials.

## Thesis Overview

This project was developed as part of our thesis titled:

**"AI Camera-Integrated Waste Collection and Segregation System Using Raspberry Pi"**

The system integrates **YOLOv5 object detection models** for waste classification and uses a combination of sensors and motors to navigate and perform waste segregation tasks autonomously.

## Robot Behavior

1. **Waste Detection**:
   - The robot uses a camera and YOLOv5 models to detect waste objects in its environment.
   - Detected objects are classified into categories such as **biodegradable**, **non-biodegradable**, and **recyclable**.

2. **Navigation**:
   - Ultrasonic sensors are used for obstacle detection and avoidance.
   - The robot moves forward, backward, left, or right based on the distance to obstacles.

3. **Waste Collection**:
   - Once waste is detected, the robot positions itself near the object and uses servo motors to collect it.

4. **Waste Segregation**:
   - Based on the classification, the robot segregates the waste into the appropriate bin using a servo-controlled mechanism.

5. **Storage Monitoring**:
   - Ultrasonic sensors monitor the storage levels of each bin and activate LEDs to indicate when a bin is full.

## Python Libraries and Virtual Environment Setup

The following Python libraries are required to run the system. It is recommended to install them in a virtual environment.

### Libraries

- **RPi.GPIO**: For controlling GPIO pins on the Raspberry Pi.
- **OpenCV (cv2)**: For image processing and camera handling.
- **torch (PyTorch)**: For loading and running YOLOv5 models.
- **numpy**: For numerical operations.

### Setting Up the Virtual Environment

1. Create a virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install the required libraries:
   ```bash
   pip install RPi.GPIO opencv-python torch torchvision numpy
   ```

3. Verify the installation:
   ```bash
   python -m pip list
   ```

## Hardware Setup

### Components

- **Raspberry Pi** (tested on Raspberry Pi 4)
- **Camera Module**: For real-time waste detection.
- **Ultrasonic Sensors**: For obstacle detection and storage monitoring.
- **Servo Motors**: For waste collection and segregation.
- **DC Motors**: For robot navigation.
- **LEDs**: To indicate bin storage levels.

### GPIO Pin Configuration

The GPIO pins are configured as follows:

| Component         | GPIO Pin | Description               |
|--------------------|----------|---------------------------|
| Segregator Servo  | 17       | Controls waste segregation |
| Arm Servo (Left)  | 2        | Controls left arm          |
| Arm Servo (Right) | 3        | Controls right arm         |
| Collector Servo L | 14       | Controls left collector    |
| Collector Servo R | 15       | Controls right collector   |
| Obstacle Sensor   | 22, 27   | Ultrasonic sensor for navigation |
| Red Bin Sensor    | 5, 6     | Ultrasonic sensor for red bin |
| Green Bin Sensor  | 0, 1     | Ultrasonic sensor for green bin |
| Blue Bin Sensor   | 9, 10    | Ultrasonic sensor for blue bin |
| Red LED           | 7        | Indicates red bin full     |
| Green LED         | 8        | Indicates green bin full   |
| Blue LED          | 21       | Indicates blue bin full    |
| DC Motors         | 23, 24, 25, 26 | Controls robot movement |

## Running the System

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/your-repo-name.git
   cd your-repo-name
   ```

2. Activate the virtual environment:
   ```bash
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Run the main script:
   ```bash
   python3 main.py
   ```

4. Place the YOLOv5 models (`A.pt` and `B.pt`) in the appropriate directory and update the paths in the code:
   ```python
   modelPathA = "/path/to/A.pt"
   modelPathB = "/path/to/B.pt"
   ```

## Project Contributors

- **Karl Russell Dino**
- **Carl Mathew Estorga**

## Acknowledgments

This project was developed as part of our thesis and would not have been possible without the guidance of our mentors and the support of our institution.
