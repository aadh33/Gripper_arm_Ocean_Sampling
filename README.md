Gripper Arm for Ocean Sampling 
A precision-controlled robotic gripper arm designed for autonomous or remotely operated oceanic sampling missions.

Overview
Gripper_arm_Ocean_Sampling is a project focused on the design, development, and deployment of an underwater robotic arm system capable of collecting marine samples, such as water, sediments, or biological specimens. Developed as part of a robotics and embedded systems internship, this system supports both surface-controlled and semi-autonomous sampling in harsh underwater environments.

The primary goal is to provide a rugged and responsive mechanical gripper arm integrated with reliable embedded control logic and intuitive surface-level user interfaces to support oceanographic studies, pollution monitoring, and underwater inspections.

Applications
Marine Biology Sampling

Pollution Monitoring & Microplastics Collection

Seafloor Sediment Retrieval

Underwater Inspection & Maintenance

ROV (Remotely Operated Vehicle) Integration

🔧 System Architecture
1. Mechanical Subsystem
Waterproofed Servo or DC Motors: For articulated movement of arm joints and the gripper claw.

3-DOF Arm Mechanism: Base rotation, vertical arm movement, and gripper control.

End Effector (Gripper): Designed to hold various objects (e.g., coral, plastic, rock) securely underwater.

Corrosion-Resistant Materials: For prolonged underwater usage.

2. Embedded Electronics
Microcontroller (ESP32 / STM32 / Arduino): Executes motion control algorithms.

Motor Drivers: Controls direction and speed of motors under load.

Limit Switches / Encoders: Ensures positional feedback for safe operation.

Waterproof Connectors: Ensure signal integrity underwater.

3. Control Logic
Surface Control Interface via ROS2/WebSocket/Python GUI

Predefined Sampling Sequences or Manual Joystick Operation

Sensor-Integrated Gripping Feedback: Detects object presence before closing the gripper.

Modes of Operation-
Mode	Description
Manual Control	Surface joystick or GUI used to directly control each joint.
Predefined Motion	Run specific sampling sequences with fixed timing and positions.

Features-
✅ Durable underwater gripper for real-time sampling

✅ ROS2-based integration for ROV control

✅ Feedback loop using limit switches or encoders

✅ Modular codebase and hardware for scalability

✅ Joystick/WebSocket/manual GUI control support

✅ Real-time command execution via UART/Ethernet

✅ Safety limits to prevent overextension or damage

Repository Structure

/Gripper_arm_Ocean_Sampling
│
├── firmware/
│   ├── esp32_controller/       # Control firmware for arm movement and grip
│   ├── arduino_encoder/        # Optional feedback system
│
├── README.md

Communication Setup-
ESP32 <--> GUI: UART or WebSocket

ESP32 <--> Motor Driver: PWM signals

Setup Instructions-
Prerequisites
Python 3.8+

ESP-IDF or Arduino IDE for ESP32

Firmware Flashing-
# For ESP32 (via ESP-IDF)
idf.py build
idf.py -p /dev/ttyUSB0 flash
Running the Control Interface
bash
Copy
Edit

📊 Performance Highlights
Metric	Result
Gripper Response Time	~300 ms
Max Lifting Weight	~2.5 kg underwater
Control Latency	< 100 ms (WebSocket)
Waterproof Duration	45+ min continuous ops

🔬 Future Improvements-
Add haptic feedback for object detection

Integrate underwater pressure sensors for adaptive grip

Implement force sensors to prevent over-gripping

Deploy AI-based vision for object targeting and classification

Enable auto-stow and retract functions post sampling

🤝 Contributing
We welcome feedback, issues, and contributions! Please open an issue or submit a pull request if you'd like to collaborate.

👨‍💻 Author & Credits
Adithyan Manoj

GitHub | LinkedIn

