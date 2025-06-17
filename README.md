# A-Computer-Based-System-for-Controlling-Home-Appliances-Via-Internet-Of-Things-IoT

## Project Idea
A general embedded system model designed to control, automate, and monitor a home, store, or institution wirelessly through the Internet of Things (IoT) technology. The system uses a set of sensors and electronic modules controlled by an Arduino microcontroller, with IoT functionality implemented using the ESP-8266 module. The integration mechanism between the Arduino and ESP-8266 is explained in detail in the project documentation.

---

## Project Features
- **IoT Remote Control & Monitoring:** Enables control and monitoring of any home or institution worldwide, bypassing physical boundaries via internet access.
- **Flexible System Design:** The hardware and software are designed to be practical and modifiable, allowing future upgrades and adaptation to different microcontrollers.
- **Cost-Effective Latest Technology:** Uses up-to-date technologies while considering costs and offering alternatives to keep the system current.
- **Efficient Firmware:** Employs periodic and interrupt-driven programming for real-time operation, minimizing CPU load (~30% RAM usage) and replacing software delays with hardware timer0 delays on the AVR microcontroller.
- **Reliable Hardware & Software References:** Design and code based on tested, modern references to ensure optimal performance.

---

## Applications
This system can be applied in homes, factories, or any environment requiring monitoring or control. With minimal code changes, it can control various devices or circuits based on the main microcontroller without affecting performance. Arduino’s relatively simple programming facilitates easy customization.

---

## Control System Description
1. **Temperature Sensor (LM-35):** Controls a DC fan with up to three speed levels based on ambient temperature.
2. **Flame Sensor:** Detects fire via infrared, triggering an alarm.
3. **CO Gas/Smoke Sensor (MQ7):** Detects smoke early and triggers a smoke alarm.
4. **SIM800L GSM/GPRS Module:** Sends SMS alerts during fire incidents.
5. **ESP-8266 V2 Module:** Provides IoT connectivity for remote control and monitoring.
6. **Numeric Keypad:** For password input to open doors; triggers alarm on multiple incorrect attempts.
7. **DC Fan:** Controlled automatically by temperature or manually via internet.
8. **Solid State Relay (SSR):** Controls high power devices and smart window states.
9. **Light Sensor (BH1750 or LDR):** Controls home lighting and window dimming based on sunlight.
10. **NEXTION Touch Display:** User interface for control and temperature display.
11. **Stepper Motor:** Automated fish feeder running every 8 hours.
12. **Bidirectional Visitor Counter:** Tracks occupancy to manage lighting for energy savings.
13. **Weight Sensor:** Detects presence on shelves and triggers indicators.
14. **Manual Override:** Allows switching between automatic and manual control.
15. **Real-Time Clock:** Schedules tasks like lighting or watering at preset times.

---

## Challenges Faced
- Missing components in the simulation software (Proteus), such as smoke sensors and GSM modules.
- Limited availability of electronic components.
- High cost of some modules.

---

## Solutions Implemented
- Added custom Proteus libraries for required components.
- Simplified simulations to demonstrate working principles.
- Divided the circuit simulation into sections to showcase subsystem operations.

---

## Development Methodology
- **Software Development:** Modular programming, system analysis and design, real-time principles for low resource use and efficient operation.
- **Hardware Design:** Circuit design and maintenance on prototype boards, ensuring quality and cost management.
- Regular testing via simulation software and real-world integration testing ensured system reliability.
- Code protection measures to prevent unauthorized copying of microcontroller code.
