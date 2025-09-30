# Week 3 Lab Notebook Entry

**Team Members:** Daniel, Ritvik, Tim  
**Date:** [Insert date here]

---

## Objectives
- Finalize component list and order critical parts.  
- Begin testing subsystems individually:  
  - Daniel: ESP32, servo motor, microphone testing.  
  - Ritvik: Power subsystem design with buck converter and 9V input.  
  - Tim: PCB layout design exploration.  

---

## Progress and Work Done

### Parts and Ordering
We have researched, selected, and ordered some of the main components.  
- Servo motor (HS-318) ordered through E-Shop.  
- LCD display ordered via Amazon.  
- Microphone (MAX9814) identified as candidate for audio subsystem (to test pitch detection).  
- Other parts identified but not yet ordered: rotary encoder, push button, Li-ion battery, buck converter.  

### Subsystem Status
- **Audio (Daniel):** Preparing to test microphone responses with ESP32 ADC to verify usable signal quality. Considering both off-the-shelf module (MAX9814) and custom preamp circuit with electret mic if required.  
- **Motor (Daniel):** Preparing demo with ESP32 controlling HS-318 servo motor using PWM.  
- **Power (Ritvik):** Designed subsystem using 9V battery with AP62150 buck converter to generate regulated 3.3 V and 5 V rails for ESP32, LCD, and servo motor.  
- **PCB (Tim):** Beginning schematic capture and PCB design, ensuring footprints match ordered parts.  

---

## Parts List (Work in Progress)

| Component Name | Part Number   | Available in E-Shop | Subsystem | Link | Ordered? | Operating Voltage | Supply Current | Features | Price | Comments |
|----------------|--------------|---------------------|-----------|------|----------|------------------|----------------|----------|-------|----------|
| Microphone     | MAX9814      | No                  | Audio     | —    | Yes      | 2.7–5.5 V        | 3.1–6 mA       | AGC Mic Preamp | $13 | Testing pitch detection |
| Rotary Encoder | KY-040       | No                  | UI        | [Amazon](https://www.amazon.com/JTAREA-KY-040-Encoder-Encoders-Modules/dp/B0D2TW63G1/) | No | 5 V | — | Push button input | — | Ask Gregg |
| Li-ion Battery | —            | No                  | Power     | [Amazon](https://www.amazon.com/Battery-Packs-Lithium-Polymer-1200mAh/dp/B00J2QET64) | No | 3.7–4.2 V | 1200 mAh | JST-PH cable | — | Rechargeable |
| Servo Motor    | HS-318       | Yes                 | Motor     | —    | No       | 4.8–6.0 V | 8–180 mA | RC servo | — | For demo |
| LCD Display    | —            | No                  | UI        | [Amazon](https://www.amazon.com/dp/B01CZL6QIQ) | Yes | 3.3–5 V | — | 16x2 LCD | — | Ordered |
| Push Button    | EG1930-ND    | No                  | UI        | [DigiKey](https://www.digikey.com/en/products/detail/e-switch/RP3502ARED/280448) | No | — | — | 3A @ 120 VAC | — | — |
| Buck Converter | AP62150WU-7  | No                  | Power     | [DigiKey](https://www.digikey.com/en/products/detail/diodes-incorporated/AP62150WU-7) | No | 4.2–18 V in, 0.8–7 V out | 1.5 A | Low IQ synchronous | — | Main regulator |

---

## Testing Notes
- Servo motor and ESP32 PWM will be demonstrated by Daniel.  
- Microphone response testing is ongoing. Need to confirm usable ADC signal and dynamic range.  
- Power system design reviewed. Need to acquire buck converter and test efficiency with 9V battery input.  

---

## Next Steps
- Finish ordering pending components.  
- Integrate microphone preamp circuit with ESP32 for pitch detection testing.  
- Implement demo: ESP32 controlling servo motor and reading mic input simultaneously.  
- Continue schematic/PCB design to consolidate all subsystems.  

---


