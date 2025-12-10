# Auto Guitar Tuner  
[Watch Demo Video](https://www.youtube.com/watch?v=O1a539eG_tw&feature=youtu.be)

<video controls src="video/demo.mp4" title="Demo"></video>

An automated, microcontroller-driven guitar tuner that detects pitch in real time using digital signal processing and precisely adjusts tuning pegs using a servo-based mechanical interface. Built on the ESP32 platform, the tuner integrates audio sensing (piezo), autocorrelation pitch detection, a TFT-based user interface, and closed-loop servo control to achieve reliable, hands-free tuning.

---

## 🌟 Project Overview

This project implements a fully functional automatic guitar tuner capable of:

- Capturing string vibrations using a piezo sensor mounted to the tuner.
- Digitizing and analyzing audio signals through the ESP32’s ADC at ~8 kHz.
- Identifying pitch using an optimized autocorrelation algorithm.
- Computing cents deviation relative to selectable tuning presets.
- Automatically tightening or loosening tuning pegs via a servo with soft limits.

The system supports:
- Single-string tuning  
- Auto-tune-all mode (fully autonomous tuning of all six strings)

---

## 🧠 Key Features

### 🎵 Autocorrelation-Based Pitch Detection
- Robust to harmonics and noise  
- DC offset removal  
- Lag windowing matched to guitar frequency ranges  
- Subharmonic correction  
- Frequency smoothing and hold logic

### 🤖 Closed-Loop Servo Tuning
- Error-dependent variable step size  
- In-tune stability detection  
- Mechanical soft-limit protection  
- Servo-centering routine and user confirmation flow

### 🖥️ TFT Display User Interface
- Real-time frequency and cents display  
- Visual tuning meter  
- Tuning mode selection  
- String selection (manual or automatic)  
- Safety warnings and success animations  
---
## 📹 Demo
[Watch Demo Video](https://www.youtube.com/watch?v=O1a539eG_tw&feature=youtu.be)

<video controls src="video/demo.mp4" title="Demo"></video>