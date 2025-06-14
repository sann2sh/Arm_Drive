# 🤖 ARM Drive

Powered by Solder fumes, caffeine shots, and hackathon panic.

**ARM Drive** is a four-wheel robot equipped with a 5-DOF robotic arm with inverse kinematics. It is wirelessly controlled by a custom-built 2.4Ghz radio controller featuring joysticks and gesture controls powered by the MPU6050 IMU.

> ⚡ Built in under 24 hours from scratch during the **24-Hour KU Hackathon**! 



---

![ARM Drive Overview](images/1.jpg)
*Sketchy but functional – built from scratch overnight.*

---

## 🚀 Features

- **5-DOF Robotic Arm** with inverse kinematics for precise movement
- **4-Wheel Drive Base** for mobility
- **Custom Wireless Controller** with:
  - Dual Joysticks for drive + arm control
  - **MPU6050 IMU** for intuitive gesture-based control
- **Wireless Communication** between controller and robot using NRF24LO1 radio modules


---

## 🛠️ Tech Stack / Components

- **Microcontrollers**: Arduino nano
- **Motor Driver**: L298N
- **IMU**: MPU6050 (Accelerometer + Gyro)
- **Communication**: NRF24L01 2.4GHz modules
- **Inverse Kinematics**: IK logic for arm

---

## 🕹️ Controller

![Custom Controller](images/4.jpg)

- Dual-axis joysticks for movement & arm control  
- Tilt-based gestures via IMU for intuitive vehicle movement

---

## 🔧 How It Works

1. **Gamepad** reads joystick + IMU input
2. Sends data wirelessly via NRF24L01
3. **Robot** receives commands, calculates arm end point (using inverse kinematics), and drives motors accordingly

---

## 📷 More Images

| Robot Arm Close-up | Wide shot |
|--------------------|------------------|
| ![Arm](images/2.jpg) | ![Photo](images/3.jpg) |

---


## 🤝 Acknowledgements

Special thanks to the organizing team of **24 Hour KU Hackathon** for the platform and opportunity.

---


> "Ignore the sketchy build."

## 🎞️ Build Montage

> A behind-the-scenes look at the 24-hour madness! 🛠️💡 Click on the image.

<a href="https://www.youtube.com/shorts/N1ZkKimU3vc" target="_blank">
  <img src="https://i.ytimg.com/vi/N1ZkKimU3vc/oardefault.jpg" width="600"/>
</a>


---
