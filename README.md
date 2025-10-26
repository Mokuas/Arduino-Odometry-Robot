# ISS_Project1_Sim (Arduino)

Differential drive robot with encoder-based odometry control.

- Board: Arduino Uno (modifiable)
- Wheel diameter: 6.5 cm  
- Resolution: 20 slots × 2 edges = 40 pulses/rev  
- Wheelbase: 9.0 cm  

## Pins
IN1=A0, IN2=A1, IN3=A2, IN4=A3  
ENA=5, ENB=6  
WHEEL_L=2, WHEEL_R=3 (interrupts)

## Examples
- `moveBy(20);` → move forward 20 cm  
- `moveBy(-20);` → move backward 20 cm  
- `rotate(180);` → rotate 180°  

## Description
This project demonstrates precise motion control using wheel encoders and basic odometry for distance and rotation tracking. Ideal for small differential robots and educational purposes.