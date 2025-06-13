# 4WD Rover for Arduino
#### Author: Bocaletto Luca
A versatile 4-wheel drive rover with Bluetooth manual control, line-following, obstacle avoidance and tilt safety. Packed with multiple sensors to navigate complex environments.

## 1. Key Features
- 4×WD chassis with four independent DC motors  
- **Modes of Operation**:  
  - Manual via Bluetooth (HC-05)  
  - Line following with QTR-8A sensor array  
  - Obstacle avoidance using front and side HC-SR04 ultrasonic units  
- **Tilt Safety** using MPU-6050: motors shut off if tilt > 30°  
- Additional sensors: BH1750 (lux), DS18B20 (temperature)  
- Status indicators: RGB LED for mode & alerts, buzzer for alarms  
- LiPo 2S battery pack with protection & 5 V step-down regulator  

## 2. Bill of Materials
- Arduino Mega 2560  
- 2× TB6612FNG dual H-bridge drivers  
- 4× DC gear motors + wheels  
- 1× 4×4 aluminium chassis  
- HC-05 Bluetooth module  
- QTR-8A reflectance sensor array  
- 3× HC-SR04 ultrasonic sensors  
- MPU-6050 6-axis IMU  
- BH1750 ambient light sensor (I²C)  
- DS18B20 temperature probe + 4.7 kΩ pull-up resistor  
- Common-anode RGB LED + 3× 220 Ω resistors  
- Active buzzer  
- LiPo 2S battery + 5 V regulator  
- Jumpers, breadboard, cable ties, mounting hardware  

## 3. Wiring Diagram
    Arduino Mega       TB6612FNG #1        Motors A & B
    ────────────       ─────────────────    ─────────────
    5 V                VCC                 
    GND                GND                 
    D22                AIN1                M1A  
    D23                AIN2                M1B  
    D24                PWMA                
    D30                STBY                

    Arduino Mega       TB6612FNG #2        Motors C & D
    ────────────       ─────────────────    ─────────────
    D25                BIN1                M2A  
    D26                BIN2                M2B  
    D27                PWMB                
    D30                STBY                

    HC-05 Bluetooth    Arduino Mega
    ───────────────    ────────────
    VCC                5 V
    GND                GND
    TXD                D19 (RX1)
    RXD                D18 (TX1)

    QTR-8A Array       Arduino Mega
    ────────────       ────────────
    OUT0…OUT7          A0…A7
    VCC                5 V
    GND                GND
    EN                 D30 (LOW to enable)

    Ultrasonic Sensors
    ─────────────────
    Front TRIG         D2
    Front ECHO         D3
    Left TRIG          D4
    Left ECHO          D5
    Right TRIG         D6
    Right ECHO         D7

    I²C Bus (MPU-6050 & BH1750)
    ──────────────────────────
    SDA                20
    SCL                21
    VCC                5 V
    GND                GND

    DS18B20 & RGB LED & Buzzer
    ──────────────────────────
    DS18B20 DATA        D8 (4.7 kΩ pull-up to 5 V)
    RGB LED R           D9 (220 Ω)
    RGB LED G           D10 (220 Ω)
    RGB LED B           D11 (220 Ω)
    Buzzer              D12
    GND                 GND


**Note:** Use 4.7 kΩ pull-ups on I²C and DS18B20, 220 Ω resistors on LED pins. Tie EN low to enable the QTR-8A sensor array.

## 4. Operating Modes
- **Mode 0 – Standby**: white LED, motors off  
- **Mode 1 – Manual**: receive commands “F/B/L/R/S” via Bluetooth for forward/back/left/right/stop  
- **Mode 2 – Line Following**: follow black line on light background  
- **Mode 3 – Avoidance**: autonomously avoid obstacles using ultrasonic sensors  
- **Tilt Safety**: motors disabled if tilt > 30°

## 5. Software Setup
1. Install libraries in Arduino IDE:  
   - Wire, SoftwareSerial  
   - NewPing (HC-SR04)  
   - QTRSensorsRC (Pololu QTR-8A)  
   - I2Cdev, MPU6050  
   - BH1750  
   - OneWire, DallasTemperature  
2. Create project folder `Rover4WD/` containing:  
   - `README.md` (this file)  
   - `Rover4WD.ino` (the Arduino sketch)  
3. Open `Rover4WD.ino` in the IDE, select **Arduino Mega 2560**, set the right COM port, then upload.

## 6. Next Steps
- Calibrate sensor thresholds (ultrasonic, line-sensor, tilt)  
- Tune PID controller for smooth line following  
- Design a custom PCB for power distribution and sensor headers  
- Add GPS, FPV camera or LoRa communications for remote operation  

Enjoy your professional 4WD rover build!  
