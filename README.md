# ระบบควบคุมระดับน้ำและแรงดันน้ำอุตสาหกรรม (Industrial Water Level & Pressure Control System)
### Waveshare ESP32-S3-Touch-LCD-4.3B | Dual Inverter OMRON SYSDRIVE 3G3JZ (Modbus RTU & MQTT)

เฟิร์มแวร์ระบบควบคุมระดับน้ำและแรงดันน้ำอัตโนมัติ สำหรับบอร์ด **Waveshare ESP32-S3-Touch-LCD-4.3B** ทำงานร่วมกับอินเวอร์เตอร์ควบคุมปั๊มน้ำ **OMRON SYSDRIVE 3G3JZ** จำนวน 2 ตัว, เซนเซอร์วัดแรงดันน้ำ RS485, เซนเซอร์วัดระดับน้ำ RS485, มาตรวัดน้ำ RS485 และโมดูลรีเลย์ RS485 8DI/8DO

---

## 🏗️ สถาปัตยกรรมระบบ (System Architecture)

ระบบทำงานบน **FreeRTOS Dual-Core Architecture** เพื่อประสิทธิภาพและความเสถียรสูงสุด:
* **Core 1 (UI & Display Engine):** 
  * จัดการหน้าจอสัมผัส Capacitive Touch (GT911) และจอสี ST7701 800x480 ผ่าน LovyanGFX
  * ประมวลผลกราฟิก **LVGL 8.3 / 9.x** (60 FPS) แสดงผลหน้า Auto, Manual, System Monitor, Wi-Fi & MQTT
* **Core 0 (Dedicated Industrial Communication & Control Engine):**
  * สื่อสารพอร์ต **RS485 Modbus RTU Master** (Baudrate 9600, 8E1)
  * ประมวลผลอัลกอริทึม **PID Pressure Control** ควบคุมความถี่อินเวอร์เตอร์ 0 – 50 Hz ทุก 100ms
  * ควบคุมระบบประหยัดพลังงาน **Sleep / Wake-up Logic** อัตโนมัติ
  * ควบคุมชิปขยายขา **CH422G IO Expander**, สื่อสาร **CAN Bus (TWAI)** และเชื่อมต่อ **Wi-Fi / MQTT**

---

## 📌 ตารางการเชื่อมต่อฮาร์ดแวร์ (Hardware Pinout & Mapping)

### 1. Terminal Block P1 (บล็อกเทอร์มินอลขันสายด้านล่างของบอร์ด)

| ขา (Pin) | ชื่อสัญญาณ (Label) | คำอธิบายการทำงาน (Description) | พิน ESP32-S3 / วงจร |
|:---:|:---:|:---|:---|
| **1** | `VIN` | ไฟเลี้ยงบอร์ด DC กว้าง (7V – 36V) | วงจร Step-Down บนบอร์ด |
| **2** | `GND` | กราวด์ไฟเลี้ยงหลัก (Power Ground) | GND |
| **3** | `I2C_VCC` | ไฟเลี้ยงเซนเซอร์ I2C (เลือก 3.3V หรือ 5V ผ่านจัมเปอร์) | VOUT |
| **4** | `GND` | กราวด์สัญญาณ I2C | GND |
| **5** | `D_SDA` | ข้อมูล I2C Data ภายนอก | **GPIO 8** (พูลอัปบนบอร์ด) |
| **6** | `D_SCL` | สัญญาณนาฬิกา I2C Clock ภายนอก | **GPIO 9** (พูลอัปบนบอร์ด) |
| **7** | `CANL` | สายสัญญาณ CAN Low | ชิป Transceiver `TJA1051T/3` |
| **8** | `CANH` | สายสัญญาณ CAN High | ชิป Transceiver `TJA1051T/3` |
| **9** | `RS485_A` | สัญญาณ RS485 Differential A (D+) | ชิป Transceiver `SP3485EN` |
| **10** | `RS485_B` | สัญญาณ RS485 Differential B (D-) | ชิป Transceiver `SP3485EN` |
| **11** | `DOUT0` | Isolated Digital Output 0 | ออปโต้คัปเปิลควบคุมภายนอก |
| **12** | `DOUT1` | Isolated Digital Output 1 | ออปโต้คัปเปิลควบคุมภายนอก |
| **13** | `DI_COM` | ขากลางร่วมของ Digital Input (Common) | วงจรออปโต้ขาเข้า |
| **14** | `GND` | กราวด์สัญญาณ | GND |
| **15** | `DIN0` | Isolated Digital Input 0 | ควบคุมผ่าน **CH422G EXIO0** |
| **16** | `DIN1` | Isolated Digital Input 1 | ควบคุมผ่าน **CH422G EXIO5** |

---

## ⚙️ แผนผัง Modbus Slave ID ในระบบ (Modbus RTU Slave Mapping)

การสื่อสาร Modbus RTU ผ่านพอร์ต RS485 ตั้งค่าคอนฟิก: **Baudrate 9600 bps**, **Parity Even**, **Data 8-bit**, **Stop 1-bit** (`SERIAL_8E1`)

| Slave ID | อุปกรณ์ (Device) | หน้าที่ / ฟังก์ชันการทำงาน | รอบเวลา (Poll Interval) |
|:---:|:---|:---|:---:|
| **1** | **Water Meter (มาตรวัดน้ำ)** | อ่าน Flow Rate (m³/h) และ Total Volume (L) | 1,000 ms |
| **2** | **RS485 8DI/8DO Relay Module** | DI1: Pressure Switch, DI2: Overload P1, DI3: Overload P2, DO CH1-CH6 ควบคุมตามวงจร | 500 ms |
| **3** | **Water Level Sensor** | อ่านระดับความสูงน้ำในถัง/บ่อ (cm) | 1,000 ms |
| **4** | **Inverter Pump 1 (Omron 3G3JZ)** | ควบคุมและอ่านสถานะปั๊มน้ำตัวที่ 1 (ความถี่, กระแส, Run, Alarm) | **100 ms** |
| **5** | **Inverter Pump 2 (Omron 3G3JZ)** | ควบคุมและอ่านสถานะปั๊มน้ำตัวที่ 2 (ความถี่, กระแส, Run, Alarm) | **100 ms** |
| **6** | **Pressure Sensor (Transmitter)** | อ่านแรงดันน้ำจริงในท่อจ่ายน้ำ (0.00 – 6.00 bar) | 200 ms |

### การกำหนดช่องต่อ RS485 8DI/8DO Module (Slave ID 2)
* **Digital Outputs (Relay CH1 - CH8):**
  - **CH1 (`rs485_ch1_io_pin`):** ไฟสถานะรัน Pump 1
  - **CH2 (`rs485_ch2_io_pin`):** ไฟสถานะรัน Pump 2
  - **CH3 (`rs485_ch3_io_pin`):** ไฟ Alarm ของ Pump 1 (Inverter Alarm / Overload)
  - **CH4 (`rs485_ch4_io_pin`):** ไฟ Alarm ของ Pump 2 (Inverter Alarm / Overload)
  - **CH5 (`rs485_ch5_io_pin`):** Alarm ของระบบ (`alarm_io_pin` น้ำต่ำ/สูงเกินเวลาที่ตั้ง)
  - **CH6 (`rs485_ch6_io_pin`):** วาล์วเติมน้ำ (`valve_io_pin`)
  - **CH7 - CH8:** สำรอง (Spare)
* **Digital Inputs (DI1 - DI8):**
  - **DI1 (`rs485_di1_io_pin`):** สวิตช์แรงดัน (Pressure Switch)
  - **DI2 (`rs485_di2_io_pin`):** Overload Pump 1
  - **DI3 (`rs485_di3_io_pin`):** Overload Pump 2
  - **DI4 - DI8:** สำรอง (Spare)

---

## ⚡ การตั้งค่า Inverter OMRON SYSDRIVE 3G3JZ อย่างละเอียด (Address & Parameters Setup)

อินเวอร์เตอร์ **OMRON SYSDRIVE 3G3JZ** ทั้งสองตัวสื่อสารกับ ESP32 ผ่านพอร์ต RS485 ขนานกันบนบัสเดียว (Daisy Chain) โดยต้องตั้งค่าพารามิเตอร์ที่หน้าจอ Digital Operator ของตัวอินเวอร์เตอร์ตามตารางด้านล่างนี้:

### 1. การต่อสายสัญญาณ RS485 (Wiring)
* ขั้ว **RS485+ (D+)** ของอินเวอร์เตอร์ตัวที่ 1 และ 2 ต่อเข้ากับ **RS485_A (พิน 9)** บน Terminal P1 ของบอร์ด ESP32
* ขั้ว **RS485- (D-)** ของอินเวอร์เตอร์ตัวที่ 1 และ 2 ต่อเข้ากับ **RS485_B (พิน 10)** บน Terminal P1 ของบอร์ด ESP32
* สวิตช์ปิดหัวท้าย (Termination Resistor 120Ω): เปิดสวิตช์ SW บนตัวอินเวอร์เตอร์ตัวสุดท้ายที่ปลายสาย

---

### 2. ตารางการตั้งค่าพารามิเตอร์บน Inverter 3G3JZ

| พารามิเตอร์ | ค่าสำหรับ Pump 1 | ค่าสำหรับ Pump 2 | ความหมายและหน้าที่ (Description) |
|:---:|:---:|:---:|:---|
| **`n9.00`** | **`4`** | **`5`** | **Modbus Slave ID:** กำหนด Address ปั๊ม 1 = 4, ปั๊ม 2 = 5 |
| **`n9.01`** | **`1`** | **`1`** | **Communication Speed:** `1` = 9600 bps (`0`=4800, `1`=9600, `2`=19200, `3`=38400) |
| **`n9.02`** | **`2`** | **`2`** | **Communication Error Treatment:** `2` = Free stop (หยุดหมุนทันทีหากสื่อสารผิดพลาด) |
| **`n9.04`** | **`0`** | **`0`** | **Response Wait Time:** `0` = ส่งสัญญาณตอบกลับทันที (Fast Response) |
| **`n9.05`** | **`5.0`** | **`5.0`** | **Communication Timeout Time:** ตั้งเวลาตัด Timeout **5.0 วินาที** (ป้องกันข้อผิดพลาด `cE10` เวลาบัสสื่อสารกับ Slave ตัวอื่น) |
| **`n2.00`** | **`4`** | **`4`** | **Frequency Reference Selection:** `4` = รับคำสั่งความเร็ว (Hz) ผ่านสื่อสาร RS-485 Modbus |
| **`n2.01`** | **`3`** | **`3`** | **Operation Command Selection:** `3` = รับคำสั่ง RUN / STOP ผ่านสื่อสาร RS-485 Modbus |
| **`n1.09`** | `4.0` | `4.0` | **Acceleration Time 1 (วินาที):** อัตราเร่งเริ่มต้น (ระบบ ESP32 จะเขียนอัปเดตทับอัตโนมัติตามค่า `SetAccDec`) |
| **`n1.10`** | `4.0` | `4.0` | **Deceleration Time 1 (วินาที):** อัตราลดความเร็วเริ่มต้น (ระบบ ESP32 จะเขียนอัปเดตทับอัตโนมัติตามค่า `SetAccDec`) |

> **⚠️ ข้อสำคัญอย่างยิ่ง:**  
> หลังจากกด Save พารามิเตอร์กลุ่ม `n9` เรียบร้อยแล้ว **ต้องปิดสวิตช์ตัดไฟเลี้ยงอินเวอร์เตอร์ (Power OFF) แล้วรอจนหน้าจอดับสนิท จากนั้นเปิดเครื่องใหม่ (Power ON)** เพื่อให้อินเวอร์เตอร์โหลดค่าการสื่อสาร Modbus ชุดใหม่ไปใช้งาน

---

### 3. ทำไมต้องตั้ง `n9.05 = 5.0` (การแก้ปัญหาโค้ด `cE10`)
* **โค้ด `cE10`** บนหน้าจอ 3G3JZ คือการแจ้งเตือน **RS-485 Communication Timeout**
* หากตั้ง `n9.05` ไว้สั้นเกินไป (เช่น 0.5s หรือ 1.0s) เมื่อบอร์ด ESP32 ต้องสลับไปคุยกับ Slave ตัวอื่นบนบัสที่ยังไม่ได้เชื่อมต่อ (ทำให้เกิด Modbus Wait Timeout ประมาณ 200–500ms) อินเวอร์เตอร์จะไม่ได้รับสัญญาณและจะทริป `cE10` ขึ้นมา แล้วพอ ESP32 วนกลับมาคุย `cE10` ก็จะหายไป
* การตั้ง **`n9.05 = 5.0`** (5.0 วินาที) หรือ **`0.0`** (ปิดการตรวจจับ Timeout) จะช่วยให้อินเวอร์เตอร์ทำงานได้อย่างต่อเนื่อง 100% ไม่มีปัญหาโค้ด `cE10` กะพริบกวนใจ

---

### 4. ตาราง Modbus Holding Registers ของ Omron 3G3JZ ที่ระบบ ESP32 ใช้งาน

| Register Address | ฟังก์ชัน Modbus | สเกล / หน่วย | คำอธิบายการทำงาน |
|:---:|:---:|:---:|:---|
| **`0x2101`** | `0x06` (Write Single) | Bitfield | **Control Command:** `0x0031` = Forward Run, `0x0030` = Stop |
| **`0x2102`** | `0x06` (Write Single) | 0.01 Hz | **Frequency Command:** เช่น `3850` = 38.50 Hz, `5000` = 50.00 Hz |
| **`0x0109`** | `0x06` (Write Single) | 0.1 วินาที | **Acceleration Time 1:** เช่น `40` = 4.0 วินาที, `100` = 10.0 วินาที |
| **`0x010A`** | `0x06` (Write Single) | 0.1 วินาที | **Deceleration Time 1:** เช่น `40` = 4.0 วินาที, `100` = 10.0 วินาที |
| **`0x2120`** | `0x03` (Read Holding) | Bitfield | **Status Word:** Bit 0 = Running (1=กำลังทำงาน), Bit 3/4 = Alarm/Trip (1=แจ้งเตือนข้อผิดพลาด) |
| **`0x2121`** | `0x03` (Read Holding) | Code | **Fault Code:** รหัสข้อผิดพลาดล่าสุดของอินเวอร์เตอร์ |
| **`0x2124`** | `0x03` (Read Holding) | 0.01 Hz | **Output Frequency Monitor:** ความถี่จริงที่อินเวอร์เตอร์จ่ายออก |
| **`0x2126`** | `0x03` (Read Holding) | 0.1 A | **Output Current Monitor:** กระแสไฟฟ้าจริงที่มอเตอร์ใช้งาน |

---

## 🎛️ โหมดการทำงานและตรรกะการควบคุม (Operation Modes & Logic)

ระบบแบ่งหน้าจอและโหมดการทำงานออกเป็น 2 โหมดหลัก:

### 1. โหมดอัตโนมัติ (Auto Mode — หน้า `ui_PanelAuto`)
* **PID Pressure Control:** ESP32 คำนวณ PID ทุกๆ 100ms เพื่อส่งความถี่ (Hz) ให้ Inverter ควบคุมแรงดันในท่อให้คงที่ตามค่า `SetPressure`
* **สวิตช์เลือกปั๊ม:** `SwitchAutoPump1` และ `SwitchAutoPump2` สำหรับเลือกเปิดให้ปั๊มตัวใดตัวหนึ่งหรือทั้งสองตัวเข้าร่วมในระบบ Auto
* **สลับประเภทเซนเซอร์ (`SwitchSensorType`):**
  * **OFF (`rs485`):** ใช้ค่าแรงดันจริงจาก Pressure Sensor RS485 (Slave ID 6) ในการคำนวณ PID ความถี่ 0 – 50 Hz
  * **ON (`pressure_switch`):** ใช้สัญญาณดิจิทัลจากสวิตช์แรงดัน (DI1 ของโมดูล RS485 IO)
* **Sleep & Wake-up Logic:**
  * หากแรงดันได้ที่และความถี่ลดลงมาถึงหรือต่ำกว่า `SetSleepFreq` เป็นเวลาต่อเนื่องเกิน `SetSleepDelay` (วินาที) ปั๊มจะหยุดทำงาน (Sleep) เพื่อประหยัดพลังงาน
  * เมื่อแรงดันตก ระบบจะ Wake-up กลับมาทำงานอัตโนมัติ
  * ค่า `SetSleepFreq` จะเป็นตัวควบคุมความเร็วต่ำสุด (Minimum Frequency) ของปั๊มด้วย
* **ระบบตัดน้ำแห้ง (Dry-Run Protection):**
  * หากระดับน้ำในถัง/บ่อลดลงต่ำกว่า `SetLowerLimit` (cm) ปั๊ม Auto จะหยุดทำงานทันทีเพื่อป้องกันปั๊มเสียหาย
* **Interlock:** เมื่ออยู่ในโหมด Auto สวิตช์ Manual Pump 1, 2 และ Manual Valve จะถูกปรับเป็น OFF และล็อก (Disable) ไว้

### 2. โหมดสั่งการเอง (Manual Mode — หน้า `ui_PanelManual`)
* **สวิตช์ควบคุมอิสระ:** `SwitchManualPump1`, `SwitchManualPump2` สำหรับเปิด/ปิดปั๊มแต่ละตัว และ `SwitchValve` สำหรับเปิดวาล์วเติมน้ำลงบ่อ
* **ตั้งความเร็วอิสระ:** `SetSpeedPump1` และ `SetSpeedPump2` ตั้งค่าความถี่ได้ตั้งแต่ `SetSleepFreq` ถึง `50.0 Hz`
* **ตั้งอัตราเร่งและลดความเร็ว:** `SetAccDec` ตั้งเวลาอัตราเร่งและอัตราลดความเร็ว (วินาที) ส่งไปยัง Register `0x0109` และ `0x010A` ของ Inverter ทั้งสองตัว

### 3. ระบบแสดงสถานะและการเตือน (Display & Alarms)
* **ข้อความสถานะปั๊ม:** แสดงผลแบบเรียลไทม์ที่ `ui_LabelPumpStatus` และ `ui_LabelPumpStatus1`:
  ```text
  Pressure [ 2.45 bar ]   Pump 1  [ 38.5 Hz  3.8A ]  Pump 2  [ 38.5 Hz  3.8A ]
  ```
* **ไฟสถานะปั๊ม (`ui_LedPump1`, `ui_LedPump2`):**
  * **สีเขียว (`#22C55E`):** ปั๊มกำลังทำงานปกติ (Running)
  * **สีส้ม (`#F97316`):** อินเวอร์เตอร์ติด Alarm / Trip
  * **สีมืด (`#1E293B`):** ปั๊มหยุดทำงานปกติ (Standby / Off)
* **หน้า PanelMonitor (ไฟสถานะการเชื่อมต่อ Modbus 6 อุปกรณ์):**
  * `Rs485LedMeter` ➜ สถานะการเชื่อมต่อ Water Meter (ID 1)
  * `Rs485LedIo` ➜ สถานะการเชื่อมต่อ RS485 IO Module (ID 2)
  * `Rs485LedLevel` ➜ สถานะการเชื่อมต่อ Water Level Sensor (ID 3)
  * `Rs485LedPump1` ➜ สถานะการเชื่อมต่อ Inverter Pump 1 (ID 4)
  * `Rs485LedPump2` ➜ สถานะการเชื่อมต่อ Inverter Pump 2 (ID 5)
  * `Rs485LedPressure` ➜ สถานะการเชื่อมต่อ Pressure Sensor (ID 6)

---

## 📡 การสื่อสารและการควบคุมผ่าน MQTT (MQTT Guide & Payloads)

กำหนดให้ `<base>` คือ Topic หลักที่ตั้งค่าไว้ในหน้าจอเมนู Wi-Fi & MQTT (เช่น `water_level`)

### 1. Topics ที่ใช้งาน
* **ข้อมูลที่บอร์ดส่งออก (Outbound):**
  * `<base>/data` ➜ ส่งข้อมูลสถานะ Real-time ทุก **1 วินาที**
  * `<base>/settings` ➜ ส่งข้อมูลค่าพารามิเตอร์ปัจจุบันทุก **1 วินาที** (หรือเมื่อมีการเปลี่ยนแปลง)
  * `<base>/configs` ➜ ส่งข้อมูลคอนฟิกฮาร์ดแวร์ Modbus ทุก **30 วินาที**
* **ส่งคำสั่งเข้ามายังบอร์ด (Inbound):**
  * ส่ง JSON มาที่ `<base>/set`, `<base>/control`, `<base>/command` หรือ `<base>/settings/set`
  * ส่งค่าเดี่ยวมาที่ Subtopic เช่น `<base>/set/pump1` หรือ `<base>/set/mode`

---

### 2. ตัวอย่างข้อมูล Telemetry ที่บอร์ดส่งออก (`<base>/data`)
```json
{
  "water_level": {
    "level_cm": 125.4,
    "level_pct": 62.7,
    "volumn": 1254.000,
    "flow_rate": 15.200,
    "tank_capacity_cm": 200.0
  },
  "controls": {
    "valve_on": false,
    "pump1_on": true,
    "pump2_on": false,
    "alarm_on": false,
    "pump1_alarm": false,
    "pump2_alarm": false,
    "auto_mode": true,
    "auto_pump1": true,
    "auto_pump2": false,
    "manual_pump1": false,
    "manual_pump2": false,
    "manual_valve": false,
    "manual_speed_pump1": 38.5,
    "manual_speed_pump2": 35.0,
    "pressure_bar": 2.45,
    "set_pressure": 2.50,
    "pid_kp": 2.00,
    "pid_ki": 0.50,
    "acc_dec": 4.0,
    "sleep_freq": 25.0,
    "sleep_delay_sec": 10,
    "is_sleeping": false,
    "lower_limit_cm": 30.0,
    "sensor_type_switch": false,
    "sensor_type": "rs485",
    "pressure_switch": false,
    "pressure_sensor_ok": true,
    "pid_target_freq": 38.5
  },
  "pump1": {
    "running": true,
    "alarm": false,
    "freq": 38.5,
    "current": 3.8,
    "status_word": 1,
    "auto_enabled": true,
    "manual_enabled": false,
    "modbus_ok": true
  },
  "pump2": {
    "running": false,
    "alarm": false,
    "freq": 0.0,
    "current": 0.0,
    "status_word": 0,
    "auto_enabled": false,
    "manual_enabled": false,
    "modbus_ok": true
  },
  "local_io": {
    "di0": true,
    "di1": true,
    "do0": false,
    "do1": false
  },
  "rs485_io": {
    "status_ok": true,
    "input": { "di1": false, "di2": false, "di3": false, "di4": false, "di5": false, "di6": false, "di7": false, "di8": false },
    "output": { "ch1": true, "ch2": false, "ch3": false, "ch4": false, "ch5": false, "ch6": false, "ch7": false, "ch8": false }
  },
  "system": {
    "wifi_rssi": -45,
    "ip": "192.168.1.100",
    "sensor_comm_ok": true,
    "rs485_io_ok": true,
    "meter_comm_ok": true,
    "level_comm_ok": true,
    "pump1_comm_ok": true,
    "pump2_comm_ok": true,
    "pressure_comm_ok": true,
    "uptime_sec": 3600
  }
}
```

---

### 3. ตัวอย่างการส่งคำสั่งควบคุม (Inbound Command Payloads)

ส่ง JSON Payload ไปยัง Topic `<base>/set`:

#### 3.1 สลับโหมดและเลือกปั๊ม Auto
```json
{
  "mode": "auto",
  "auto_pump1": true,
  "auto_pump2": true
}
```

#### 3.2 สลับเป็นโหมด Manual และสั่งเปิดปั๊ม 1
```json
{
  "mode": "manual",
  "pump1": true,
  "pump2": false
}
```

#### 3.3 สั่งเปิดวาล์วเติมน้ำลงบ่อ
```json
{
  "valve": true
}
```

#### 3.4 ตั้งค่าพารามิเตอร์ Inverter PID และความเร็ว Sleep
```json
{
  "set_pressure": 2.50,
  "pid_kp": 2.00,
  "pid_ki": 0.50,
  "sleep_freq": 25.0,
  "sleep_delay_sec": 10,
  "lower_limit_cm": 30.0
}
```

#### 3.5 ตั้งความเร็วปั๊ม Manual (Hz) และอัตราเร่งลด Acc/Dec (วินาที)
```json
{
  "speed_pump1": 38.5,
  "speed_pump2": 42.0,
  "acc_dec": 4.0
}
```

#### 3.6 สลับประเภทเซนเซอร์แรงดัน
```json
// ใช้เซนเซอร์วัดแรงดัน RS485 Transmitter (ID 6)
{ "sensor_type": "rs485" }

// ใช้ Pressure Switch หน้าสัมผัสดิจิทัล (DI1)
{ "sensor_type": "pressure_switch" }
```

#### 3.7 สั่งการผ่าน Subtopic ค่าเดี่ยว (Plain Text / Number)
* `<base>/set/mode` ➜ ส่ง `"auto"` หรือ `"manual"`
* `<base>/set/pump1` ➜ ส่ง `1` หรือ `0`
* `<base>/set/pump2` ➜ ส่ง `1` หรือ `0`
* `<base>/set/valve` ➜ ส่ง `1` หรือ `0`
* `<base>/set/speed_pump1` ➜ ส่ง `38.5`
* `<base>/set/pressure` ➜ ส่ง `2.50`
* `<base>/set/acc_dec` ➜ ส่ง `4.0`
* `<base>/set/reboot` ➜ ส่ง `1` เพื่อรีสตาร์ตบอร์ด

---

## 💻 การคอมไพล์และอัปโหลด (PlatformIO Build & Upload)

```powershell
# 1. คอมไพล์โปรเจกต์
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run

# 2. อัปโหลดเฟิร์มแวร์ลงบอร์ดผ่านพอร์ต COM19
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload

# 3. ตรวจสอบ Serial Monitor (115200 bps)
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM19
```
