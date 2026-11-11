# WaveShare ESP32-S3-Touch-LCD-4.3B Driver Library

ไลบรารี Arduino / PlatformIO สำหรับบอร์ด **Waveshare ESP32-S3-Touch-LCD-4.3B**  
ออกแบบมาเพื่อรองรับการใช้งานพอร์ตสื่อสารอุตสาหกรรม **RS485**, **CAN Bus (TWAI)**, **I2C Bus** และชิปขยายขา **CH422G IO Expander** อย่างครบถ้วน สะดวก และมีประสิทธิภาพสูง

อ้างอิงเอกสารอย่างเป็นทางการ: [ESP32-S3-Touch-LCD-4.3B | WaveShare Documentation](https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3B)

---

## 📌 ตารางการเชื่อมต่อฮาร์ดแวร์ (Hardware Pinout & Mapping)

### 1. Terminal Block P1 (บล็อกเทอร์มินอลขันสายด้านหลัง/ด้านล่าง)

| ขา (Pin) | ชื่อสัญญาณ (Label) | คำอธิบายการทำงาน (Description) | พิน ESP32-S3 / วงจร |
|:---:|:---:|:---|:---|
| **1** | `VIN` | ไฟเลี้ยงบอร์ด DC กว้าง (7V – 36V) | วงจร Step-Down บนบอร์ด |
| **2** | `GND` | กราวด์ไฟเลี้ยง (Power Ground) | GND |
| **3** | `I2C_VCC` | ไฟเลี้ยงสำหรับเซนเซอร์ I2C (เลือก 3.3V หรือ 5V ผ่านจัมเปอร์) | VOUT |
| **4** | `GND` | กราวด์สัญญาณ I2C | GND |
| **5** | `D_SDA` | ข้อมูล I2C Data (ภายนอก) | **GPIO 8** (พูลอัปบนบอร์ด) |
| **6** | `D_SCL` | สัญญาณนาฬิกา I2C Clock (ภายนอก) | **GPIO 9** (พูลอัปบนบอร์ด) |
| **7** | `CANL` | สายสัญญาณ CAN Low | Transceiver `TJA1051T/3` |
| **8** | `CANH` | สายสัญญาณ CAN High | Transceiver `TJA1051T/3` |
| **9** | `RS485_A` | สัญญาณ RS485 Differential A (D+) | Transceiver `SP3485EN` |
| **10** | `RS485_B` | สัญญาณ RS485 Differential B (D-) | Transceiver `SP3485EN` |
| **11** | `DOUT0` | Isolated Digital Output 0 | ออปโต้คัปเปิลควบคุมภายนอก |
| **12** | `DOUT1` | Isolated Digital Output 1 | ออปโต้คัปเปิลควบคุมภายนอก |
| **13** | `DI_COM` | ขากลางร่วมของ Digital Input (Common) | วงจรออปโต้ขาเข้า |
| **14** | `GND` | กราวด์สัญญาณ | GND |
| **15** | `DIN0` | Isolated Digital Input 0 | ควบคุมผ่าน **CH422G EXIO0** |
| **16** | `DIN1` | Isolated Digital Input 1 | ควบคุมผ่าน **CH422G EXIO5** |

---

### 2. รายละเอียดพอร์ตสื่อสารแต่ละพอร์ต

#### 1) RS485 (Hardware Serial + Auto Direction)
* **ESP32-S3 TX:** `GPIO 44` (ส่งข้อมูล)
* **ESP32-S3 RX:** `GPIO 43` (รับข้อมูล)
* **ชิป Transceiver:** `SP3485EN`
* **การควบคุมทิศทาง (DE/RE):** **Automatic Direction Control ในระดับฮาร์ดแวร์** ไม่จำเป็นต้องสลับขา RTS ในโค้ด สามารถส่งและรับข้อมูลผ่าน Hardware Serial ได้ทันที
* **ตัวต้านทาน 120Ω:** มีสวิตช์ SW2 บนบอร์ดสำหรับเปิด/ปิด 120Ω Termination Resistor

#### 2) CAN Bus (ESP32-S3 TWAI Controller)
* **ESP32-S3 TX:** `GPIO 15`
* **ESP32-S3 RX:** `GPIO 16`
* **ชิป Transceiver:** `TJA1051T/3`
* **ความเร็ว:** รองรับตั้งแต่ 25 kbps ถึง 1 Mbps (ค่าเริ่มต้นแนะนำ 500 kbps)
* **ตัวต้านทาน 120Ω:** มีสวิตช์ SW2 บนบอร์ดสำหรับเปิด/ปิด 120Ω Termination Resistor

#### 3) I2C Bus & อุปกรณ์ที่ต่ออยู่ร่วมกัน
* **ESP32-S3 SDA:** `GPIO 8`
* **ESP32-S3 SCL:** `GPIO 9`
* **อุปกรณ์ภายในที่ใช้ I2C บัสนี้ร่วมกัน:**
  * **GT911 (Capacitive Touch):** Address `0x5D` (หรือ `0x14`)
  * **CH422G (IO Expander):** Address `0x24` (Config), `0x38` (Output), `0x26` (Input)
  * **PCF85063 (RTC):** Address `0x51` (หากติดตั้งบนบอร์ด)
  * **เซนเซอร์ภายนอก:** ต่อผ่านขั้วขันสาย `D_SDA` (พิน 5) และ `D_SCL` (พิน 6)

#### 4) CH422G IO Expander (ควบคุมหน้าจอ & พอร์ตดิจิทัล)
* **EXIO0:** `DIN0` (อ่านค่า Digital Input 0)
* **EXIO1:** `TP_RST` (รีเซ็ตทัชสกรีน GT911)
* **EXIO2:** `DISP` (เปิด/ปิดไฟหน้าจอ Backlight)
* **EXIO3:** `LCD_RST` (รีเซ็ตจอ LCD ST7701)
* **EXIO4:** `SD_CS` (Chip Select ของช่องใส่ MicroSD Card)
* **EXIO5:** `DIN1` (อ่านค่า Digital Input 1)

---

## 🚀 วิธีการใช้งาน (Quick Start)

### ตัวอย่างที่ 1: เริ่มต้นระบบทั้งหมดในบรรทัดเดียว (All-in-One Init)

```cpp
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);

    // เริ่มต้นระบบทั้งหมด:
    // RS485 ที่ 9600 baud | CAN ที่ 500 kbps | I2C ที่ 100 kHz
    // และเปิดไฟหน้าจอ Backlight อัตโนมัติ!
    WS43B.begin(9600, 500000, 100000);
}

void loop() {
    // ส่งข้อมูลออก RS485
    WS43B.rs485.println("Hello RS485");

    // ส่งเฟรม CAN Bus
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    WS43B.can.send(0x123, data, 8);

    delay(1000);
}
```

---

### ตัวอย่างที่ 2: การใช้งาน RS485 & Modbus RTU

```cpp
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    WS43B.rs485.begin(9600); // เริ่มต้น RS485 ที่ 9600 baud, 8N1
}

void loop() {
    // 1. อ่านข้อมูลทั่วไป
    if (WS43B.rs485.available()) {
        char c = (char)WS43B.rs485.read();
        Serial.print(c);
    }

    // 2. ส่งคำสั่ง Modbus RTU Read Holding Registers (Function 0x03)
    // อ่าน Slave ID 1, เริ่มต้นที่ Register 0x0000, จำนวน 2 รีจิสเตอร์
    WS43B.rs485.sendModbusReadHolding(1, 0x0000, 2);

    // 3. รับและตรวจสอบ CRC16 Modbus อัตโนมัติ
    uint8_t rxBuf[64];
    size_t rxLen = 0;
    if (WS43B.rs485.receiveModbusFrame(rxBuf, sizeof(rxBuf), rxLen, 200)) {
        Serial.printf("ได้รับ Modbus Response ถูกต้อง! ขนาดยาว %d ไบต์\n", rxLen);
    }

    delay(2000);
}
```

---

### ตัวอย่างที่ 3: การใช้งาน CAN Bus (TWAI)

```cpp
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    WS43B.can.begin(500000); // 500 kbps
}

void loop() {
    // รับเฟรม CAN
    twai_message_t msg;
    if (WS43B.can.receiveMessage(msg, 10)) {
        WaveShare4_3B_CAN::printMessage(msg, Serial);
    }

    // ส่งเฟรม CAN Standard ID 0x123
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    WS43B.can.send(0x123, payload, 4);

    delay(1000);
}
```

---

### ตัวอย่างที่ 4: สแกนและอ่านเซนเซอร์บนบัส I2C

```cpp
#include <WaveShare4_3B.h>

void setup() {
    Serial.begin(115200);
    WS43B.i2c.begin(100000); // 100 kHz

    // สแกนอุปกรณ์ I2C ทั้งหมด และพิมพ์รายงานออกมา
    WS43B.i2c.scan(Serial);
}

void loop() {
    // เช็คว่าเซนเซอร์ address 0x44 ต่ออยู่หรือไม่
    if (WS43B.i2c.isDeviceConnected(0x44)) {
        uint8_t val;
        WS43B.i2c.readRegister8(0x44, 0x00, val);
    }
    delay(5000);
}
```

---

### ตัวอย่างที่ 5: ควบคุม Backlight และอ่าน Isolated Digital Inputs

```cpp
#include <WaveShare4_3B.h>

void setup() {
    WS43B.i2c.begin();
    WS43B.io.begin(WS43B.i2c.getWire());

    // ควบคุมไฟหน้าจอ Backlight
    WS43B.io.setBacklight(true);  // เปิดไฟหน้าจอ
    // WS43B.io.setBacklight(false); // ปิดไฟหน้าจอ
}

void loop() {
    // อ่านค่า Digital Input 0 และ 1 จากบล็อกขันสาย
    bool din0 = WS43B.io.readDigitalInput(0); // P1 Pin 15
    bool din1 = WS43B.io.readDigitalInput(1); // P1 Pin 16

    Serial.printf("DIN0: %s | DIN1: %s\n", din0 ? "HIGH" : "LOW", din1 ? "HIGH" : "LOW");
    delay(500);
}
```

---

## 📂 โครงสร้างตัวอย่างโค้ด (Examples Included)

ในโฟลเดอร์ `examples/` ประกอบด้วยตัวอย่างพร้อมรันใน Arduino IDE:
1. `01_RS485_Echo_Test` : ส่ง-รับและ Echo ข้อมูลผ่าน RS485
2. `02_RS485_Modbus_Read` : รันเป็น Modbus RTU Master พร้อมคำนวณและตรวจสอบ CRC16
3. `03_CAN_Send_Receive` : ส่ง-รับเฟรม CAN (TWAI) ตรวจสอบความเร็วและแพ็กเก็ต
4. `04_I2C_Scanner_And_Sensor` : สแกนหาชิปและเซนเซอร์ที่ต่อผ่าน Terminal D_SDA/D_SCL
5. `05_IO_Expander_And_Digital_IO` : ควบคุม Backlight และอ่านค่า Opto Isolated DI
6. `06_All_In_One_Industrial_Demo` : รวมการทำงานทุกพอร์ต (RS485 + CAN + I2C + DI/Backlight) พร้อมกัน

---

## 🛠️ การตั้งค่าใน Arduino IDE / PlatformIO

### Arduino IDE
1. คัดลอกโฟลเดอร์นี้ไปไว้ที่โฟลเดอร์ `Arduino/libraries/`
2. เลือก Board: **ESP32S3 Dev Module**
3. ตั้งค่าการตั้งค่าบอร์ด:
   * **PSRAM:** "OPI PSRAM"
   * **Flash Size:** "16MB (128Mb)"
   * **USB CDC On Boot:** "Enabled" (เพื่อให้พิมพ์ Serial ออกทางช่อง Type-C ได้สะดวก)
   * **Partition Scheme:** "16M Flash (3MB APP/9.9MB FATFS)" หรือขนาดที่เหมาะสม

### PlatformIO (`platformio.ini`)
```ini
[env:esp32-s3-touch-lcd-43b]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_build.arduino.memory_type = qio_opi
board_build.flash_size = 16MB
board_build.psram_type = opi
build_flags = 
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_CDC_ON_BOOT=1
monitor_speed = 115200
```

---

## ⚙️ แผนผัง Modbus Slave ID ในระบบ (Modbus RTU Slave Mapping)

การสื่อสาร Modbus RTU ผ่านพอร์ต RS485 (Baudrate: **9600**, Parity: **Even**, Data bits: **8**, Stop bit: **1** -> `8E1`):

| Slave ID | อุปกรณ์ (Device) | หน้าที่ / ฟังก์ชันการทำงาน | พารามิเตอร์หลัก |
|:---:|:---|:---|:---|
| **1** | **Water Meter (มาตรวัดน้ำ)** | อ่านอัตราการไหล (Flow Rate) และปริมาณน้ำรวมสะสม (Total Volume) | Flow rate (m³/h), Volume (L/m³) |
| **2** | **RS485 8DI/8DO Relay Module** | โมดูลรีเลย์และอินพุตดิจิทัลภายนอก | DI1: Pressure Switch, DI2-DI8, CH1-CH8 Relay |
| **3** | **Water Level Sensor** | เซนเซอร์วัดระดับน้ำในถัง/บ่อ | ระดับน้ำ (cm / raw register 0) |
| **4** | **Inverter Pump 1 (Omron 3G3JZ)** | ควบคุมและอ่านสถานะปั๊มน้ำตัวที่ 1 | Run/Stop, Freq Set, Acc/Dec, Freq Mon, Current, Fault |
| **5** | **Inverter Pump 2 (Omron 3G3JZ)** | ควบคุมและอ่านสถานะปั๊มน้ำตัวที่ 2 | Run/Stop, Freq Set, Acc/Dec, Freq Mon, Current, Fault |
| **6** | **Pressure Sensor (Transmitter)** | เซนเซอร์วัดแรงดันน้ำท่อส่ง (0 – 6.00 bar) | แรงดันน้ำแบบ Analog Modbus (bar) |

---

## ⚡ การตั้งค่า Inverter OMRON SYSDRIVE 3G3JZ (Address & Parameters Setup)

อินเวอร์เตอร์ **OMRON SYSDRIVE 3G3JZ** ทั้งสองตัวสื่อสารกับ ESP32 ผ่านพอร์ต RS485 (โปรโตคอล Modbus RTU) โดยต้องตั้งค่าพารามิเตอร์ที่หน้าจอ Digital Operator ของอินเวอร์เตอร์ดังนี้:

### 1. การต่อสายสัญญาณ RS485 (Wiring)
* ขั้ว **RS485+ (D+)** ของอินเวอร์เตอร์ ต่อเข้ากับ **RS485_A (พิน 9)** บน Terminal P1 ของบอร์ด ESP32
* ขั้ว **RS485- (D-)** ของอินเวอร์เตอร์ ต่อเข้ากับ **RS485_B (พิน 10)** บน Terminal P1 ของบอร์ด ESP32
* ต่อขนาน (Daisy Chain) ปั๊ม 1 และปั๊ม 2 เข้าด้วยกัน และเปิดสวิตช์ความต้านทานปิดหัวท้าย (Terminator 120Ω) ที่ตัวปลายทาง

### 2. ตารางพารามิเตอร์ที่ต้องตั้งบนตัว Inverter 3G3JZ

| พารามิเตอร์ | ชื่อฟังก์ชัน (Function Name) | ค่าที่ต้องตั้ง (Pump 1) | ค่าที่ต้องตั้ง (Pump 2) | ความหมาย / คำอธิบาย |
|:---:|:---|:---:|:---:|:---|
| **`n003`** | Operation Command Selection | **`2`** | **`2`** | ให้รับคำสั่ง Run / Stop ผ่านสื่อสาร RS485 (Modbus) |
| **`n004`** | Frequency Reference Selection | **`2`** | **`2`** | ให้รับค่าความถี่ (Hz) ผ่านสื่อสาร RS485 (Modbus) |
| **`n151`** | Modbus Slave Address | **`4`** | **`5`** | **Pump 1 = Slave ID 4**, **Pump 2 = Slave ID 5** |
| **`n152`** | Communication Speed (Baud Rate) | **`2`** | **`2`** | **9600 bps** (`0`=2400, `1`=4800, `2`=9600, `3`=19200) |
| **`n153`** | Communication Parity | **`0`** | **`0`** | **Even Parity 8E1** (`0`=Even, `1`=Odd, `2`=None) |
| **`n154`** | Send Wait Time | **`10`** | **`10`** | หน่วงเวลาส่งเฟรมตอบกลับ 10 ms |
| **`n155`** | Comm. Error Detection Action | **`0`** | **`0`** | เตือนและทำงานต่อเมื่อสื่อสารขัดข้อง (`0`=Warn & Continue) |
| **`n019`** | Acceleration Time 1 | `4.0` | `4.0` | อัตราเร่งเริ่มต้น (ระบบจะอัปเดตทับอัตโนมัติตามค่า `SetAccDec`) |
| **`n020`** | Deceleration Time 1 | `4.0` | `4.0` | อัตราลดเริ่มต้น (ระบบจะอัปเดตทับอัตโนมัติตามค่า `SetAccDec`) |

> **⚠️ ข้อสำคัญ:** หลังจากเปลี่ยนพารามิเตอร์การสื่อสาร `n151` ถึง `n155` ต้อง **ปิดไฟเลี้ยงอินเวอร์เตอร์แล้วเปิดใหม่ (Power Cycle)** เพื่อให้การตั้งค่าการสื่อสารมีผล

---

### 3. ตาราง Modbus Holding Registers ของ Omron 3G3JZ ที่ระบบใช้งาน

| Register Address (Hex) | ชื่อรีจิสเตอร์ | คำสั่ง Function Code | หน่วย / สเกล | คำอธิบายรายละเอียด |
|:---:|:---|:---:|:---:|:---|
| **`0x0001`** | Operation Command | `0x06` (Write) | Bitfield | **Bit 0**: Forward Run (`1`=Run, `0`=Stop)<br>**Bit 1**: Reverse Run (`0`)<br>**Bit 3**: Fault Reset (`1`=ล้าง Alarm/Trip) |
| **`0x0002`** | Frequency Reference | `0x06` (Write) | 0.01 Hz | ความถี่เป้าหมาย เช่น `5000` = 50.00 Hz, `3850` = 38.50 Hz |
| **`0x0109`** | Acceleration Time 1 | `0x06` (Write) | 0.1 วินาที | อัตราเร่ง เช่น `40` = 4.0 วินาที, `100` = 10.0 วินาที |
| **`0x010A`** | Deceleration Time 1 | `0x06` (Write) | 0.1 วินาที | อัตราลดความเร็ว เช่น `40` = 4.0 วินาที, `100` = 10.0 วินาที |
| **`0x0020`** | Inverter Status Word | `0x03` (Read) | Bitfield | **Bit 0**: During Run (`1`=กำลังทำงาน)<br>**Bit 3**: Inverter Fault (`1`=Alarm/Trip) |
| **`0x0023`** | Output Frequency | `0x03` (Read) | 0.01 Hz | ความถี่จริงที่กำลังจ่ายออก เช่น `3850` = 38.50 Hz |
| **`0x0024`** | Output Current | `0x03` (Read) | 0.1 A | กระแสไฟฟ้าจริงของมอเตอร์ เช่น `38` = 3.8 A |
| **`0x0027`** | Fault History Record | `0x03` (Read) | Code | รหัสความผิดปกติล่าสุด (Inverter Fault Code) |

---

## 🛠️ การตั้งค่า Waveshare โมดูล8-ch โมดูลรีเลย์อิเล็กทรอนิกส์ Modbus RTU (D)
อ้างอิงเอกสารอย่างเป็นทางการ: [WaveShare Modbus RTU Relay (D) Documentation](https://www.waveshare.com/wiki/Modbus_RTU_Relay_(D))
### คำสั่งตั้งค่า `Address`
```ini
Set the device address as 0x01: `00 06 40 00 00 01 5C 1B`
Set the device address as 0x02: `00 06 40 00 00 02 1C 1A`
Set the device address as 0x03: `00 06 40 00 00 03 DD DA`
```

### คำสั่งอ่าน `Address`
```ini
For example: [Address 2 device]
Send:    00 03 40 00 00 01 90 1B
Receive: 00 03 02 00 02 04 45       //Address: 0x02
```

---

## 📡 การควบคุมและตั้งค่าระบบผ่าน MQTT (MQTT Remote Control & Monitoring Guide)

ระบบควบคุมระดับน้ำรองรับการเชื่อมต่อและสั่งการทางไกลผ่านโปรโตคอล **MQTT** ทั้งการรายงานข้อมูลแบบ Real-time (Telemetry) และการส่งคำสั่งตั้งค่าควบคุม (Commands) ผ่าน Postman, Home Assistant, Node-RED หรือ MQTT Explorer

---

### 1. โครงสร้าง Topics (MQTT Topics Architecture)

กำหนดให้ `<base>` คือ Topic หลักที่ตั้งค่าไว้ในหน้าจอเมนู Wi-Fi & MQTT (เช่น `water_level` หรือ `water_level/data`)

#### 📥 Topics สำหรับรับข้อมูล (Subscribe / Telemetry จากบอร์ด)
* `<base>/data` : ส่งข้อมูลสถานะเซนเซอร์, ระดับน้ำ, แรงดัน, สถานะ Inverter ปั๊ม 1-2, ความถี่ (Hz), กระแส (A), วาล์ว, อลาร์ม และ IO แบบ Real-time ทุก **1 วินาที**
* `<base>/settings` : ส่งข้อมูลค่าพารามิเตอร์ PID, ความเร็วปั๊ม, ค่าแรงดัน และ Thresholds ต่างๆ ทุก **1 วินาที** (หรือเมื่อมีการเปลี่ยนแปลง)
* `<base>/configs` : ส่งข้อมูลการตั้งค่าฮาร์ดแวร์ Modbus ของทุกอุปกรณ์ (Slave ID 1-6) และค่าระบบ ทุก **30 วินาที**

> **💡 คำแนะนำสำหรับการ Subscribe:**  
> ในโปรแกรม MQTT Explorer, Node-RED หรือ Home Assistant ให้ Subscribe:  
> `water_level/data`, `water_level/settings`, `water_level/configs` หรือใช้ `water_level/+`

#### 📤 Topics สำหรับส่งคำสั่งควบคุมและตั้งค่า (Publish / Commands ไปยังบอร์ด)
สามารถส่งคำสั่งในรูปแบบ JSON ไปยัง Topic ใด Topic หนึ่งดังต่อไปนี้:
* `<base>/set`
* `<base>/control`
* `<base>/command`
* `<base>/settings/set`
* `<base>/configs/set`

หรือส่งค่าเดี่ยว (Plain Text / Number) ไปยัง Subtopic เฉพาะได้โดยตรง เช่น:
* `<base>/set/pump1` (ส่ง `1`, `0`, `true`, `false`)
* `<base>/set/pump2`
* `<base>/set/mode` (ส่ง `"auto"`, `"manual"`)
* `<base>/set/speed_pump1` (ส่ง `38.5`)
* `<base>/set/speed_pump2` (ส่ง `40.0`)
* `<base>/set/pressure` (ส่ง `2.50`)
* `<base>/set/sensor_type` (ส่ง `"rs485"` หรือ `"pressure_switch"`)
* `<base>/set/acc_dec` (ส่ง `4.0`)
* `<base>/set/valve` (ส่ง `1`, `0`)
* `<base>/set/reboot` (ส่ง `1`, `true`)

---

### 2. กฎการควบคุมปั๊มและระบบความปลอดภัย (Pump Control & Safety Rules)

1. **โหมดอัตโนมัติ (Auto Mode):**
   * ควบคุมปั๊มด้วยระบบ PID ไปยัง Inverter โดยอ้างอิงแรงดันเป้าหมาย `set_pressure`
   * สวิตช์ `auto_pump1` และ `auto_pump2` ใช้กำหนดว่าอนุญาตให้ปั๊มตัวใดร่วมทำงานในระบบ Auto บ้าง
   * สวิตช์ `sensor_type_switch`:
     * `false` (`"rs485"`): ใช้ค่าแรงดันจริงจาก Pressure Sensor RS485 (Slave ID 6) ในการคำนวณ PID ความถี่ 0 – 50 Hz
     * `true` (`"pressure_switch"`): ใช้หน้าสัมผัสดิจิทัล Pressure Switch (DI1 ของโมดูล RS485 IO)
   * หากแรงดันหรือความถี่ลดลงมาถึง `sleep_freq` เป็นเวลาต่อเนื่องเกิน `sleep_delay_sec` ปั๊มจะเข้าสู่โหมด **Sleep** หยุดการทำงานเพื่อประหยัดพลังงาน
   * **Dry-Run Protection:** หากระดับน้ำในบ่อลดลงต่ำกว่า `lower_limit_cm` ปั๊ม Auto จะหยุดทำงานทันทีเพื่อป้องกันปั๊มเสียหาย
   * เมื่ออยู่ในโหมด Auto ปุ่ม/สวิตช์ Manual Pump 1, Pump 2 และ Manual Valve จะถูกปิดและล็อกไว้ (Disabled)

2. **โหมดแมนนวล (Manual Mode):**
   * ควบคุมเปิด/ปิดปั๊มอิสระผ่าน `manual_pump1` และ `manual_pump2`
   * ความถี่ของปั๊มจะหมุนตามความเร็วที่กำหนดใน `manual_speed_pump1` และ `manual_speed_pump2` (หน่วย Hz) โดยมีค่าขั้นต่ำตาม `sleep_freq` และสูงสุด 50.0 Hz
   * ควบคุมวาล์วเติมน้ำอิสระผ่าน `manual_valve`

3. **Inverter Alarm / Trip Protection:**
   * หาก Inverter ตัวใดแจ้งเตือน Trip/Alarm หลอดไฟสถานะ `ui_LedPump1` / `ui_LedPump2` บนหน้าจอจะเปลี่ยนเป็น **สีส้ม (`#F97316`)**
   * หากอยู่ในโหมด Manual และเกิด Inverter Trip ระบบจะปลดสวิตช์ปั๊มตัวนั้นเป็น OFF ทันทีเพื่อความปลอดภัย

---

### 3. ตัวอย่าง Payload คำสั่งควบคุมและการตั้งค่า (MQTT Command Payloads)

#### 3.1 การสลับโหมดการทำงาน (Mode Selection)
* **สลับเป็นโหมด Auto พร้อมเลือกให้ปั๊ม 1 และ 2 ทำงาน:**
  ```json
  {
    "mode": "auto",
    "auto_pump1": true,
    "auto_pump2": true
  }
  ```
* **สลับเป็นโหมด Manual:**
  ```json
  {
    "mode": "manual"
  }
  ```

---

#### 3.2 การสั่งเปิด / ปิดปั๊ม และวาล์วเติมน้ำ (Pumps & Valve Control)
* **สั่งเปิดปั๊ม 1 และปิดปั๊ม 2 (ในโหมด Manual):**
  ```json
  {
    "mode": "manual",
    "pump1": true,
    "pump2": false
  }
  ```
* **สั่งเปิดวาล์วเติมน้ำลงบ่อ:**
  ```json
  {
    "valve": true
  }
  ```
* **สั่งปิดวาล์วเติมน้ำ:**
  ```json
  {
    "valve": false
  }
  ```

---

#### 3.3 การตั้งค่าความเร็วปั๊มในโหมด Manual (Manual Speed Hz)
* **ตั้งความเร็วปั๊ม 1 และปั๊ม 2 แยกกัน:**
  ```json
  {
    "speed_pump1": 38.5,
    "speed_pump2": 42.0
  }
  ```
* **ตั้งความเร็วเท่ากันทั้งสองปั๊มในคำสั่งเดียว:**
  ```json
  {
    "speed": 40.0
  }
  ```

---

#### 3.4 การตั้งค่าอัตราเร่งและอัตราลดความเร็ว (Acceleration / Deceleration Time)
ส่งค่าเวลาเป็นวินาที (ระบบจะส่งไปยัง Modbus Register `0x0109` และ `0x010A` ของ Inverter ทั้งสองตัว):
```json
{
  "acc_dec": 4.0
}
```
*(หรือส่งคีย์ `"acc": 4.0` หรือ `"dec": 4.0`)*

---

#### 3.5 การสลับประเภทเซนเซอร์แรงดัน (Sensor Type Selection)
* **เลือกใช้เซนเซอร์วัดแรงดัน RS485 (Analog Pressure Transmitter):**
  ```json
  {
    "sensor_type": "rs485"
  }
  ```
  *(หรือ `{"sensor_type_switch": false}`)*

* **เลือกใช้ Pressure Switch ดิจิทัล:**
  ```json
  {
    "sensor_type": "pressure_switch"
  }
  ```
  *(หรือ `{"sensor_type_switch": true}`)*

---

#### 3.6 การตั้งค่าพารามิเตอร์ Inverter PID (Auto PID Configuration)
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

| คีย์ JSON | ชนิดข้อมูล | หน่วย | คำอธิบาย |
|---|:---:|:---:|---|
| `set_pressure` (หรือ `sp`, `pressure`) | Float | bar | ค่าแรงดันเป้าหมายในระบบ Auto PID (0.00 – 6.00 bar) |
| `pid_kp` (หรือ `kp`) | Float | - | ค่าสัมประสิทธิ์ Proportional Gain |
| `pid_ki` (หรือ `ki`) | Float | - | ค่าสัมประสิทธิ์ Integral Gain |
| `sleep_freq` | Float | Hz | ความถี่หยุดปั๊ม (Sleep) และเป็นความเร็วต่ำสุดของปั๊ม |
| `sleep_delay_sec` (หรือ `sleep_delay`) | Integer | วินาที | เวลาหน่วงเมื่อความถี่แตะ Sleep Freq ก่อนหยุดปั๊ม |
| `lower_limit_cm` (หรือ `lower_limit`) | Float | cm | ระดับน้ำขั้นต่ำในบ่อ (ต่ำกว่านี้ปั๊ม Auto จะหยุดทำงาน) |

---

#### 3.7 การตั้งค่าระดับน้ำและสัญญาณเตือน (Water Level & Alert Thresholds)
```json
{
  "valve_on_cm": 80.0,
  "tank_capacity": 200.0,
  "valve_period_sec": 300,
  "alert_min_cm": 30.0,
  "alert_max_cm": 180.0,
  "alert_period_sec": 10
}
```

| คีย์ JSON | ชนิดข้อมูล | หน่วย | คำอธิบาย |
|---|:---:|:---:|---|
| `valve_on_cm` | Float | cm | ระดับน้ำที่สั่งเปิดวาล์วเติมน้ำอัตโนมัติ |
| `tank_capacity` | Float | cm | ความลึก/ความจุสูงสุดของถังน้ำ |
| `valve_period_sec` | Float | วินาที | หน่วงเวลารอบการเปิด/ปิดวาล์ว |
| `alert_min_cm` | Float | cm | ระดับน้ำเตือนขั้นต่ำ (Low Level Alert) |
| `alert_max_cm` | Float | cm | ระดับน้ำเตือนขั้นสูง (High Level Alert) |
| `alert_period_sec` | Integer | วินาที | หน่วงเวลาก่อนส่งสัญญาณ Siren Alarm |

---

#### 3.8 การควบคุม Local IO และ RS485 Relay Channels
* **ควบคุมรีเลย์ RS485 8 ช่องโดยตรง:**
  ```json
  {
    "rs485_io": {
      "ch3": true,
      "ch4": false,
      "ch5": true
    }
  }
  ```
  *(หรือส่งแบบ `{"ch3": true}` หรือ `{"do3": true}`)*

* **ควบคุม Local Digital Output บนบอร์ด (DO0, DO1):**
  ```json
  {
    "do0": true,
    "do1": false
  }
  ```

---

#### 3.9 คำสั่งระบบ (System Commands)
* **รีบูตบอร์ด ESP32-S3:**
  ```json
  {
    "reboot": true
  }
  ```
  *(หรือ `{"restart": true}`)*

* **เปิด / ปิดไฟหน้าจอ Backlight ทันที:**
  ```json
  {
    "backlight": true
  }
  ```

* **ตั้งเวลาดับหน้าจอและรีสตาร์ตอัตโนมัติ:**
  ```json
  {
    "auto_backlight_enabled": true,
    "auto_backlight_min": 5,
    "auto_restart_enabled": true,
    "auto_restart_min": 1440
  }
  ```

---

### 4. โครงสร้างข้อมูลที่ส่งออกจากบอร์ด (Outbound Telemetry Payloads)

#### 4.1 ข้อมูล Real-time Telemetry (`<base>/data`) — ส่งทุก 1 วินาที
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
    "input": {
      "di1": false,
      "di2": false,
      "di3": false,
      "di4": false,
      "di5": false,
      "di6": false,
      "di7": false,
      "di8": false
    },
    "output": {
      "ch1": true,
      "ch2": false,
      "ch3": false,
      "ch4": false,
      "ch5": false,
      "ch6": false,
      "ch7": false,
      "ch8": false
    }
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

#### 4.2 ข้อมูลการตั้งค่าปัจจุบัน (`<base>/settings`) — ส่งทุก 1 วินาที
```json
{
  "valve_on_cm": 80.0,
  "tank_capacity": 200.0,
  "valve_period_sec": 300.0,
  "pump_delay_sec": 10,
  "sleep_delay_sec": 10,
  "lower_limit_cm": 30.0,
  "alert_min_cm": 30.0,
  "alert_max_cm": 180.0,
  "alert_period_sec": 10,
  "auto_mode": true,
  "auto_pump1": true,
  "auto_pump2": false,
  "manual_pump1": false,
  "manual_pump2": false,
  "manual_valve": false,
  "pid_kp": 2.00,
  "pid_ki": 0.50,
  "set_pressure": 2.50,
  "sleep_freq": 25.0,
  "manual_speed_pump1": 38.5,
  "manual_speed_pump2": 35.0,
  "acc_dec": 4.0,
  "sensor_type_switch": false,
  "sensor_type": "rs485"
}
```

---

#### 4.3 ข้อมูลคอนฟิกฮาร์ดแวร์ Modbus และระบบ (`<base>/configs`) — ส่งทุก 30 วินาที
```json
{
  "modbus": {
    "baud_rate": 9600,
    "parity": "Even",
    "devices": {
      "water_meter": { "slave_id": 1, "poll_ms": 1000 },
      "rs485_io": { "slave_id": 2, "poll_ms": 100 },
      "water_level": { "slave_id": 3, "poll_ms": 200 },
      "inverter_pump1": { "slave_id": 4, "poll_ms": 250 },
      "inverter_pump2": { "slave_id": 5, "poll_ms": 250 },
      "pressure_sensor": { "slave_id": 6, "poll_ms": 200 }
    }
  },
  "system": {
    "auto_backlight_enabled": false,
    "auto_backlight_min": 5,
    "auto_restart_enabled": false,
    "auto_restart_min": 1440
  }
}
```
