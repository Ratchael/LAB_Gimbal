# RoboMaster EP - Auto Target & Telemetry Tracking System

ระบบควบคุม Gimbal อัตโนมัติบนหุ่นยนต์ **DJI RoboMaster EP** สำหรับตรวจจับ ติดตาม และยิงเป้าหมายสีด้วยระบบประมวลผลภาพ (Computer Vision) พร้อมระบบแสดงผลและเก็บบันทึกข้อมูล Telemetry แบบ Real-time ผ่าน GUI

---

# ภาพรวมโครงการ (Project Overview)

โปรเจกต์นี้พัฒนาขึ้นสำหรับปฏิบัติการควบคุมหุ่นยนต์ โดยการรวม **RoboMaster SDK**, **OpenCV (HSV Color Space)** และ **Tkinter GUI** เข้าด้วยกัน เพื่อให้หุ่นยนต์สามารถค้นหา ติดตาม และล็อกเป้าหมายสี (Red, Green, Blue, Dark Yellow) ในระยะ 1.0 - 1.5 เมตร พร้อมสั่งยิงอินฟราเรด (IR) หรือกระสุนเจล (Gel) ตามลำดับที่ผู้ใช้กำหนดได้อย่างแม่นยำ

---

# คุณสมบัติ (Features)

*  Multi-Color Detection (HSV Model)**: ตรวจจับสีเป้าหมายด้วยปริภูมิสี HSV ทนทานต่อสภาวะแสง พร้อมเทคนิค Morphological Filtering (Opening/Closing) เพื่อลด Noise
*  Gun Barrel Masking**: ระบบ Mask สี่เหลี่ยมบดบังบริเวณลำกล้องปืนส่วนล่างของเฟรมภาพ ป้องกันสัญญาณรบกวนสีจากโครงรถ
*  Proportional Tracking Controller**: อัลกอริทึม Bounded P-Control คำนวณ Error ดึงเป้าหมายเข้าหากึ่งกลางกล้อง ป้องกันการ Over-speed และอาการสั่นไหว
*  Lock & Cooldown Mechanism**: ยืนยันการล็อกเป้าหมายเมื่อจุดกึ่งกลางอยู่ในรัศมี <= 15 px ติดต่อกัน 3 เฟรม พร้อมระบบหน่วงเวลา Cooldown 1.8 s ป้องกันการยิงซ้ำ
*  Interactive Graphical User Interface (GUI)**:
  * เลือกโหมดการยิง (IR / Gel)
  * ตั้งค่าจำนวนเป้าหมาย (1 - 4 ป้าย) และกำหนดลำดับสีในการยิง
  * จอแสดงผลวิดีโอคู่ (Live Camera Feed & Debug HSV Mask)
  * Log เฝ้าระวังค่า HSV และองศาหมุน Gimbal
* Real-time Telemetry & CSV Logging**:
  * ฝังพล็อตกราฟ Matplotlib แสดง Time Response (Yaw/Pitch Angle) บนหน้าจอ GUI
  * บันทึกข้อมูล Telemetry ความถี่สูงลงไฟล์ CSV (`telemetry_YYYYMMDD_HHMMSS.csv`)
* Simulation Mode**: รองรับการเปิดทำงานในโหมดจำลองด้วย Webcam หากไม่พบ RoboMaster SDK หรือไม่ได้เชื่อมต่อตัวรถ

---

# ความต้องการของระบบ (Prerequisites)

* **Python**: เวอร์ชัน 3.8 - 3.10
* **Hardware**: หุ่นยนต์ DJI RoboMaster EP (เชื่อมต่อผ่าน Wi-Fi AP Mode) หรือ Webcam สำหรับทดสอบ
* **Dependencies**:

```bash
pip install opencv-python numpy matplotlib robomaster
```

> **หมายเหตุ**: `tkinter` มักจะถูกติดตั้งมาพร้อมกับ Python Standard Library 

---



# วิธีการใช้งาน (Usage Guide)

1. **เชื่อมต่อหุ่นยนต์**:
   * เปิดเครื่องหุ่นยนต์ RoboMaster EP และสลับสวิตช์เป็นโหมด **Wi-Fi AP**
   * เชื่อมต่อ Wi-Fi ของคอมพิวเตอร์เข้ากับ Wi-Fi ของ RoboMaster EP

2. **เริ่มการทำงานของโปรแกรม**:
   ```bash
   python Test6.py
   ```

3. **ขั้นตอนการใช้งานผ่าน GUI**:
   1. **เลือกโหมดการยิง**: สลับระหว่าง `ยิงอินฟราเรด (IR)` หรือ `ยิงกระสุนเจล (Gel)`
   2. **เลือกจำนวนป้าย**: กำหนดจำนวนเป้าหมายที่จะยิง (1 - 4 ป้าย)
   3. **ตั้งลำดับสี**: กำหนดสีเป้าหมายสำหรับแต่นัด (Red, Green, Blue, Dark Yellow)
   4. **เริ่มระบบ**: กดปุ่ม **`เริ่มระบบ (Start)`** Gimbal จะสั่ง `recenter()` และเริ่มการกวาดสแกนติดตามเป้าหมายโดยอัตโนมัติ
   5. **หยุดการทำงาน**: กดปุ่ม **`หยุด (Stop)`** เพื่อปิดการขับเคลื่อนและบันทึกไฟล์ CSV

# หลักการทำงานหลัก (Core Logic)

1. **HSV Color Thresholding**:
   $$\text{Mask} = \text{inRange}(\text{HSV}, \text{Lower\_Bound}, \text{Upper\_Bound})$$
2. **Proportional Tracking Control**:
   * คำนวณระยะคลาดเคลื่อน:
     $$\text{Error}_X = \text{Target}_{CX} - \text{Center}_X$$
     $$\text{Error}_Y = \text{Target}_{CY} - \text{Center}_Y$$
   * สั่งการความเร็วป้อนกลับ Gimbal:
     $$\text{Yaw\_Speed} = \text{sign}(\text{Error}_X) \times \text{clamp}(3.0, 14.0, |\text{Error}_X| \times 0.08)$$
     $$\text{Pitch\_Speed} = -\text{sign}(\text{Error}_Y) \times \text{clamp}(3.0, 10.0, |\text{Error}_Y| \times 0.08)$$
3. **Multi-threaded Blaster Execution**:
   * แยกฟังก์ชันสั่งยิงปืนด้วย `threading.Thread` ป้องกันไม่ให้วิดีโอเฟรมกระตุกขณะ Blaster ทำงาน
