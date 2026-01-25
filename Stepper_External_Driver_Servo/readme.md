############# START notes for External Driver Servo ########################

First goes of external driver with FYSTEC 5VDC level shift and Howards Shift Stick.
https://oshwlab.com/hdutton/shiftstick

MaxESP4

Using the generic driver model in config.h

Using Default SWS

[Wiring Diagram](ExternalStepperDriverConnections-2.jpg)

# ShiftStick: Professional 5V Signal Interface for OnStep

The **ShiftStick** is a high-performance signal buffer and logic-level shifter designed specifically for the **OnStep** telescope controller project. It provides a robust, reliable bridge between low-voltage microcontrollers and industrial external stepper drivers.

---

## 🚀 Product Description

### Eliminate Missed Steps and Signal Jitter
Are you experiencing erratic motor behavior or "phantom" movements in your OnStep-controlled mount? This is often caused by a logic-level mismatch. Most modern microcontrollers (ESP32, Teensy, etc.) output at **3.3V**, while industrial external drivers (DM542, TB6600, etc.) are designed for **5V** logic.

The ShiftStick solves this by using a high-speed **74HCT245** transceiver to "boost" your controller's pulses to a crisp, high-current 5V signal. This ensures your commands are received with 100% reliability, even over long cable runs.

### Two Hardware Configurations:
1.  **With Regulator (Self-Powered):** Features an onboard AMS1117-5.0. It taps into your motor power ($V_{mot}$) and regulates it down to 5V for the logic. Ideal for 12V systems and clean wiring.
2.  **Without Regulator (External Power):** Designed for systems with a central 5V buck converter. Recommended for high-voltage setups (24V+).

---

## 🛠 Technical Specifications

| Feature | Specification |
| :--- | :--- |
| **Main IC** | 74HCT245PW High-Speed Octal Bus Transceiver |
| **Logic Conversion** | 3.3V CMOS to 5V TTL |
| **Channels** | 4 Independent Channels (STEP, DIR, EN, AUX) |
| **Compatibility** | DM542, TB6600, StepperOnline, & similar external drivers |
| **Filtering** | Integrated decoupling capacitors for EMI suppression |
| **Safety** | Onboard pull-up/down resistors for stable boot-up states |

### Power Options

| Mode | With AMS1117-5.0 Regulator | Without Regulator |
| :--- | :--- | :--- |
| **Logic Power** | Self-powered via $V_{mot}$ | Requires External 5V DC |
| **Input Range** | 7V to 15V DC | N/A (Isolated) |
| **Best For** | 12V Battery / Field Use | 24V-48V Bench/Fixed Setups |

> [!CAUTION]
> **Voltage Limit:** If using the version with the regulator, do not exceed **15V** on the $V_{mot}$ line. For 24V or 48V systems, use the **Without Regulator** version and an external 5V buck converter.

---

## 🔌 Connection Guide

* **H1 Header:** Input from OnStep Controller (3.3V Logic).
* **H2 Header:** Output to External Stepper Driver (5V Logic).
* **H3/H4 Headers:** Auxiliary 5V and signal pass-throughs.

