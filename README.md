# si5351onRP2350

A high performance, 3 channel programmable RF VFO frequency generator built for the **Raspberry Pi RP2350** microcontroller, **Si5351** clock generator, and **SSD1306** OLED display.

***

## Features

* **60 FPS Fluid UI:** Non blocking 16 ms rendering loop prevents display draw lag and avoids dropping rotary encoder pulses.
* **Dynamic Range Tuning:** Fixed point 64 bit math tracking from 8 kHz up to 160 MHz in steps down to 0.1 Hz.
* **Independent Channel Memory:** 3 active configurations stored in temporary registers with manual RAM save overrides.

***

## Hardware Connections

* **I2C SDA:** GP16 (Pico 2 Pin 21) | Shared I2C Data Line (SSD1306 & Si5351)
* **I2C SCL:** GP17 (Pico 2 Pin 22) | Shared I2C Clock Line (SSD1306 & Si5351)
* **Page Button:** GP18 (Pico 2 Pin 24) | Cycle through active channels (0 to 2)
* **Step Button:** GP19 (Pico 2 Pin 25) | Change tuning step size (0.1 Hz to 160 MHz)
* **Save Button:** GP20 (Pico 2 Pin 26) | Commit working frequency to channel RAM
* **Encoder Phase A:** GP21 (Pico 2 Pin 27) | Rotary encoder quadrature phase A or S1
* **Encoder Phase B:** GP22 (Pico 2 Pin 29) | Rotary encoder quadrature phase B or S2

***

## Quick Flashing via `.uf2` 

1. Hold down the **BOOTSEL** button on your RP2350 board while connecting it to your computer via USB.
2. Open the mounted mass storage drive named `RPI RP2`.
3. Drag and drop **`si5351onRP2350.uf2`** into the `RPI RP2` drive.
4. The microcontroller will reboot automatically and launch the firmware.

***

## Compiling from Source (`.ino`)

1. Open **`si5351onRP2350.ino`** in the Arduino IDE.
2. Install the required libraries:
   * **Wire** (built in)
   * **Etherkit Si5351** library
3. Select your board target: **Raspberry Pi Pico 2 / Generic RP2350**.
4. Set hardware I2C pins appropriately and compile/upload.
