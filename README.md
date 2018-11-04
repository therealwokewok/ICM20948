# ICM-20948 Driver for STM32 HAL

This project outputs X, Y, and Z data for the accelerometer, magnetometer, and gyroscope on the Invensense ICM-20948 and is optimized for use with the FTI ProMotion Module.

# Porting Guide
Minimal effort is needed to port this library to another STM32 MCU.  Please place new ports in an appropriately named folder inside 'Examples' folder.  (e.g. Examples/F405RGxx).  The following peripheral settings are important to note.

  - SPI Settings
    ```
    Frame Format: Motorola
    Data Size:    8 bits
    First Bit:    MSB First
    Baud Rate:    4.5 Mbits/s
    Clk Polarity: HIGH
    Clk Phase:    2nd Edge
    CRC Calc:     Disabled
    NSS Signal:   Software
    
    DMA Requests must be enabled bidirectionally using the following settings
    Mode:         Normal
    Data Width:   Byte
    ```
  - GPIO Settings
    ```
    All GPIO's should be Output Push / Pull with low default output levels except for ICM_INT, which should be held low and triggered high.
    ```

  - UART Settings
    ```
    I have been using UART in Asynchronous mode with a baud rate of 115200 with success.  Use what you please :)  It's UART...
    ```
 
   - Files needed
     ```
     You only need to include ICM20948.h/c in your project.  Be sure to change the section labeled SPI Abstraction and I2C Abstraction to match your platform.
     ```