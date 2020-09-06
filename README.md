# ST7789_power_meter_cs5460a_display
CS5460A-based plug power meter color display

*** Be sure you know what your are doing around lethal mains-level voltages. Use at your own risk! ***

YouTube video:

https://youtu.be/XAVUyOCNxC0


## Features

- 8 values  displayed on 2 screens
- no buttons, screens are changed automatically
- power min/max with times when they were observed
- big high resolution software generated 7-segment font for numbers
- sniffing the SPI CLK and MISO from CS5460A chip to read voltage, current and power

## Notes

- Required: DigiFont and RREFont libraries (available on my GitHub)
- In case of compatibility issues use Arduino IDE 1.6.5 
- Use only ENABLE_RRE_16B = 1 in RREFont.h, other ENABLEs should be 0 to save memory

If you find it useful and want to buy me a coffee or a beer:

https://www.paypal.me/cbm80amiga
