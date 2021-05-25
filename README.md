# NeoPill
a Blue Pill Neopixel Emulator, firmware for STM32F103C8T6.

To build with STM32CubeMX (6.1.1), open bluepill_neoemu_clk.ioc, generate code in a selected folder. 
Locate and copy over the 3 files main.c, usbd_cdc_if.c, usbd_cdc_if.h .
Build with selected environment (originally used Keil uVision 5.26.)
Flash into a Blue Pill.
Follow instructions at https://hackaday.io/project/179916-neopill

Or use STM32CubeProgrammer v2.6.0 to flash the Hex file.


Python code for the PC side is 'stripsym.py', and two YAML config files ledstrip.yaml, ledmatrix.yaml.

Command Line:
  python stripsim.py ledstrip.yaml
or
  python stripsim.py ledmatrix.yaml
