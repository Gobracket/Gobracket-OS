# T12 clone reverse engineering and replacement firmware

## Replacement Firmware

Development WIP in `./test_fw`.

## Board Info

!(./layout.jpg)

Label on case: T12

PCB:

```
PEN-SOLDER V3 2023-02-02
```

MCU: CHIPSEA F030F6P6
https://www.st.com/resource/en/datasheet/stm32f030f4.pdf
https://www.st.com/resource/en/reference_manual/rm0360-stm32f030x4x6x8xc-and-stm32f070x6xb-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

FET: NCE30P30K
https://datasheet.lcsc.com/szlcsc/NCE30P30K_C130106.pdf

FET Driver: 72K
Maybe https://pdf1.alldatasheet.net/datasheet-pdf/view-marking/1085166/RUICHIPS/2N7002K.html

OpAmp: 8331CGn04
https://datasheet.lcsc.com/lcsc/2206101816_Gainsil-GS8331-TR_C157712.pdf

Zener: W3
BZT52C3V3? https://pdf1.alldatasheet.net/datasheet-pdf/view-marking/58974/DIODES/BZT52C3V3.html

SMPS: XL1509-3.3E1
https://www.xlsemi.com/datasheet/XL1509-EN.pdf

USB: CH224K
https://datasheet.lcsc.com/lcsc/2204251615_WCH-Jiangsu-Qin-Heng-CH224K_C970725.pdf
https://www.laskakit.cz/user/related_files/ch224ds1.pdf
https://components101.com/ics/ch224k-usb-power-delivery-controller-ic

```console
$ st-info --probe 
Found 1 stlink programmers
  version:    V2J34S7
  serial:     ...
  flash:      32768 (pagesize: 1024)
  sram:       8192
  chipid:     0x0440
  descr:      F0xx
```

## Firmware Dump

```
st-flash --hot-plug read t12-2.bin 0x8000000 32768
```

## "STM"32F030 Pinout

* 1: BOOT0 -> 10k to GND
* 4: NRST -> filtered VDD
* 6: PA0 -> not connected here, but optionally a non-populated part could connect it to some kind of SMD part which might be a tilt sensor? (only guessing)
* 7: PA1 -> ADC: VIN measurement of 1/11*VBUS
* 8: PA2 -> ADC: Temperature measurement of soldering tip's thermocouple (through amplifier)
* 9: PA3 -> Heating enable for soldering tip
* 10: PA4 -> Button "Minus (-)", high active (10k pull-down to GND, button pulls to VDD)
* 11: PA5 -> Button "Plus (+)", high active
* 12: PA6 -> I2C SCL to display (4k7 to VDD)
* 13: PA7 -> I2C SDA to display (4k7 to VDD)
* 14: PB1 -> Button "Set", high active
* 17: PA9 -> unknown, used in firmware for something
* 19: PA13/SWDIO -> SWD header/pads
* 20: PA14/SWCLK -> SWD header/pads

## CH224K USB-PD Control

CFG1,2,3 pins are not connected to anything. Floating CFG1 means it will request VBUS=20V from USB power supply.

Too bad those pins aren't connected to the uC.

## OpAmp

* Used as non-inverting amplifier, 221x gain
* Output connected to PA2 through passive low-pass (I wonder how well it works with the long line between R and C)
* Input connected to T12 soldering tip pin A (the last ring), (tip pin B (the ring between A and Earth) is connected to GND)
* Zener diode clamps input to 3.3V max.
* Input range should be 14.9 mV max. assuming the output is 3.3V max, also assuming the OpAmp is rail-to-rail
* I wonder how noisy the temperature measurement is


