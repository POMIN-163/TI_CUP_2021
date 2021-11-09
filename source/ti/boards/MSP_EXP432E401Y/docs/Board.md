# SimpleLink&trade; MSP-EXP432E401Y LaunchPad&trade; Settings & Resources

The [__SimpleLink MSP-EXP432E401Y LaunchPad__][launchpad] contains a
[__MSP432E401Y__][device] device.


## Jumper Settings

* Close __`JP2/MCU 3V3`__ to provide power to the MSP432E401Y device.
* Close __`JP1:5V-XDS`__ to provide power to the MSP432E401Y device.
* Close the __`RXD<<`__, __`TXD>>`__, __`JP5:UART0`__, and __`JP4:UART0`__
to enable UART via the XDS110 on-board USB debugger for UART0 (default).
* Close __`JP5:UART2`__, and __`JP4:UART2`__ to enable UART via the XDS110
on-board USB debugger for UART2 (alternative).


## SysConfig Board File

The [MSP_EXP432E401Y.syscfg.json](../.meta/MSP_EXP432E401Y.syscfg.json)
is a handcrafted file used by SysConfig. It describes the physical pin-out
and components on the LaunchPad.


## Driver Examples Resources

Examples utilize SysConfig to generate software configurations into
the __ti_drivers_config.c__ and __ti_drivers_config.h__ files. The SysConfig
user interface can be utilized to determine pins and resources used.
Information on pins and resources used is also present in both generated files.


## TI BoosterPacks&trade;

The MSP-EXP432E401Y LaunchPad features 2 BoosterPack headers.
* The BoosterPack header located at the top of the device and
labeled __`J1`__, __`J2`__, __`J3`__, and __`J4`__ is referred to as
__BoosterPack 1 connector__.
* The BoosterPack header located at the bottom of the device and
labeled __`J5`__, __`J6`__, __`J7`__, and __`J8`__ is referred to as
__BoosterPack 2 connector__.

The following BoosterPack(s) are used with some driver examples.

#### [__BOOSTXL-SHARP128 LCD & SD Card BoosterPack__][boostxl-sharp128]
  * Examples require the BoosterPack be placed on __BoosterPack 1 connectors__.

#### [__BP-BASSENSORSMKII BoosterPack__][bp-bassensorsmkii]
  * Examples require the BoosterPack be placed on __BoosterPack 2 connectors__.



[device]: http://www.ti.com/product/MSP432E401Y
[launchpad]: http://www.ti.com/tool/MSP-EXP432E401Y
[boostxl-sharp128]: http://www.ti.com/tool/boostxl-sharp128
[bp-bassensorsmkii]: http://www.ti.com/tool/bp-bassensorsmkii