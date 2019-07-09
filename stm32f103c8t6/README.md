# Altura-MIDI-Theremin
This is a rework of the Zeppelin Design Labs Altura-MIDI-Theremin from the ATmega328 processor to the STM32F103C8T6 BluePill.

Some enhancements from the original design include:
- On-board USB to serve as MIDI interface with PC (in addition to direct MIDI-out port), eliminating the need for external MIDI adapters
- Faster 72MHz CPU with more RAM and more Flash for future expansion
- TM1637 based LED driver to reduce display flicker, add brightness control, and reduce CPU loading
- Can be powered from USB when using the PC as a synthesizer device, or use 9V battery or 9VDC adapter for standalone operation
- Several bug fixes to the code

The original source code is up one level in the `ATmega328` folder.  The reworked source is in this `stm32f103c8t6` folder.  It was developed using [PlatformIO](https://platformio.org/) using a [custom STSTM32 Platform package](https://github.com/dewhisna/platform-ststm32) and [custom Arduino STM32 Framework](https://github.com/dewhisna/Arduino_STM32).

The PCB schematic, gerbers, bill-of-materials, and 3D Printed Case STL and OpenSCAD files are in the `hardware` subfolder.

The PCBs are posted on and available through OSHPark:
- [Main PCB](https://oshpark.com/shared_projects/Z14xbxoM)
- [LED5631B_TM1637 Display Board](https://oshpark.com/shared_projects/c8xrlyW0)

Prebuilt binary files are available on the [Releases Page](https://github.com/dewhisna/Altura-MIDI-Theremin/releases).  Two STM32 binaries are posted with the second one having the left/right sensors swapped.

Note: For extended battery life, replace the 10K potentiometers in the bill-of-materials (RV1-RV8) with 100K versions (like on the original ATmega328 version).  The 100K pots work just as well, but will use 1/10th the power than the 10K will in terms of constant voltage drop.  The 10K parts were chosen only because I happened to already have an entire bag-full of them on-hand.

To program the binary file into the chip, use the `stm32flash` tool from the `stm32duino` project or from [http://github.com/rogerclarkmelbourne/arduino_stm32](http://github.com/rogerclarkmelbourne/arduino_stm32).

Flashing can be accomplished through the on-board FTDI port using a standard off-the-shelf FTDI cable (either 3.3v or 5v version).  This port is the standard FTDI configuration as used on Arduino ProMini and similar boards.
Note that it's usually not necessary to place the FTDI +3.3/+5v jumper on this board, as most FTDI adapters receive power through their USB port.  That jumper is provided as a convenience for other FTDI adapters that require external power other than USB, such as a fully isolated adapter.

To flash the firmware, plug in the FTDI cable, move the `BOOT0` jumper on the `BluePill` board to the `0` position, and press the reset-button on the `BluePill`.  Then enter the following on your PC:

```
stm32flash -g 0x08000000 -v -b 115200 -w firmware.bin /dev/ttyUSB0
```

Replace `firmware.bin` with the name of the binary firmware file being programmed.  And `/dev/ttyUSB0` with the device path of your FTDI adapter.  These instructions are for Linux.  There are similar tools for Windows and Mac, but I haven't tried them.

Note that some `BluePill` boards come with their flash in read-only mode.  If you receive a `NACK from device on command 0x43, Can't initiate chip erase` error, run this:

```
stm32flash -k /dev/ttyUSB0
```

to disable the read-only protection mode (again substituting the `/dev/ttyUSB0` for your FTDI path).

Note that the above assumes `stm32flash` is in your search path.  If not, use the path to `stm32flash` instead, such as `./stm32flash`.
Also, it assumes your user account has at least group privileges to the `/dev/ttyUSB0` device.  If not, you will either need to add yourself to that group (probably the `dialout` group) or prefix this with `sudo`.

When it's finished flashing, it will automatically boot the code and you can run and experiment with it.  But don't forget to move the `BOOT0` jumper back to the `1` position so that it will boot from code after you cycle power.

See the [documentation at the Zeppelin Design Labs website](https://zeppelindesignlabs.com/product/altura-theremin-midi-controller/) for details on how to use it.  Enjoy!

And give your support to them as the original inventors.  This remix is released as-is in full open-source form and is **not** intended to be competition to their design, but instead to complement it.

