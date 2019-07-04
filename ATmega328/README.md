# Altura-MIDI-Theremin
Source code and binaries for the clever controller that emulates a theremin

To get the latest release onto your altura
1. Obtain and setup avrdude. (See detailed instructions in the Macchiato README.)
2. Download Altura_[version number].ino.hex. Always use the latest release!
3. Move Altura_[version number].ino.hex into the same folder as avrdude.exe and avrdude.conf
4. Connect your usbtiny programmer to your computer: just plug it in to any available USB port. Your computer should recognize the programmer as a new USB device. If it does not, you may need to install the driver, called libusb. See details in the Macchiato README.
5. Plug the usbTiny programmer into the Altura ISP header, a cluster of six pins near the right side of the circuit board. When oriented correctly, the Altura's LED and display will light. 
6. Open up a command prompt and navigate to the folder where you have avrdude.exe.
7. Enter this command: "avrdude -c usbtiny -p m328p -B 1 -V -U flash:w:Altura_[version number].ino.hex" If you copy-and-paste this line, be careful to edit the file name to exactly match the name of your hex file!

To make custom edits to the source code.
1. Obtain and setup the latest version of the Arduino IDE.
2. Obtain the arduino MIDI library from https://github.com/FortySevenEffects.
3. Download Altura_[version number].ino and open it with Arduino (Arduino should create a folder for it and move it inside)
4. Make your edits.
5. When you're done click verify and wait for it to finish.
6. Altura_[version number].ino.hex should be created somewhere in your appdata folder. (If you're having trouble finding it make sure you are showing hidden folders in folder options.)
7. Follow the instructions for "how to get the latest release", above, from step 3.
