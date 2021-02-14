# lora-button-example
Example sketch for the ["Lora Button" board by Iotdevices](https://www.tindie.com/products/iotdevices/lora-buttoninterrupt-node-arduino-compatible/).

This is essentially adapted from my [Mini Lora example](https://github.com/crox-net/mini-lora-examples) code, and from the documentation available with the Lora Button board.

### Enclosure, battery

An enclosure for the board, as well as some details about the hardware used (battery, screws) are available here: [Enclosure for LoRa Button](https://www.thingiverse.com/thing:4762075).

![board in box](/images/box_open-gh.jpg)

### Programming

To flash the Arduino bootloader, I used my [nano-as-isp PCB](https://github.com/crox-net/arduino-nano-as-isp) with the pogo pins ICSP adapter. Note that I had to add a 10 uF capacitor between the RST and GND pins of the Nano used for flashing (see details on the linked page).

To program the board I did not solder headers either, but used a 3.3V FTDI to USB adapter along with a [clamp with pogo pins](https://s.click.aliexpress.com/e/_9GOm8s).

![programming](/images/prog.jpg)

### Voltage Divider

A feature I appreciate with the board is the availabiliy of a voltage divider to read the battery level. I've configured the jumpers as follows:

![jumpers](/images/jumpers.jpg)

This means:
* the onboard LDO is enabled (SJ1) - as I power the board with a 3.7V LiPo battery
* the voltage divider measures the RAW input (SJ2) - same reason
* the voltage divider is controlled by PD7 (SJ3) - this probably saves power, as the VD is disabled between readings by setting the pin as input

The initial tests I ran seem to indicate that by following the instructions in the documentation, one can achieve a fairly good accuracy (down to +- 0.01V).

### Additional Libraries

You will need the following additional libraries:

* [arduino-lmic](https://github.com/matthijskooijman/arduino-lmic) (see also the details [here](https://github.com/crox-net/mini-lora-examples) / [my fork](https://github.com/crox-net/arduino-lmic))
* [rocketscream/Low-Power](https://github.com/rocketscream/Low-Power)
* [NicoHood/PinChangeInterrupt](https://github.com/NicoHood/PinChangeInterrupt)
* [ElectronicCats/CayenneLPP](https://github.com/ElectronicCats/CayenneLPP)


