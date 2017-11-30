# esp-open-rtos LoRa

An [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos) library for sending and receiving data using [LoRa](https://www.lora-alliance.org/) radios. Ported from [Sandeep Mistry's](https://github.com/sandeepmistry) [Arduino-LoRa library](https://github.com/sandeepmistry/arduino-LoRa), all credit goes to him.

## Compatible Hardware

 * [Semtech SX1276/77/78/79](http://www.semtech.com/apps/product.php?pn=SX1276) based boards including:
   * [Dragino Lora Shield](http://www.dragino.com/products/module/item/102-lora-shield.html)
   * [HopeRF](http://www.hoperf.com/rf_transceiver/lora/) [RFM95W](http://www.hoperf.com/rf_transceiver/lora/RFM95W.html), [RFM96W](http://www.hoperf.com/rf_transceiver/lora/RFM96W.html), and [RFM98W](http://www.hoperf.com/rf_transceiver/lora/RFM98W.html)
   * [Modtronix](http://modtronix.com/) [inAir4](http://modtronix.com/inair4.html), [inAir9](http://modtronix.com/inair9.html), and [inAir9B](http://modtronix.com/inair9b.html)

### Semtech SX1276/77/78/79 wiring

| Semtech SX1276/77/78/79 | ESP8266 |
| :---------------------: | :------:|
| VCC | 3.3V |
| GND | GND |
| SCK | SCK |
| MISO | MISO |
| MOSI | MOSI |
| NSS | CS |
| NRESET | 16 |
| DIO0 | 5 |


`NSS`, `NRESET`, and `DIO0` pins can be changed by using `lora_set_pins(ss, reset, dio0)`. `DIO0` pin is optional, it is only needed for receive callback mode. If `DIO0` pin is used, it **must** be interrupt capable.

## API

See [api.md](api.md).

## License

This libary is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).
