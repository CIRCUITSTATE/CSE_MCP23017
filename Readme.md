
![CSE_MCP23017](https://socialify.git.ci/CIRCUITSTATE/CSE_MCP23017/image?description=1&font=KoHo&forks=1&issues=1&logo=https%3A%2F%2Fwww.circuitstate.com%2Fwp-content%2Fuploads%2F2024%2F05%2FCIRCUITSTATE-R-Emblem-20052024-2.svg&name=1&pattern=Circuit%20Board&pulls=1&stargazers=1&theme=Auto)

# CSE_MCP23017

**CSE_MCP23017** is an Arduino library from **CIRCUITSTATE Electronics** to interface the Microchip **MCP23017** 16-bit I/O expander IC. The library supports all features of MCP23017 including GPIO operations, and interrupts. All functions of the chip can be accessed via Arduino-style APIs. The library can run on any Arduino-supported microcontroller with an I2C interface.

This library is in pre-release stage. Any feedback is highly appreciated.

# Installation

This library is available from the official **Arduino Library Manager**. Open the Arduino IDE, search for `CSE_MCP23017` and install the latest version of the library.

Additionally, you can download the latest release package from the GitHub repository and install it manually. To do so, open the Arduino IDE, go to `Sketch > Include Library > Add .ZIP Library…`​ and select the downloaded file.

Another method is to clone the GitHub repository directly into your libraries folder. The development branch will have the latest features, bug fixes and other changes. To do so, navigate to your `libraries` folder (usually located at `Documents/Arduino/libraries` on Windows and `~/Documents/Arduino/libraries` on macOS) and execute the following command:

```
git clone https://github.com/CIRCUITSTATE/CSE_MCP23017.git
```

The library can also be installed via **PlatformIO**. All officially listed Arduino listed libraries are automatically fetched by PlatformIO. Use the `lib_deps` search option to install the library.

# Examples

Two example sketches are included with this library which you can find inside the `examples` folder.

* `Print_GPRMC` - Directly reads the NMEA output from the GNSS module and prints it on the serial monitor.
* `View_GNSS_Data` - Reads the NMEA output from the GNSS module, extract the data and prints it on the serial monitor in key-value format.

# Tutorial

A complete tutorial on GPS/GNSS is available on the [CIRCUITSTATE website](https://www.circuitstate.com/tutorials/what-is-gps-gnss-how-to-interface-ublox-neo-6m-gps-module-with-arduino/). This tutorial uses the **u-blox NEO-6M GY-NEO6MV2** GPS module wired with a **FireBeetle-ESP32E** board.

# API Documentation

The API documentation is available at [API.md](/docs/API.md).

# References

1. [What is GPS/GNSS & How to Interface u-blox NEO-6M GPS Module with Arduino - CIRCUITSTATE Electronics](https://www.circuitstate.com/tutorials/what-is-gps-gnss-how-to-interface-ublox-neo-6m-gps-module-with-arduino/)

