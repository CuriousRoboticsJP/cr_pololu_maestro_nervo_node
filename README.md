# cr_pololu_maestro_nervo_node
ROS2 node to connect with the Pololu Maestro Servo controller.

## Requirements
* Ubuntu 22
* ROS2 Humble
* libusb-1.0-0-dev

## Docker + WSL2 Usage
* WSL2 with Ubuntu 22.04
* USBIPD which can be found [here](https://github.com/dorssel/usbipd-win)
* Connect USB device instructions [here](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

## Pololu Documentation
The documentation for the Pololu controllers is available [here](https://www.pololu.com/resources/documentation)

The utilities have been provided here with an install script. The package can be untarred and used locally.
```bash
cd ./install
sudo ./install_pololu_utils.sh
```
NOTE:
* The Pololu Utilities are unpackaged in the install folder under `maestro-linux`.
* The utilities are meant to be run locally on the host, not in the docker container!

