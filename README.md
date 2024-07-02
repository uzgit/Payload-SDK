# How to set up custom payload

The following is a set of guidelines for setting up the payload proposed in the paper [Lowering Barriers to Entry for Fully-Integrated Custom Payloads on a DJI Matrice](https://arxiv.org/abs/2405.06176): https://arxiv.org/abs/2405.06176.

## Overview

We propose a top-mounted, generic computational payload intended to be cheap and customizable. It uses the E-port for accessing the drone's camera feeds, sensors and data topics, and for controlling the drone's flight and the behavior of other payloads. It uses the PSDK port for streaming video to the controller using the drone's transmission infrastructure, so that an operator can see a custom visualization in real time without adding transmission equipment to the drone. Currently, this requires two PSDK applications to be running simultaneously -- one for each port. DJI may of course change this in the future. However, since the applications are not interconnected, one such application can be run on its own if the functionality of one of the ports is not required. This demo runs the DJI E-port demo code (accesses camera feeds, sensor data, controls flight and other payloads), and a custom application to stream the desktop of the Raspberry Pi to the controller. Finally, the payload fits in a 3D-printed case with quick-release brackets for convenient and quick installation in the field.

|  Installs on top  |  Fits in M350 case  |
| ------------- | ------------- |
| ![matrice_landed](https://github.com/uzgit/Payload-SDK/assets/14451567/0591db04-fbdc-40a4-b78b-ef8c1c4b73fe)  |  ![payload_in_case](https://github.com/uzgit/Payload-SDK/assets/14451567/ae6e5fb7-1634-4dd9-acae-dd766647ca67) |
| **Interior** | **Streams to Controller** |
| ![payload_electronics](https://github.com/uzgit/Payload-SDK/assets/14451567/48fe10d7-0b4d-4516-a579-6add097f0512)  |  ![payload_desktop_stream](https://github.com/uzgit/Payload-SDK/assets/14451567/8cc97706-ff47-4ac2-aa09-a77d23e5d6d3) |

## Physical setup

For creating the 3D-printed parts, see the Thingiverse entry here: https://www.thingiverse.com/thing:6681945.

We are using the following electronics:
* **Raspberry PI 5:** the whole reason for the existence of this payload -- a Linux computational environment that provides access to the drone.
* **E-port expansion board:** an interface providing USB-C access to the drone via its E-port (https://store.dji.com/product/dji-e-port-development-kit?vid=141171)
* **SkyPort V2 expansion board:** an interface providing ethernet access to the drone via its PSDK port -- and implicitly via an upward gimbal connector. (https://store.dji.com/product/psdk-development-kit-v2?vid=89481, https://store.dji.com/product/m300-upward-gimbal-connector?vid=100911)
* **DFR0753 DC-to-DC converter:** a power supply unit that can provide a more reliable 5V source to the Raspberry Pi than the expansion boards can.

### Power
The Raspberry Pi 5 can consume 5A at 5V, which is a lot to keep up with. Although the E-port board provides a 5V source, it is unable to power the Pi at full speed according to our tests. Therefore, we instead pull from the E-port's 12V source, stepping down to 5V with the DFR0753, which is more reliable. We connect the output of the DFR0753 to the Raspberry Pi's 5V and GND GPIO pins. This is the only connection that is explicitly for power on this setup.

### Data
We connect the USB-C connector on the E-port board to the Raspberry Pi's main USB-C connector. This provides power and data to the Pi. We transfer data using 2 bulk devices configured over this single USB link (explained later). It is possible to use Ethernet over USB, but this interferes with the SkyPort V2 expansion board, which requires Ethernet. Further, fewer data sources are available over the Ethernet over USB link, e.g., the drone's onboard depth cameras.

We connect the Ethernet port on the SkyPort V2 expansion board to the Raspberry Pi's Ethernet port. The SkyPort V2 expansion board connects to the SkyPort V2 connector using its included cable, and the SkyPort V2 connector connects to an upward gimbal connector, which connects to the drone's PSDK port. We have disassembled the upward gimbal connector and the SkyPort V2 connector and reassembled them using a low-profile 3D printed assembler that allows the pieces to fit inside the payload case. We do not draw power from the SkyPort V2 expansion board.



---
Below is the normal readme given by DJI.

# DJI Payload SDK (PSDK)

![](https://img.shields.io/badge/version-V3.8.1-purple.svg)
![](https://img.shields.io/badge/platform-linux_|_rtos-green.svg)
![](https://img.shields.io/badge/license-MIT-blue.svg)

## What is the DJI Payload SDK?

The DJI Payload SDK(PSDK), is a development kit provided by DJI to support developers to develop payload that can be
mounted on DJI drones. Combined with the X-Port, SkyPort or extension port adapter, developers can obtain the
information or other resource from the drone. According to the software logic and algorithm framework designed by the
developer, users could develop payload that can be mounted on DJI Drone, to perform actions they need, such as Automated
Flight Controller, Payload Controller, Video Image Analysis Platform, Mapping Camera, Megaphone And Searchlight, etc.

## Documentation

For full documentation, please visit
the [DJI Developer Documentation](https://developer.dji.com/doc/payload-sdk-tutorial/en/). Documentation regarding the
code can be found in the [PSDK API Reference](https://developer.dji.com/doc/payload-sdk-api-reference/en/)
section of the developer's website. Please visit
the [Latest Version Information](https://developer.dji.com/doc/payload-sdk-tutorial/en/)
to get the latest version information.

## Latest Release

The latest release version of PSDK is 3.8.1. This version of Payload SDK mainly add some new features support and fixed some
bugs. Please refer to the release notes for detailed changes list.

* Removed the camera management module interface DjiCameraManager_GetPhotoBurstCount.
* Removed the AEB photography function in the camera management module.
* Fixed the issue where subscribing to real-time point cloud data was not byte-aligned on some platforms.
* Fixed the discrepancy between flight speed units and annotations in the motion planning WP 2.0 feature.
* Fixed issues with obtaining the camera shooting mode and zoom magnification on the Mavic 3 series drones.
* Optimized the startup time for PSDK initialization.
* Complemented the flight control topic CONTROL_DEVICE data structure definition and added related enumeration ranges.
* Fixed the incorrect zoom multiplier retrieval for the H20N camera.
* Fixed the issue where custom media file suffix names were not taking effect.
* Fixed the occasional subscription errors for single battery data in the M300 RTK and M350 RTK.
* Fixed anomalous Pilot widget values for the M30 series, M300 RTK, and M350 RTK remote controllers.
* Note: Streamlined the toolchain for compiling PSDK, removing low-usage toolchain libraries.
> We have stopped offering some toolchains on GitHub. If you have trouble building your project, reach out to the SDK Support Team for help.

## License

Payload SDK codebase is MIT-licensed. Please refer to the LICENSE file for detailed information.

## Support

You can get official support from DJI and the community with the following methods:

- Post questions on Developer Forum
    * [DJI SDK Developer Forum(Cn)](https://djisdksupport.zendesk.com/hc/zh-cn/community/topics)
    * [DJI SDK Developer Forum(En)](https://djisdksupport.zendesk.com/hc/en-us/community/topics)
- Submit a request describing your problem on Developer Support
    * [DJI SDK Developer Support(Cn)](https://djisdksupport.zendesk.com/hc/zh-cn/requests/new)
    * [DJI SDK Developer Support(En)](https://djisdksupport.zendesk.com/hc/en-us/requests/new)

You can also communicate with other developers by the following methods:

- Post questions on [**Stackoverflow**](http://stackoverflow.com) using [**
  dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag

## About Pull Request
As always, the DJI Dev Team is committed to improving your developer experience, and we also welcome your contribution,
but the code review of any pull request maybe not timely, when you have any questionplease send an email to dev@dji.com.
