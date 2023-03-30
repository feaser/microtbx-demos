# Demos for the MicroTBX-Modbus software component

This `modbus`  subdirectory contains demo programs for the basic [MicroTBX-Modbus](https://github.com/feaser/microtbx-modbus) software component.

## Overview

| Source file             | Description                                                  |
| ----------------------- | ------------------------------------------------------------ |
| `serverrtu.c`           | Demonstrates how to setup a Modbus RTU server, using a traditional super loop. |
| `serverrtu_freertos`    | Demonstrates how to setup a Modbus RTU server, using the FreeRTOS operating system. |
| `serverrtu_cplusplus.c` | Demonstrates how to setup a Modbus RTU server in C++, using a traditional super loop. |
| `clientrtu.c`           | Demonstrates how to setup a Modbus RTU client, using a traditional super loop. |
| `clientrtu_freertos.c`  | Demonstrates how to setup a Modbus RTU client, using the FreeRTOS operating system. |
| `clientrtu_cplusplus.c` | Demonstrates how to setup a Modbus RTU client in C++, using a traditional super loop. |

Refer to the included `README.md` in the board specific subdirectories for details on how to build and run these demo programs.
