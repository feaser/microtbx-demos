# Demos for the MicroTBX software component

This `base`  subdirectory contains demo programs for the basic [MicroTBX](https://github.com/feaser/microtbx) software component.

## Overview

| Source file      | Description                                                  |
| ---------------- | ------------------------------------------------------------ |
| `dynamicarray.c` | Demonstrates how to use the [memory pool](https://feaser.github.io/microtbx/mempools/) and [linked list](https://feaser.github.io/microtbx/lists/) modules to build a dynamic array. |
| `fifobuffer.c`   | Demonstrates how to use the [memory pool](https://feaser.github.io/microtbx/mempools/) and [linked list](https://feaser.github.io/microtbx/lists/) modules to build a first-in-first-out (FIFO) buffer. |
| `randomdata.c`   | Demonstrates how to use the [random number](https://feaser.github.io/microtbx/random/) generator to fill a data buffer with random data. |
| `securedata.c`   | Demonstrates how to use the [checksum](https://feaser.github.io/microtbx/checksum/) and [cryptography](https://feaser.github.io/microtbx/crypto/) modules to store and retrieve a block of data in a secure manner. |
| `sortlist.c`     | Demonstrates how to sort items in a [linked list](https://feaser.github.io/microtbx/lists/). |

Refer to the included `README.md` in the board specific subdirectories for details on how to build and run these demo programs.
