These demo programs are targeted towards an ST [Nucleo-F091RC](https://www.st.com/en/evaluation-tools/nucleo-f091rc.html) board and are configured for building with the [GNU ARM Embedded](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) toolchain.

# Prerequisites

It is assumed that [CMake](https://cmake.org/) the [ARM GCC Embedded toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) are installed on the system and available on the PATH. Additionally, the GNU tools *make* and *rm* should be installed and available on the path. Windows users can install the [GNU MCU Eclipse Windows Build Tools](https://gnu-mcu-eclipse.github.io/windows-build-tools/) for this. On a Debian/Ubuntu based system, the GNU tools can be installed with the following command:

```sh
sudo apt-get install build-essential
```

# Building

## From the terminal

Using the terminal, set the working directory to where you clone the demo repository (e.g. `~/MicroTBXDemos`). Next type the following commands to automatically generate the build environment with CMake for all included demo programs:

```bash
cd build
cmake ..
```

To build a specific demo program, go to its subdirectory inside the `build` directory and type the `make` command. Example for the *dynamic array* demo program, assuming that you are already in the `build` subdirectory:

```bash
cd demos/base/ARM_CORTEXM_ST_Nucleo_F091RC/GCC/dynamicarray
make
```

This creates the `NucleoF091RC_DynamicArray.elf` executable in that same directory. This file can be programmed onto the ST Nucleo-F091RC board, using the on-board ST-Link debugger interface and with the help of the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) software program.

## Using Visual Studio Code

Even more convenient is using Visual Studio Code. From within Visual  Studio Code, open the folder where you cloned the demo repository to (e.g. `~/MicroTBXDemos`). It's the one that has the `.vscode` directory. The first time it will prompt you to install the recommended extensions. Next:

- Select the `arm-none-eabi` kit in the status bar. If not shown, use the option *Scan for kits* first and then select the `arm-none-eabi` one.
- Set both the build and launch targets to `NucleoF091RC_DynamicArray` in the status bar (or any other demo program that you want to build).

From there on, you can build the usual way (<kbd>F7</kbd>).

### Debugging

The `.vscode` directory in the root folder contains a pre-configured template for debugging with [OpenOCD](https://openocd.org/). Just make sure OpenOCD is installed and accessible on the PATH. Switch to the debug view, select the `NucleoF091RC` launch configuration (next to the green play-button), and click the  green play-button to flash and start a debug session. Afterwards you can also use the <kbd>F5</kbd> shortcut.

