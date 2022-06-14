# MicroTBX Demos
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![Build Demos](https://github.com/feaser/microtbx-demos/actions/workflows/build_demos.yml/badge.svg)](https://github.com/feaser/microtbx-demos/actions/workflows/build_demos.yml)

An assortment of demo programs to help you get started with the [MicroTBX](https://github.com/feaser/microtbx) software components in your own embedded software applications. 

MicroTBX is an open source Microcontroller ToolBoX consisting of software components commonly needed in embedded software applications. Think of it as a Swiss army knife for your firmware.

## Getting the code

To get the code, clone the Git repository to a subdirectory, for example inside your own home directory:

    git clone https://github.com/feaser/microtbx-demos.git ~/MicroTBXDemos

This Git repository makes use of submodules. This means that one more step is needed to also get the code of the submodules: `git submodule update --init`. For example:

```
cd ~/MicroTBXDemos
git submodule update --init
```

Alternatively, you can clone and obtain the submodules in one go by using the `--recursive` argument. Example:

```
git clone --recursive https://github.com/feaser/microtbx-demos.git ~/MicroTBXDemos
```

# Contact

Development and maintenance of MicroTBX is sponsored by Feaser. Feaser also offers integration/customization services, and professional technical support on a commercial basis:

* [https://www.feaser.com](https://www.feaser.com)

