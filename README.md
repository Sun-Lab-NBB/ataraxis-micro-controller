# SerializedTransferProtocol-Microcontroller

The version of the SerializedTransferProtocol intended to be used with Arduino and Teensy microcontrollers to enable
bidirectional serialized communication with other devices over the USB or UART port.

## Detailed Description

This project uses the 'chrono' C++ library to access the fastest available system clock and use it to provide interval
timing and delay functionality via a Python binding API. While the performance of the timer heavily depends on the
particular system configuration and utilization, most modern CPU should be capable of sub-microsecond precision using
this timer.

## Features

- Supports Windows, Linux and OSx.
- Sub-microsecond precision on modern CPUs (~ 3 GHz+).
- Pure-python API.
- Fast C++ core with direct extension API access via nanobind.
- GPL 3 License.

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Developers](#developers)
- [Authors](#authors)
- [License](#license)
- [Acknowledgements](#Acknowledgments)

## Dependencies

For users, all library dependencies are installed automatically when using the appropriate installation specification
for all supported installation methods (see [Installation](#Installation) section). For developers, see the
[Developers](#Developers) section for information on installing additional development dependencies.

## Installation

### Source

1. Download this repository to your local machine using your preferred method, such as git-cloning.
2. ```cd``` to the root directory of the project using your CLI of choice.
3. Run ```python -m pip install .``` to install the project. Optionally, use the
   ```python -m pip install .'[benchmark]'``` command to install benchmark script dependencies.
4. Optionally run the timer benchmark using ```benchmark_timer``` command from your CLI (no need to use 'python'
   directive).

### PIP

Use the following command to install the library using PIP:
```pip install high-precision-timer```

Optionally, use the following command to instead to also install benchmark dependencies:
```pip install high-precision-timer[benchmark]```

### Conda / Mamba

Use the following command to install the library using Conda or Mamba:
```conda install high-precision-timer```

Optionally, use the following command to instead to also install benchmark dependencies:
```conda install high-precision-timer with_benchmark```

## Usage

This is a minimal example of how to use this library:

```
# First, import the timer class.
from high_precision_timer.precision_timer import PrecisionTimer
import time as tm

# Then, instantiate the timer class using the desired precision. Supported precisions are: 'ns' (nanoseconds),
# 'us' (microseconds), 'ms' (milliseconds) and 's' seconds.
timer = PrecisionTimer('us')

# Interval timing example
timer.reset()  # Resets (re-bases) the timer
tm.sleep(1)  # Simulates work (for 1 second)
print(f'Work time: {timer.elapsed} us')  # This returns the 'work' duration using the precision units of the timer.

print()  # Separates interval example from delay examples

# Delay example:
for i in range(10):
    print(f'us delay iteration: {i}')
    timer.delay_block(500)  # Delays for 500 microseconds, does not release the GIL

print()  # Separates the us loop from ms loop

timer.set_precision('ms')  # Switches timer precision to milliseconds
for i in range(10):
    print(f'ms delay iteration: {i}')
    timer.delay_noblock(500)  # Delays for 500 milliseconds, releases the GIL
```

See the API documentation[LINK STUB] for the detailed description of the methods and their arguments, exposed through
the PrecisionTimer python class or the direct CPrecisionTimer binding class.

## Developers

This section provides additional installation, dependency and build-system instructions for the developers that want to
modify the source code of this library

### Installing the library

1. Download this repository to your local machine using your preferred method, such as git-cloning.
2. ```cd``` to the root directory of the project using your CLI of choice.
3. Run ```python -m pip install .'[dev]'``` command to install development dependencies.

### Additional Dependencies

In addition to installing the python packages, separately install the following dependencies:

- [Doxygen](https://www.doxygen.nl/manual/install.html), if you want to generate C++ code documentation.

### Development Automation

To assist developers, this project comes with a set of fully configured 'tox'-based pipelines for verifying and building
the project. Each of the tox commands builds the project in an isolated environment before carrying out its task.

Below is a list of all available commands and their purpose:

- ```tox -e lint``` Checks and, where safe, fixes code formatting, style, and type-hinting.
- ```tox -e test``` Builds the projects and executes the tests stored in the /tests directory using pytest-coverage
  module.
- ```tox -e doxygen``` Uses the externally-installed Doxygen distribution to generate documentation from docstrings of
  the C++ extension file.
- ```tox -e docs``` Uses Sphinx to generate API documentation from Python Google-style docstrings. If Doxygen-generated
  .xml files for the C++ extension are available, uses Breathe plugin to convert them to Sphinx-compatible format and
  add
  them to the final API .html file.
- ```tox --parallel``` Carries out all commands listed above in-parallel (where possible). Remove the '--parallel'
  argument to run the commands sequentially. Note, this command will build and test the library for all supported python
  versions.
- ```tox -e build``` Builds the binary wheels for the library for all architectures supported by the host machine. Note,
  you need to manually adjust the 'test-skip' argument of the '[tool.cibuildwheel]' section inside the pyproject.toml
  file
  to disable testing the wheels for non-native architectures (e.g: testing x86 wheels is possible, but will not work for
  M1 OSx distributions).

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))

## License

This project is licensed under the GPL3 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- All Sun Lab [WEBSITE LINK STUB] members for providing the inspiration and comments during the development of this
  library.
- My NBB Cohort for answering 'random questions' pertaining to the desired library functionality.
