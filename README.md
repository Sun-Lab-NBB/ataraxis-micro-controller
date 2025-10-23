# ataraxis-micro-controller

A C++ library for Arduino and Teensy microcontrollers that provides the framework for integrating custom hardware 
modules with a centralized PC control interface.

[![PlatformIO Registry](https://tinyurl.com/bdhdc2d8)](https://tinyurl.com/mrj7h9an)
![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=fff&style=plastic)
![license](https://img.shields.io/badge/license-GPLv3-blue)

___

## Detailed Description

This library allows integrating custom hardware modules of any complexity managed by the Arduino or Teensy 
microcontrollers with the [centralized PC interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface) 
implemented in Python. To do so, the library defines a shared API that can be integrated into 
user-defined modules by subclassing the (base) Module class. It also provides the Kernel class that manages runtime 
task scheduling, and the Communication class, which handles high-throughput bidirectional communication with the PC.

___

## Features

- Supports all recent Arduino and Teensy architectures and platforms.
- Provides an easy-to-implement API that integrates any hardware with the centralized host-computer (PC) 
  [interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface) written in Python.
- Abstracts communication and runtime task scheduling, allowing end users to focus on implementing the logic of their 
  custom hardware modules.
- Supports concurrent command execution for multiple module instances.
- Contains many sanity checks performed at compile time and initialization to minimize the potential for unexpected
  behavior and data corruption.
- GPL 3 License.

___

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [API Documentation](#api-documentation)
- [Developers](#developers)
- [Versioning](#versioning)
- [Authors](#authors)
- [License](#license)
- [Acknowledgements](#Acknowledgments)

___

## Dependencies

- An IDE or Framework capable of uploading microcontroller software that supports
  [Platformio](https://platformio.org/install). This library is explicitly designed to be uploaded via Platformio and
  will likely not work with any other IDE or Framework.

***Note!*** Developers should see the [Developers](#developers) section for information on installing additional
development dependencies.

___

## Installation

### Source

Note, installation from source is ***highly discouraged*** for anyone who is not an active project developer.

1. Download this repository to the local machine using the preferred method, such as git-cloning. Use one of the 
   [stable releases](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller/releases).
2. Unpack the downloaded tarball and move all 'src' contents into the appropriate destination
   ('include,' 'src,' or 'libs') directory of the project that needs to use this library.
3. Add `include <kernel.h>`, `include <communication.h>`, and `include <module.h>` at the top of the main.cpp file and 
   `include <module.h>` at the top of each custom hardware module header file.

### Platformio

1. Navigate to the project’s platformio.ini file and add the following line to the target environment specification:
   ```lib_deps = inkaros/ataraxis-micro-controller@^2.0.0```.
2. Add `include <kernel.h>`, `include <communication.h>`, and `include <module.h>` at the top of the main.cpp file and
   `include <module.h>` at the top of each custom hardware module header file.

___

## Usage

### Quickstart
This section demonstrates how to use custom hardware modules compatible with this library. See 
[this section](#implementing-custom-hardware-modules) for instructions on how to implement custom hardware module 
classes. Note, the example below should be run together with the 
[companion python interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface#quickstart) example. See 
the [module_integration.cpp](./examples/module_integration.cpp) for the .cpp implementation of this example:
```
// Dependencies
#include "../examples/example_module.h"  // Since there is an overlap with the general 'examples', uses the local path.
#include "Arduino.h"
#include "communication.h"
#include "kernel.h"
#include "module.h"

// Specifies the unique identifier for the test microcontroller
constexpr uint8_t kControllerID = 222;

// Initializes the Communication class. This class instance is shared by all other classes and manages incoming and
// outgoing communication with the companion host-computer (PC). The Communication has to be instantiated first.
// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
Communication axmc_communication(Serial);

// Creates two instances of the TestModule class. The first argument is the module type (family), which is the same (1)
// for both, the second argument is the module ID (instance), which is different. The type and id codes do not have
// any inherent meaning, they are defined by the user and are only used to ensure specific module instances can be
// uniquely addressed during runtime.
TestModule<> test_module_1(1, 1, axmc_communication);

// Also uses the template to override the digital pin controlled by the module instance from the default (5) to 6.
TestModule<6> test_module_2(1, 2, axmc_communication);

// Packages all module instances into an array to be managed by the Kernel class.
Module* modules[] = {&test_module_1, &test_module_2};

// Instantiates the Kernel class. The Kernel has to be instantiated last.
Kernel axmc_kernel(kControllerID, axmc_communication,  modules);

// This function is only executed once. Since Kernel manages the setup for each module, there is no need to set up each
// module's hardware individually.
void setup()
{
    // Initializes the serial communication.
    Serial.begin(115200);

    // Sets up the hardware and software for the Kernel and all managed modules.
    axmc_kernel.Setup();
}

// This function is executed repeatedly while the microcontroller is powered.
void loop()
{
    // Since the Kernel instance manages the runtime of all modules, the only method that needs to be called
    // here is the RuntimeCycle method.
    axmc_kernel.RuntimeCycle();
}
```

### User-Defined Variables
This library is designed to flexibly support many different use patterns. To do so, it intentionally avoids hardcoding
certain metadata variables that allow the PC interface to individuate and address the managed microcontroller and 
specific hardware module instances. **Each end user has to manually define these values both for the microcontroller 
and the PC.**

- `Controller ID`. This is a unique code from 1 to 255 that identifies the microcontroller. This ID code is used when 
   communicating with the microcontroller and logging the data received from the microcontroller, so 
   **it has to be unique for all microcontrollers and other Ataraxis assets used at the same time.** For example, 
   [Video System](https://github.com/Sun-Lab-NBB/ataraxis-video-system) classes also use the IDs to 
   identify themselves during communication and logging and **clash** with microcontroller IDs if both are used at the
   same time.

- `Module Type` for each hardware module instance. This is a unique code from 1 to 255 that identifies the family 
   (class) of each module instance. For example, all solenoid valves may use the type-code '1,' while all voltage 
   sensors may use the type-code '2.' The type-codes do not have any inherent meaning. Their interpretation depends 
   entirely on the end-user’s preference when implementing the hardware module and its PC interface.

- `Module ID` for each hardware module instance. This code has to be unique within the module type (family) and is used 
   to identify specific module instances. For example, if two voltage sensors (type code 2) are used at the same time, 
   the first voltage sensor should use ID code 1, while the second sensor should use ID code 2.

### Custom Hardware Modules
For this library, any external hardware that communicates with Arduino or Teensy microcontroller pins is a hardware 
module. For example, a 3d-party voltage sensor that emits an analog signal detected by an Arduino microcontroller is a 
module. A rotary encoder that sends digital interrupt signals to 3 digital pins of a Teensy 4.1 microcontroller is a 
module. A solenoid valve gated by HIGH signal sent from an Arduino microcontroller’s digital pin is a module.

The library expects that the logic that governs how the microcontroller interacts with these modules is 
provided by a C++ class, the 'software' portion of the hardware module. Typically, this class contains the methods for 
manipulating the hardware module or collecting the data from the hardware module. The central purpose 
of this library is to enable the centralized PC interface, implemented in Python, to work with a wide range of custom 
hardware modules in a standardized fashion. To achieve this, all custom hardware modules have to subclass the base 
[Module](/src/module.h) class, provided by this library. See the section below for details on how to implement 
compatible hardware modules.

### Implementing Custom Hardware Modules
All modules intended to be accessible through this library have to follow the implementation guidelines described in the
[example module header file](./examples/example_module.h). Specifically, **all custom modules have to subclass the 
Module class from this library and overload all pure virtual methods**. Additionally, it is highly advised to implement 
custom command logic for the Module using the **stage-based design pattern** shown in the example. Note, all examples 
featured in this guide are taken directly from the [example_module.h](./examples/example_module.h) and the 
[module_integration.cpp](./examples/module_integration.cpp).

The library is intended to be used together with the 
[companion PC interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface). **Each custom hardware 
module class implemented using this library must have a companion ModuleInterface class implemented in Python.** 
These two classes act as the endpoints of the PC-Microcontroller interface, while other library assets abstract the 
intermediate steps that connect the PC interface class with the microcontroller hardware logic class.

***Do not directly access the Kernel or Communication classes when implementing custom hardware modules.*** The base 
Module class allows accessing all necessary library assets through the inherited [utility methods](#utility-methods).

#### Concurrent (Non-Blocking) Execution
A major feature of the library is that it allows maximizing the microcontroller throughput by partially overlapping the 
execution of multiple commands under certain conditions. Specifically, it allows executing other commands while waiting 
for a time-based delay in the currently executed command. This feature is especially relevant for higher-end 
microcontrollers, such as Teensy 4.0+, that can execute many instructions during a multi-millisecond delay interval.

During each cycle of the main microcontroller `loop()` function, the Kernel sequentially instructs each managed module 
instance to execute its active command. Typically, the module runs through the command, delaying code execution as 
necessary, and resulting in the microcontroller doing nothing during the delay. With this library, commands can use the 
`WaitForMicros` utility method together with the **stage-based design pattern** showcased by the 
[TestModule’s Pulse command](./examples/example_module.h) to allow other modules to run their commands while the module 
waits for the delay to expire.

**Warning!** The non-blocking mode is most effective when used with delays that tolerate a degree of imprecision or on 
microcontrollers that have a very high CPU clock speed. Additionally, to support non-blocking runtimes, all 
modules used at the same time must support non-blocking execution for all commands. Overall, the decision of whether to
use the non-blocking mode often requires practical testing under the intended runtime conditions and may not be optimal
for certain use cases.

**Note!** While this library only supports non-blocking execution for time-based delays natively, advanced users can 
follow the same design principles to implement non-blocking sensor-based delays when implementing custom command logic.

#### Virtual methods
These methods provide the inherited API that integrates any custom hardware module with the centralized control 
interface running on the companion host-computer (PC). Specifically, the Kernel calls these methods during runtime to 
interface with each managed module instance.

#### SetCustomParameters
This method enables the Kernel to unpack and save the module’s runtime parameters, when updated parameter values are 
received from the PC. The primary purpose of this virtual method is to tell the Kernel where to unpack the module’s
parameter data.

For most use cases, the method can be implemented with a single call to the ExtractParameters() utility method 
inherited from the base Module class:
```
bool SetCustomParameters() override
{
    return ExtractParameters(parameters);  // Unpacks the received parameter data into the storage object
}
```

The `parameters` object is typically a structure that stores the instance’s PC-addressable runtime parameters. 
The `ExtractParameters()` utility method, reads the data received from the PC and uses it to overwrite the memory of 
the provided object.

### RunActiveCommand
This method enables the Kernel to execute the managed module’s logic in response to receiving module-addressed commands 
from the PC. Specifically, the Kernel receives and queues the commands to be executed and then calls this method for 
each managed module to run the queued command’s logic. The primary purpose of this method is to translate the active 
command code into the call to the command’s logic method.

For most use cases, this method can be implemented with a simple switch statement:
```
switch (static_cast<kCommands>(GetActiveCommand()))
{
    // Active command matches the code for the Pulse command
    case kCommands::kPulse:
        Pulse();      // Executes the command logic
        return true;  // Notifies the Kernel that command was recognized and executed
       
    // Active command matches the code for the Echo command
    case kCommands::kEcho: Echo(); return true;
    
    // Active command does not match any valid command code
    default: return false;  // Notifies the Kernel that the command was not recognized
}
```

The switch uses the `GetActiveCommand()` method, inherited from the base Module class, to retrieve the code of the 
currently active command. The Kernel assigns this command to the module for each runtime loop cycle. It is recommended 
to use an enumeration to map valid command codes to meaningful names addressable in code, like it is done with the 
`kCommands` enumeration in the demonstration above. The switch statement matches the command code to one of the valid 
commands, calls the function associated with each command, and returns `true`. Note, the method ***has*** to returns 
`true` if it recognized the command and return `false` if it did not. The return of this method ***only*** communicates 
whether the command was recognized. It should ***not*** be used to track the runtime status (success / failure) of the 
called command’s method.

### SetupModule
This method enables the Kernel to set up the hardware and software assets for each managed module instance. This is 
done from the global `setup()` function, which is executed by the Arduino and Teensy microcontrollers after firmware 
reupload. This is also done in response to the PC requesting the controller to be reset to the default state. The 
primary purpose of this method is to initialize the hardware and software of each module instance to the state that 
supports the runtime.

The implementation of this method should follow the same guidelines as the general microcontroller `setup()` function:
```
bool SetupModule() override
{
    // Configures class hardware
    pinMode(kPin, OUTPUT);
    digitalWrite(kPin, LOW);

    // Configures class software
    parameters.on_duration  = 2000000;
    parameters.off_duration = 2000000;
    parameters.echo_value   = 123;
    
    // Notifies the Kernel that the setup is complete
    return true;
}
```

It is recommended to implement the method in a way that always returns `true` and does not fail. However, to support 
the runtimes that need to be able to fail, the method supports returning `false` to notify the Kernel that the setup has
failed. If this method returns `false`, the Kernel deadlocks the microcontroller in the error state until the 
microcontroller firmware is reuploaded to fix the setup error.

### Utility methods
To further simplify implementing custom hardware modules, the base Module class exposes a collection of utility methods.
These methods provide an easy-to-use API for safely accessing internal attributes and properties of the superclass, 
simplifying the interaction between the superclass (Module) and the custom logic of each hardware module that inherits 
from the base class.

See the [API Documentation of the base Module class](#api-documentation) for the list of available (protected) Utility
methods. Also, see the [TestModule](./examples/example_module.h) for the demonstration on how to use some of these 
methods when implementing custom hardware module, most notably those relating to sending the data to the PC and using
the stage-based command design pattern.

___

## API Documentation

See the [API documentation](https://ataraxis-micro-controller-api-docs.netlify.app/) for the detailed description of
the methods and classes exposed by components of this library.

___

## Developers

This section provides installation, dependency, and build-system instructions for project developers.

### Installing the Project

1. Install [Platformio](https://platformio.org/install/integration) either as a standalone IDE or as an IDE plugin.
2. Download this repository to the local machine using the preferred method, such as git-cloning.
3. If the downloaded distribution is stored as a compressed archive, unpack it using the appropriate decompression tool.
4. ```cd``` to the root directory of the prepared project distribution.
5. Run ```pio project init ``` to initialize the project on the local machine. See
   [Platformio API documentation](https://docs.platformio.org/en/latest/core/userguide/project/cmd_init.html) for
   more details on initializing and configuring projects with platformio.
6. If using an IDE that does not natively support platformio integration, call the ```pio project metadata``` command
   to generate the metadata to integrate the project with the IDE. Note; most mainstream IDEs do not require or benefit
   from this step.

***Warning!*** To build this library for a platform or architecture that is not explicitly supported, edit the
platformio.ini file to include the desired configuration as a separate environment. This project comes preconfigured
with support for `teensy 4.1`, `arduino due`, and `arduino mega (R3)` platforms.

### Additional Dependencies

In addition to installing Platformio and main project dependencies, install the following dependencies:

- [Tox](https://tox.wiki/en/4.15.0/user_guide.html) and [Doxygen](https://www.doxygen.nl/manual/install.html) to build
  the API documentation for the project. Note; both dependencies have to be available on the local system’s path.

### Development Automation

Unlike other Ataraxis libraries, the automation for this library is primarily provided via the
[Platformio’s command line interface](https://docs.platformio.org/en/latest/core/userguide/index.html).
Additionally, this project uses [tox](https://tox.wiki/en/latest/user_guide.html) for certain automation tasks not
directly covered by platformio, such as API documentation generation. Check the [tox.ini file](tox.ini) for details
about the available pipelines and their implementation. Alternatively, call ```tox list``` from the root directory of
the project to see the list of available tasks.

**Note!** All pull requests for this project have to successfully complete the `tox`, `pio check`, and `pio test` tasks
before being submitted.

---

## Versioning

This project uses [semantic versioning](https://semver.org/). See the
[tags on this repository](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller/tags) for the available project
releases.

---

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))
- Jasmine Si

---

## License

This project is licensed under the GPL3 License: see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- All Sun lab [members](https://neuroai.github.io/sunlab/people) for providing the inspiration and comments during the
  development of this library.
- The creators of all other dependencies and projects listed in the [platformio.ini](platformio.ini) file.

---
