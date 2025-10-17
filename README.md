# ataraxis-micro-controller

A C++ library for Arduino and Teensy microcontrollers that provides the framework for integrating custom hardware 
modules with a centralized PC control interface.

[![PlatformIO Registry](https://tinyurl.com/bdhdc2d8)](https://tinyurl.com/mrj7h9an)
![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=fff&style=plastic)
![license](https://img.shields.io/badge/license-GPLv3-blue)

___

## Detailed Description

This library is designed to simplify the integration of custom hardware, managed by Arduino or Teensy microcontrollers,
with existing project Ataraxis libraries and infrastructure running on host-computers (PCs). To do so, it exposes 
classes that abstract microcontroller-PC communication and microcontroller runtime management (task scheduling, error 
handling, etc.). Jointly, these classes bind custom hardware to the Python PC
[interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface), enabling the 
centralized control of many hardware modules. To do so, the library defines a shared API that can be integrated into 
user-defined modules by subclassing the (base) Module class. It also provides the Kernel class that manages task 
scheduling during runtime, and the Communication class, which allows custom modules to communicate to Python clients
(via a specialized binding of the [TransportLayer class](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc)).
___

## Features

- Supports all recent Arduino and Teensy architectures and platforms.
- Provides an easy-to-implement API that integrates any user-defined hardware with the centralized host-computer (PC) 
  [interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface) written in Python.
- Abstracts communication and microcontroller runtime management through a set of classes that can be tuned to optimize 
  latency or throughput.
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

### Main Dependency
- An IDE or Framework capable of uploading microcontroller software. This library is designed to be used with
  [Platformio,](https://platformio.org/install) and we strongly encourage using this IDE for Arduino / Teensy
  development. Alternatively, [Arduino IDE](https://www.arduino.cc/en/software) also satisfies this dependency, but
  is not officially supported or recommended for most users.

### Additional Dependencies
These dependencies will be automatically resolved whenever the library is installed via Platformio. ***They are
mandatory for all other IDEs / Frameworks!***

- [digitalWriteFast](https://github.com/ArminJo/digitalWriteFast).
- [elapsedMillis](https://github.com/pfeerick/elapsedMillis/blob/master/elapsedMillis.h).
- [ataraxis-transport-layer-mc](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc).

For developers, see the [Developers](#developers) section for information on installing additional development 
dependencies.
___

## Installation

### Source

Note, installation from source is ***highly discouraged*** for everyone who is not an active project developer.
Developers should see the [Developers](#Developers) section for more details on installing from source. The instructions
below assume you are ***not*** a developer.

1. Download this repository to your local machine using your preferred method, such as Git-cloning. Use one
   of the stable releases from [GitHub](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller/releases).
2. Unpack the downloaded tarball and move all 'src' contents into the appropriate destination
   ('include,' 'src' or 'libs') directory of your project.
3. Add `include <kernel.h>`, `include <communication.h>`, `include <axmc_shared_assets.h>` at the top of the main.cpp 
   file and `include <module.h>`, `include <axmc_shared_assets.h>` at the top of each custom hardware module header 
   file.

### Platformio

1. Navigate to your platformio.ini file and add the following line to your target environment specification:
   ```lib_deps = inkaros/ataraxis-micro-controller@^1.0.0```. If you already have lib_deps specification, add the
   library specification to the existing list of used libraries.
2. Add `include <kernel.h>`, `include <communication.h>`, `include <axmc_shared_assets.h>` at the top of the main.cpp
   file and `include <module.h>`, `include <axmc_shared_assets.h>` at the top of each custom hardware module header 
   file.
___

## Usage

### Quickstart
This section demonstrates how to use custom hardware modules compatible with this library. See 
[this section](#implementing-custom-hardware-modules) for instructions on how to implement your own hardware modules. 
Note, the example below should be run together with the 
[companion python interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface#quickstart) example. See 
the [module_integration.cpp](./examples/module_integration.cpp) for this example’s .cpp file.
```
// This example demonstrates how to write the main.cpp file that uses the library to integrate custom hardware modules
// with the communication interface running on the companion host-computer (PC). This example uses the TestModule
// class implemented in the example_module.h file.

// This example is designed to be executed together with the companion ataraxis-communication-interface library running
// on the host-computer (PC): https://github.com/Sun-Lab-NBB/ataraxis-communication-interface#quickstart.
// See https://github.com/Sun-Lab-NBB/ataraxis-micro-controller#quickstart for more details.
// API documentation: https://ataraxis-micro-controller-api-docs.netlify.app/.
// Authors: Ivan Kondratyev (Inkaros), Jasmine Si.

// Dependencies
#include "../examples/example_module.h"  // Since there is an overlap with the general 'examples', uses the local path.
#include "Arduino.h"
#include "axmc_shared_assets.h"
#include "communication.h"
#include "kernel.h"
#include "module.h"

// Pre-initializes global assets
axmc_shared_assets::DynamicRuntimeParameters DynamicRuntimeParameters;  // Shared controller-wide runtime parameters
constexpr uint8_t kControllerID = 222;                                  // Unique ID for the test microcontroller

// Initializes the Communication class. This class instance is shared by all other classes and manages incoming and
// outgoing communication with the companion host-computer (PC). The Communication has to be instantiated first.
// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
Communication axmc_communication(Serial);

// Creates two instances of the TestModule class. The library can support multiple instances of different module
// families, but for this example we use the same family (type) and only create two instances. Note, the first argument
// is the module type (family), which is the same (1) for both, the second argument is the module ID (instance), which
// is different. Both type-codes and id-codes are assigned by the user at instantiation.
TestModule<5> test_module_1(1, 1, axmc_communication, DynamicRuntimeParameters);

// Also uses the template to override the digital pin controlled by the module instance from the default (5) to 6.
TestModule<6> test_module_2(1, 2, axmc_communication, DynamicRuntimeParameters);

// Packages all module instances into an array to be managed by the Kernel class.
Module* modules[] = {&test_module_1, &test_module_2};

// Instantiates the Kernel class. The Kernel has to be instantiated last.
Kernel axmc_kernel(kControllerID, axmc_communication, DynamicRuntimeParameters, modules);

// This function is only executed once. Since Kernel manages the setup for each module, there is no need to set up each
// module's hardware individually.
void setup()
{
    // Initializes the serial communication. If the microcontroller uses UART interface for serial communication, make
    // sure the baudrate defined here matches the one used by the PC. For Teensy and other microcontrollers that use
    // USB interface, the baudrate is usually ignored.
    Serial.begin(115200);

    // Sets up the hardware and software for the Kernel and all managed modules.
    axmc_kernel.Setup();  // Note, this HAS to be called at least once before calling RuntimeCycle() method.
}

// This function is executed repeatedly while the microcontroller is powered.
void loop()
{
    // Since Kernel instance manages the runtime of all modules, the only method that needs to be called here is the
    // RuntimeCycle method.
    axmc_kernel.RuntimeCycle();
}
```

### User-Defined Variables
This library is designed to flexibly support many different use patterns. To do so, it intentionally avoids hardcoding
certain metadata variables that allow the PC interface to individuate the managed microcontroller and specific hardware 
module instances. As a user, you **have to** manually define these values **both** for the microcontroller and the PC.
The PC and the Microcontroller have to have the **same** interpretation for these values to work as intended.

- `Controller ID`. This is a unique byte-code from 1 to 255 that identifies the microcontroller during communication. 
   This ID code is used when logging the data received from the microcontroller, so it has to be unique for all 
   microcontrollers **and other** Ataraxis systems used at the same time. For example, 
   [Video System](https://github.com/Sun-Lab-NBB/ataraxis-video-system) classes also use the byte-code ID system to 
   identify themselves during communication and logging and **will clash** with microcontroller IDs if you are using 
   both at the same time. This code is set by the **first** argument of the Kernel class constructor.

- `Module Type` for each module. This is a byte-code from 1 to 255 that identifies the family of each module. For 
   example, all solenoid valves may use the type-code '1,' while all voltage sensors may use type-code '2.' The type 
   codes do not have an inherent meaning, they are assigned by the user separately for each use case. Therefore, the
   same collection of custom module classes may have vastly different type-codes for two different projects. This 
   design pattern is intentional and allows developers to implement modules without worrying about clashing with 
   already existing modules. This code is set by the **first** argument of the base Module class constructor.

- `Module ID` for each module. This code has to be unique within the module type (family) and is used to identify 
   specific module instances. For example, this code will be used to identify different voltage sensors if more than 
   one sensor is used by the same microcontroller at the same time. This code is set by the **second** argument of the 
   base Module class constructor.

### Custom Hardware Modules
For this library, any external hardware that communicates with Arduino or Teensy microcontroller pins is a hardware 
module. For example, a 3d-party voltage sensor that emits an analog signal detected by Arduino microcontroller is a 
module. A rotary encoder that sends digital interrupt signals to 3 digital pins of a Teensy 4.1 microcontroller is a 
module. A solenoid valve gated by HIGH signal sent from an Arduino microcontroller’s digital pin is a module.

Additionally, the library expects that the logic that governs how the microcontroller interacts with these modules is 
provided by a C++ class, the 'software' portion of the hardware module. Typically, this would be a class that contains 
the methods for manipulating the hardware module or collecting the data from the hardware module. The central purpose 
of this library is to enable the PC communication interface to use the software of different hardware modules via a 
standardized and centralized process. To achieve this, all custom hardware modules have to be based on the 
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
[companion python interface](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface). For each custom Module 
class implemented using this library, there has to be a companion ModuleInterface class implemented in Python. These two
classes act as endpoints of the PC-Microcontroller interface, while the libraries abstract all steps on the PC and 
Microcontroller connecting the two endpoints during runtime.

Do **not** directly access the Kernel or Communication classes when implementing custom modules. The base Module class 
allows accessing all necessary assets of this library through the [Utility methods](#utility-methods).

#### Concurrent Execution
One major feature of the library is that it allows maximizing the microcontroller throughput by overlapping the 
execution of commands from multiple modules under certain conditions. This feature is specifically aimed at higher-end 
microcontrollers, such as Teensy 4.0+, which operate at ~600Mhz by default and boost to ~1Ghz. These 
microcontrollers can execute many instructions during a typical multi-millisecond delay interval used
by many hardware implementations. Therefore, having the ability to run other modules’ command logic while executing
millisecond+ delays improves the overall command throughput of a higher-end microcontroller.

To enable this functionality, the library is explicitly designed to support stage-based command execution. During 
each cycle of the main `loop` function of Arduino and Teensy microcontrollers, the Kernel will sequentially instruct 
each managed module instance to execute its active command. Typically, the module would run through the command, 
delaying execution as necessary, and resulting in the microcontroller doing nothing during the delay. However, with 
this library, commands can use `WaitForMicros` utility method together with the design pattern showcased by the 
[TestModule’s Pulse command](./examples/example_module.h) to end command method runtime early, allowing other modules to
run their commands while the module waits for the delay to expire. High-end microcontrollers may be able to run 
many `loop` cycles while the delaying module waits for the delay to expire, greatly increasing the overall throughput.

Note, however, that this also reduces the precision of delays. When the controller blocks until the delay expires, it 
guarantees that there is a minimal lag between the end of the delay period and the execution of the following 
instructions. With non-blocking delay, the microcontroller will need to run through other module commands and Kernel 
logic before it cycles back to the delaying module. For higher-end microcontrollers the lag may be minimal. Similarly, 
for some runtimes the lag may not be significantly important. For lower end microcontrollers and time-critical runtimes,
however, it may be better to use the blocking mode. The non-blocking mode can be enabled on a per-command basis and, 
therefore, it is usually a good idea to write all commands in a way that supports non-blocking execution, even if they 
run in blocking mode.

Currently, this feature is only supported for time-based delays. In the future, it may be extended to supporting 
sensor-based delays.

#### Virtual methods
These methods link custom logic of each hardware module with the rest of the library API. Thy are called by the Kernel 
class and allow it to manage the runtime behavior of the module, regardless of the specific implementation of each 
module. This is what makes the library work with any hardware module design.

#### SetCustomParameters
This method enables the Kernel to unpack and save the module’s runtime parameters, when new values for these parameters
are received from the PC.

Usually, this method can be implemented with 1 line of code:
```
bool SetCustomParameters() override
{
    return ExtractParameters(parameters);  // Unpacks the received parameter data into the storage object
}
```
The `parameters` object can be any valid C++ object used for storing PC-addressable parameters, such as a structure or 
array. `ExtractParameters` is a utility method inherited from the base Module class, which reads the data transmitted 
from the PC and uses it to overwrite the memory of the provided object. Essentially, the core purpose of this virtual 
method is to tell the Kernel where to unpack the parameter data.

### RunActiveCommand
This method allows the Kernel to execute the managed module’s logic in response to receiving module-addressed commands 
from the PC. Specifically, the Kernel receives and queues the commands to be executed and then calls this method for 
each managed module. The method has to retrieve the code of the currently active command, match it to custom command 
logic, and call the necessary function(s).

There are many ways for implementing this method, but we use a simple switch statement for this demonstration:
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
The switch uses `GetActiveCommand` method, inherited from the base Module class, to retrieve the code of the currently 
active command. The Kernel assigns this command to the module for each runtime loop cycle. To simplify code 
maintenance, we assume that all valid command codes are stored in an enumeration, in this case the `kCommands`. The 
switch statement matches the command code to one of the valid commands, calls the function associated with each command
and returns `true`. Note, the method ***has*** to returns `true` if it recognized the command and return `false` if it
did not. It does not matter if the command was executed successfully or not, the return of this method ***only*** 
communicates whether the command was recognized or not.

### SetupModule
This method allows the Kernel to set up the hardware and software for each managed module. This is done from the global
`setup` function, which is executed by the Arduino and Teensy microcontroller after firmware reupload. This is also done
in response to the PC requesting the controller to be reset to the default state.

Generally, this method would follow the same implementation guidelines as you would when writing the general 
microcontroller `setup` function:
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
It is generally expected that the method will always return `true` and will not fail. However, to support certain 
runtimes that need to be able to fail, the method supports returning `false` to notify the Kernel that the setup has 
failed. If this method returns `false`, the Kernel will deadlock the microcontroller in the error state until you 
reupload the firmware, as failing setup is considered one of the most severe error states the microcontroller can 
encounter.

### Utility methods
To further simplify implementing new custom hardware modules, the base Module class exposes a number of utility methods.
These methods provide an easy-to-use API for accessing internal attributes and properties of the superclass, which 
further abstracts the inner workings of the class, allowing the module developers to largely treat the library as a
black box.

Note, the list below only provides the names and brief descriptions for each utility method. Use the
[API documentation](https://ataraxis-micro-controller-api-docs.netlify.app/) to get more information about each of 
these methods.

#### GetActiveCommand 
Returns the byte-code of the currently active module command. Primarily, this method should be used to access the 
active command code when implementing the `RunActiveCommand` virtual method.

#### AbortCommand 
Aborts the currently active command and, if it was queued to run again, clears it from queue. Use this method if your
command logic runs into an error to immediately end its execution and ensure it is not executed again.

#### ResetStageTimer
Resets the internal timer used to delay module command execution. This method resets the timer used by `WaitForMicros` 
method (see below). It is called automatically as part of the `AdvanceCommandStage` method. We strongly advise not 
calling this method directly and to instead segregate each delay into a separate command stage, as showcased in our
[TestModule](./examples/example_module.h) implementation.

#### AdvanceCommandStage
Increments the stage of the currently active command by one and resets the stage delay timer. This method has to be 
called at the end of each multi-stage command to advance the stages. Failure to call this method may result in the 
module or the whole controller getting stuck with infinitely executing the same command stage.

#### GetCommandStage
Returns the current stage number of the executed (active) command. Use this method when writing multi-stage command 
logic to segregate the logic for each stage into separate blocks. See [TestModule](./examples/example_module.h) 
implementation for examples.

#### CompleteCommand
Notifies the PC that the command has been completed and resets the active command tracker. It is essential to call this
method when the command reaches its end point to notify the Kernel that the module has completed the command and is 
ready to execute the next queued command. Failure to call this method may result in the module or the whole controller 
getting stuck with infinitely executing the same command stage.

#### AnalogRead
Reads the value of the specified analog input pin. The method supports averaging multiple pin readouts to produce
the final pin value. It returns the detected value in 'analog units' of the microcontroller, which depend on the 
Analog-to-Digital-Converter resolution. Essentially, this method is the same as `analogRead` with an optional averaging 
mechanism. It is safe to use `analogRead` directly if you do not need the averaging mechanism.

#### DigitalRead
Reads the value of the specified digital pin. This method functions similar to AnalogRead and can also average multiple 
pin readouts. The method internally uses an efficient `digitalReadFast` library to speed up accessing the digital pin 
state.

#### WaitForMicros
Delays further command execution for the requested number of microseconds. This method can operate in two modes, 
**blocking** and **non-blocking**. The blocking mode behaves identical to the microsecond-precise `delay` method.
The non-blocking mode works by checking whether the requested delay has passed since the last command stage timer 
reset, which is done through `AdvanceCommandStage` or `ResetStageTimer` methods. If the delay has passed, the 
method returns `true` and, if not, `false`. This method should be used to delay code execution in noblock-compatible 
module commands to allow the Kernel to execute other modules’ commands while delaying. See 
[TestModule](./examples/example_module.h) for an example of using this utility method to enable non-blocking execution.

#### AnalogWrite
Sets the specified analog output pin to deliver a signal modulated by the input 8-bit duty-cycle. The method respects
the global microcontroller output pin lock, which is set by the Kernel. The lock is used as an additional safety feature
that makes it impossible to change the state of output pins until the lock is disabled by the PC. Essentially, this 
method is the same as `analogWrite`, with an additional guard that prevents writing to a locked pin. The method returns
`true` if it was able to change the state of the pin and `false` if the pin is locked.

#### DigitalWrite
Sets the specified analog output pin to the specified level (HIGH or LOW). This method works similar to the 
`AnalogWrite`, except that it works with digital signals. It also contains the mechanism that prevents changing the 
state of a locked pin. Internally, it uses an efficient `digitalReadFast` library to speed up changing the pin state.

#### SendData
Packages and sends the input data to the PC. There are two versions for this method accessible via overloading. The 
first version only takes the 8-bit `state code` and is specialized for communicating module states. The second version 
**also** takes in an 8-bit `prototype code` and a `data object` specified by that prototype. 

Currently, we support sending scalars and arrays of up to 15 elements, made up of all supported scalar types: bool, 
uint8-64, int8-64, float32-64. Generally, this range of supported objects should be enough for most conceivable 
use cases. Note, not all supported prototypes may be available on lower-end microcontrollers, as they may be too
large to fit inside the serial buffer of the microcontroller.

See [TestModule](./examples/example_module.h) for the demonstration on how to use both versions.
___

## API Documentation

See the [API documentation](https://ataraxis-micro-controller-api-docs.netlify.app/) for the detailed description of
the methods and classes exposed by components of this library.
___

## Developers

This section provides installation, dependency, and build-system instructions for the developers that want to
modify the source code of this library.

### Installing the library

1. If you do not already have it installed, install [Platformio](https://platformio.org/install/integration) either as
   a standalone IDE or as a plugin for your main C++ IDE. As part of this process, you may need to install a standalone
   version of [Python](https://www.python.org/downloads/).
2. Download this repository to your local machine using your preferred method, such as git-cloning. If necessary, unpack
   and move the project directory to the appropriate location on your system.
3. ```cd``` to the root directory of the project using your command line interface of choice. Make sure it contains
   the `platformio.ini` file.
4. Run ```pio project init ``` to initialize the project on your local machine. Provide additional flags to this command
   as needed to properly configure the project for your specific needs. See
   [Platformio API documentation](https://docs.platformio.org/en/latest/core/userguide/project/cmd_init.html) for
   supported flags.

***Warning!*** If you are developing for a platform or architecture that the project is not explicitly configured for,
you will first need to edit the platformio.ini file to support your target microcontroller by configuring a new
environment. This project comes preconfigured with `teensy 4.1`, `arduino due` and `arduino mega (R3)` support.

### Additional Dependencies

In addition to installing platformio and main project dependencies, install the following dependencies:

- [Tox](https://tox.wiki/en/stable/user_guide.html), if you intend to use preconfigured tox-based project automation.
  Currently, this is used only to build API documentation from source code docstrings.
- [Doxygen](https://www.doxygen.nl/manual/install.html), if you want to generate C++ code documentation.

### Development Automation

Unlike other Ataraxis libraries, the automation for this library is primarily provided via
[Platformio’s command line interface (cli)](https://docs.platformio.org/en/latest/core/userguide/index.html) core.
Additionally, we also use [tox](https://tox.wiki/en/latest/user_guide.html) for certain automation tasks not directly
covered by platformio, such as API documentation generation. Check [tox.ini file](tox.ini) for details about
available pipelines and their implementation. Alternatively, call ```tox list``` from the root directory of the project
to see the list of available tasks.

**Note!** All pull requests for this project have to successfully complete the `tox`, `pio check` and `pio test` tasks
before being submitted.

---

## Versioning

We use [semantic versioning](https://semver.org/) for this project. For the versions available, see the
[tags on this repository](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller/tags).

---

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))
- Jasmine Si
---

## License

This project is licensed under the GPL3 License: see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- All [Sun Lab](https://neuroai.github.io/sunlab/) members for providing the inspiration and comments during the
  development of this library.
- The creators of all other projects used in our development automation pipelines [see tox.ini](tox.ini) and in our
  source code [see platformio.ini](platformio.ini).
---
