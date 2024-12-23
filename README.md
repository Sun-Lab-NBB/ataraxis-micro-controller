# ataraxis-micro-controller

A C++ library for Arduino and Teensy microcontrollers that integrates user-defined hardware modules with the centralized
communication interface client running on the host-computer (PC).

[![PlatformIO Registry](https://badges.registry.platformio.org/packages/inkaros/library/ataraxis-micro-controller.svg)](https://registry.platformio.org/libraries/inkaros/ataraxis-micro-controller)
![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=fff&style=plastic)
![license](https://img.shields.io/badge/license-GPLv3-blue)
___

## Detailed Description

This library is designed to simplify the integration of custom hardware, managed by Arduino or Teensy microcontrollers,
with existing project Ataraxis libraries and infrastructure running on the host-computer (PC). To do so, it exposes 
classes that abstract microcontroller-PC communication and microcontroller runtime management (task scheduling, error 
handling, etc.). This library allows the developers to focus on implementing their custom hardware logic, instead of 
worrying about cross-platform communication and multi-module runtime flow control.

This library is designed as a companion for the host-computer (PC) 
[interface library](https://github.com/Sun-Lab-NBB/ataraxis-communication-interface) written in Python. To enable the 
interface to control any custom hardware module, this library defines a shared API that can be integrated into 
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
3. Add `include <kernel.h>`, `include <communication.h>`, `include <axmc_shared_assets.h>` at teh top of the main.cpp 
   file and `include <module.h>`, `include <axmc_shared_assets.h>` at the top of each custom hardware module header 
   file.

### Platformio

1. Navigate to your platformio.ini file and add the following line to your target environment specification:
   ```lib_deps = inkaros/ataraxis-micro-controller@^1.0.0```. If you already have lib_deps specification, add the
   library specification to the existing list of used libraries.
2. Add `include <kernel.h>`, `include <communication.h>`, `include <axmc_shared_assets.h>` at teh top of the main.cpp
   file and `include <module.h>`, `include <axmc_shared_assets.h>` at the top of each custom hardware module header 
   file.
___

## Usage

### Communication

The Communication class provides a high-level interface for bidirectional communication between a microcontroller
(e.g., Arduino, Teensy) and a PC over a serial USB or UART port. To do so, it wraps the **TransportLayer** class and
uses it to serialize and transmit data using one of the predefined message layout protocols to determine payload
microstructure.

This class is tightly integrated with the Kernel and (base) Module classes, together forming the 'Core Triad' of the
AtaraxisMicroController library. Therefore, it is specifically optimized to transfer data between Kernel, Modules, and
their PC interfaces.

#### Prototypes
Prototypes are byte-codes packaged in Message packets that specify the data structure object that can be used to parse
the included object data.Currently, there are only six prototype codes supported, which define the types of data that can
be transmitted. All messages sent must conform to one of the supported prototype codes. The Prototype codes and the types 
of objects they encode are available from **communication_assets**. For prototype codes to work as expected, the microcontroller
and the PC need to share the same `prototype_code` to object mapping. \
Here’s an example of sending a message using a prototype code that specifies a  `TwoUnsignedBytes` object:
```
//TwoUnsignedBytes
uint8_t test_object[2] = {0}; 

const uint8_t module_type = 112;       // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const communication_assets::kPrototypes prototype = communication_assets::kPrototypes::kTwoUnsignedBytes;

comm_class.SendDataMessage(command, event_code, prototype, test_object);
```

#### Outgoing Message Structures

The Communication class supports sending a fixed number of predefined message payload structures, each optimized for
a specific use case.

Each message type is associated with a unique message protocol code, which is used to instruct the receiver on how to
parse the message. The Communication class automatically handles the parsing and serialization of these message formats.

- **ModuleData:** Communicates the event state-code of the sender Module and includes an additional data object.\
  Use `SendDataMessage` to send Module messages to the PC. This method is specialized to send Module messages. 

```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t module_type = 112        // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const uint8_t prototype = 1;           // Prototype for OneUnsignedByte. Protocol codes are available from
                                       // communication_assets namespace.
const uint8_t test_object = 1;  // OneUnsignedByte

comm.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object );
```

- **KernelData:** Similar to ModuleData, but used for messages sent by the Kernel.
  There is an overloaded version of `SendDataMessage` that does not take `module_type` and `module_id` arguments,
  which allows sending data messages from the Kernel class.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const uint8_t prototype = 1;           // Prototype for OneUnsignedByte. Protocol codes are available from
                                       // communication_assets namespace.
const uint8_t test_object = 1;  // OneUnsignedByte

comm.SendDataMessage(command, event_code, prototype, test_object );
```

- **ModuleState:** Used for sending event state codes from modules without additional data.\
  Use `SendStateMessage` to send Module states to the PC. This method is specialized to send Module messages. It
  packages the input data into the ModuleState structure and sends it to the connected PC.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t module_type = 112        // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code

comm_class.SendStateMessage(module_type, module_id, command, event_code);
```

- **KernelState:** Similar to ModuleState, but used for Kernel messages.\
  There is an overloaded version of  `SendStateMessage` that only takes `command` and `event_code` arguments, which
  allows sending data messages from the Kernel class.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code

comm_class.SendStateMessage(command, event_code);
```

#### Incoming Message Structures
When receiving incoming messages, there are two key functions to keep in mind:
- `ExtractModuleParameters()` extracts the parameter data transmitted with the ModuleParameters message and uses it to set
the input structure values. This method will fail if it is called for any other message type, including KernelParameters 
message.
- The method `ReceiveMessage()` parses the next available message received from the PC and stored inside the serial port 
reception buffer. If the received message is a ModuleParameters message, call `ExtractModuleParameters()` method to
finalize message parsing since `ReceiveMessage()` DOES NOT extract Module parameter data from the received message.

```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

comm_class.ReceviveMessage(); 

struct DataMessage
{
uint8_t id = 1;
uint8_t data = 10;
} data_message;

bool success = comm_class.ExtractParameters(data_message);
```

#### Commands
This class supports various command types for controlling the behavior of modules and the Kernel. These commands
are sent through this class and are specified within different message types (e.g., **ModuleData**, **KernelData**). 
Each command typically contains a **command code**, which is a unique identifier for the operation to perform. Commands
can also include **return codes** to notify the sender that the command was received and processed successfully.

- **Module Commands**: These commands are sent to specific modules to perform certain actions. There are three main types:
    - **RepeatedModuleCommand**: A command that runs repeatedly or in cycles. It allows controlling module behavior
      on a timed interval.
    ```
    Communication comm_class(Serial);  // Instantiates the Communication class.
    Serial.begin(9600);  // Initializes serial interface.

    struct DataMessage
    {
    uint8_t id = 1;
    uint8_t data = 10;
    } data_message;

    comm_class.ReceviveMessage(); 
    comm_class.ExtractParameters(data_message);
    
    uint8_t module_type = static_cast<uint8_t>(comm_class.repeated_module_command.module_type);  // Extract module_type
    uint8_t module_id = static_cast<uint8_t>(comm_class.repeated_module_command.module_id);      // Extract module_id
    uint8_t return_code = static_cast<uint8_t>( comm_class.repeated_module_command.return_code); // Extract return_code
    uint8_t command = static_cast<uint8_t>(comm_class.repeated_module_command.command);          // Extract command
    bool noblock = static_cast<uint8_t>(comm_class.repeated_module_command.noblock);             // Extract noblock
    uint32_t cycle_delay = static_cast<uint32_t>(comm_class.repeated_module_command.cycle_delay);// Extract cycle_delay
    ```

    - **OneOffModuleCommand**: A single execution command that instructs the addressed Module to run the specified command
      exactly once (non-recurrently).
    ```
    Communication comm_class(Serial);  // Instantiates the Communication class.
    Serial.begin(9600);  // Initializes serial interface.

    struct DataMessage
    {
    uint8_t id = 1;
    uint8_t data = 10;
    } data_message;
    
    comm_class.ReceviveMessage(); 
    comm_class.ExtractParameters(data_message);
    
    uint8_t module_type = static_cast<uint8_t>(comm_class.one_off_module_command.module_type);  // Extract module_type
    uint8_t module_id = static_cast<uint8_t>(comm_class.one_off_module_command.module_id);      // Extract module_id
    uint8_t return_code = static_cast<uint8_t>( comm_class.one_off_module_command.return_code); // Extract return_code
    uint8_t command = static_cast<uint8_t>(comm_class.one_off_module_command.command);          // Extract command
    bool noblock = static_cast<bool>(comm_class.one_off_module_command.noblock);                // Extract noblock
    ```
    - **DequeModuleCommand**: A command that instructs the addressed Module to clear (empty) its command queue. Note that
      this does not terminate any active commands, and any active commands will eventually be allowed to finish.
    ```
    Communication comm_class(Serial);  // Instantiates the Communication class.
    Serial.begin(9600);  // Initializes serial interface.

    struct DataMessage
    {
    uint8_t id = 1;
    uint8_t data = 10;
    } data_message;
    
    comm_class.ReceviveMessage(); 
    comm_class.ExtractParameters(data_message);
    
    uint8_t module_type = static_cast<uint8_t>(comm_class.module_dequeue.module_type);    // Extract module_type
    uint8_t module_id = static_cast<uint8_t>(comm_class.module_dequeue.module_id);        // Extract module_id
    uint8_t return_code = static_cast<uint8_t>( comm_class.module_dequeue.return_code);   // Extract return_code
    ```

- **Kernel Commands**: These are commands sent to the Kernel to perform system-level operations. These commands are
  always one-off and execute immediately upon receipt.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

struct DataMessage
{
uint8_t id = 1;
uint8_t data = 10;
} data_message;

comm_class.ReceviveMessage(); 
comm_class.ExtractParameters(data_message);

uint8_t return_code = static_cast<uint8_t>( comm_class.module_dequeue.return_code);   // Extract return_code
uint8_t command = static_cast<uint8_t>(comm_class.repeated_module_command.command);   // Extract command
```
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
