# SerializedTransferProtocol-Microcontroller

The version of the SerializedTransferProtocol (STP) project intended to be used with Microcontroller platforms such as
Arduino and Teensy to enable fast bidirectional communication with other devices over the USB or UART interface, using
serialized payloads.

## Detailed Description

The SerializedTransferProtocol_Microcontroller (STP_M) library is a C++ library that is part of the broader
multiplatform SerializedTransferProtocol (STP) project. The STP project provides a fast and reliable way of serializing
and transferring data between different platforms, targeting realtime applications, such as scientific experiments. This
particular implementation is designed specifically for Microcontroller platforms, such as Arduino and Teensy boards, and
uses the serial USB or UART interface of the board to communicate with the connected device (most commonly the host PC).

## Features

- Supports all Arduino and Teensy architectures and platforms.
- Only minor modifications are needed to add support for more architectures / platforms.
- Uses Consistent Overhead Byte Stuffing (COBS) to encode payloads.
- Supports Circular Redundancy Check (CRC) 8-, 16- and 32-bit polynomials to ensure data integrity during transmission.
- Fully configurable through Constructor and Template parameters.
- Seamlessly integrates with other STP project modules, simplifying cross-platform communication.
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

- An IDE or Framework capable of uploading microcontroller software. Use [Platformio](https://platformio.org/install) if
  you want to use the project the way it was built, tested and used by the authors. Overall, it is highly encouraged to
  at
  least give this framework a try, as it greatly simplifies working with embedded projects.
  Alternatively, [Arduino IDE](https://www.arduino.cc/en/software) is also fully compatible with this project and
  satisfies this dependency.

- [Python](https://www.python.org/). This is only required for users that want to use the Platformio framework.
  Platformio is known to specifically require the standalone version of Python to be installed to work properly, so it
  is
  highly advised to install the latest stable Python version in addition to any environment manager you may already
  have.

For users, all other library dependencies are installed automatically for all supported installation methods (see
[Installation](#Installation) section). For developers, see the [Developers](#Developers) section for information on
installing additional development dependencies.

## Installation

### Source

Note, installation from source is ***highly discouraged*** for everyone who is not an active project developer.
Developers should see the [Developers](#Developers) section for more details on installing from source. The instructions
below assume you are ***not*** a developer and additionally only cover source installation with Platformio, as this is
the only framework that may benefit from installation from source.

1. Download this repository to your local machine using your preferred method, such as Git-cloning.
2. Remove the 'main.cpp' file from the 'src' directory of the project.
3. Move all remaining 'src' directory contents into the appropriate destination ('include,' 'src' or 'libs') of your
   project.
4. Add ```include <serialized_transfer_protocol.h>``` to the top of the file(s) to access the STP API.

### Platformio

Best method:

1. Navigate to your platformio.ini file and add the following line to your target environment specification:
   ```lib_deps = inkaros/SerializedTransferProtocol_Microcontroller@^1.0.0```. If you already have lib_deps
   specification,
   add the library specification to the existing list of used libraries.
2. Add ```include <serialized_transfer_protocol.h>``` to the top of the file(s) to access the STP API.

Alternatively, you can use the following method to add the library via platformio GUI:

1. Use the following command from your CLI to navigate to platformio home page ```pio home```.
2. In the left-hand navigation panel, select the 'Libraries' menu.
3. Use the navigation bar of the 'Libraries' page to search for ```SerializedTransferProtocol_Microcontroller```.
4. Click on the discovered library and in the library window click ```Add to project``` button. Make sure the library is
   listed under 'inkaros' account (by Ivan Kondratyev).
5. Select the target project and let platformio do the necessary .ini file modifications for you.
6. Add ```include <serialized_transfer_protocol.h>``` to the top of the file(s) to access the STP API.

### Arduino IDE

Install this library using ArduinoIDE GUI:

1. Navigate to the Library Manager menu using the left-hand panel.
2. Use the search bar to search for ```SerializedTransferProtocol_Microcontroller```.
3. Select the desired library version and click on the ```Install``` button. Make sure the library is listed under
   'Inkaros' account (by Ivan Kondratyev).
4. Add ```include <serialized_transfer_protocol.h>``` to the top of the file(s) to access the STP API.

## Usage
This is a minimal example of how to use this library. See the [examples](./examples) folder for .cpp implementations:

```
// Note, this example should run on both Arduino and Teensy boards.

// First, include the main STP class to access its' API.
include <serialized_transfer_protocol.h>

// Next, initialize the STP class. Use the tempalte and constructor arguments to configure the class for your specific 
// project (see below)

// Maximum outgoing payload size, in bytes. Cannot exceed 254 bytes due to COBS encoding.
uint8_t maximum_tx_payload_size = 254;

// Maximum incoming payload size, in bytes. Cannot exceed 254 bytes. Note, due to the use of a circular reception
// buffer by the Serial class, it is generally recomended to set this to a value below the maximum size of the reception
// buffer. Additionally, it is a good idea to reserve ~10 bytes for protocol variables. So, if your board's Serial 
// reception buffer is capped at 64 bytes and you are using a very fast baudrate, it is usually a good idea to cap this 
// at 50 bytes and ensure all of the payloads sent to the microcontroller do not exceed 50 bytes.
uint8_t maximum_rx_payload_size = 200;

// The minimal incoming payload size. This is used to optimzie class behavior by avoiding costly data reception 
// operations unless enough bytes are available for reading to potentially produce the payload of the minimal valid 
// size. This is generally not relevant for Microcontrollers, but critically important for PC platforms that have
// a lot of overhead associated with I/O operations on some OSes (mostly Windows).
uint8_t minimum_payload_size = 1;

// These parameters jointly specify the CRC algorithm to be used fro the CRC calcualtion. The class automatically scales
// to work for 8-, 16- and 32-bit algorithms. Currently, only non-reversed polynomials are supported.
uint16_t polynomial = 0x1021;
uint16_t initial_value = 0xFFFF;
uint16_t final_xor_value = 0x0000;

// The value used to mark the beginning of transmitted packets in the byte-stream. Generally, it is a good idea to set
// this to a value that is unlikely to be produced by the random communication line noise.
uint8_t start_byte = 129;

// The value used to mark the end of the main packet section. Generally, it is advised to keep this as 0, but the only
// major requirement is that the delimiter byte has to be different from the start byte.
uint8_t delimiter_byte = 0;

// The number of microseconds that can separate the reception of any two consecutive bytes of the same packet. This is 
// used to detect stale transmissions as the class is configured to transmit the entire packet as a single chunk of data
// with almost no delay between the bytes of the packet. This is indirectly affected by the baudrate and may need to be
// increased for slow baudrates.
uint32_t timeout = 20000; // In microseconds

// The reference to the Serial interface class. This can either be Serial or USBSerial for boards that have native USB
// interface in addition to the UART interface.
serial = Serial;

// Instantiates a new SerializedTransferProtocol object. Note, the firtst tempalte parameter is the datatype of the 
// CRC configuration variables. It has to match the type used to specify the CRC algorthm parameters.
SerializedTransferProtocol<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size, minimum_payload_size> protocol(
    serial,
    polynomial,
    initial_value,
    final_xor_value,
    start_byte,
    delimiter_byte,
    minimum_payload_size,
    timeout
);

void setup()
{
    Serial.begin(115200);  // Opens the Serial port, initiating PC communication
}

// Communcates with the PC. Note, for this example to work properly, there has to be a PC client running the STP project
// specifcially connected to this microcontroller's port. Also, this specific example uses a aimple 4-byte array, but 
// the STP class should be able to serialize and deserialize any microcontroller-recognized C++ type.
void loop()
{
    // Checks if data is available for reception.
    if (protocol.Available())
    {
        // If the data is available, carries out the reception procedure (acually receives the byte-stream, parses the 
        // payload and makes it available for reading).
        bool data_received = protocol.ReceiveData();
        
        // Provided that the reception was successful, reads the data, assumed to be the test array object
        if (data_received)
        {
            // Instantiates a simple test object
            uint8_t test_data[4] = {0, 0, 0, 0};
            
            // Reads the received data into the array object. Assuming the received data was [1, 2, 3, 4] the test_data
            // should now be set to these values. Note, the method returns the index inside the payload region that 
            // immediately follows the read data section. This is not relevant for this example, but can be used for 
            // chained read calls.
            uint16_t next_index = protocol.ReadData(test_data);
            
            // Instantiates a new object to send back to PC.
            uint8_t send_data[4] = {5, 6, 7, 8};
            
            // Writes the object to the transmission buffer, staging it to be sent witht he next SendData() command. 
            // This method also returns the index that immediately follows the portion of the buffer that the input 
            // object's data was written to.
            uint16_t add_index = protocol.WriteData(send_data, 0);
            
            // This showcases a chained addition, where test_data is staged right after send_data.
            add_index = protocol.WriteData(test_data, add_index);
            
            // Packages and sends the contents of the class transmission buffer that were written above to the PC.
            bool data_sent= protocol.SendData();  // This also returns a boolean status.
        }
    }
}
```

See the API documentation[LINK STUB] for the detailed description of the project API and more use examples.

### Communication
The Communication class a high-level interface for bidirectional communication between a microcontroller 
(e.g., Arduino, Teensy) and a PC over Serial or USB. It extends the **TransportLayer** class, handling message
serialization, error checking, and communication protocols.


#### Message Types
The Communication class supports various message types, each with a specific use case:

- **ModuleData:** Communicates the event state-code of the sender Module and includes an additional data object.
- **KernelData:** Similar to ModuleData, but used for messages sent by the Kernel.
- **ModuleState:** Used for sending event state codes from modules without additional data.
- **KernelState:** Similar to ModuleState, but used for Kernel messages.

Each message type has an associated protocol code to guide how the message is parsed. The Communication class 
automatically handles the parsing and serialization of these message formats

#### Commands Overview

This class supports various command types for controlling the behavior of modules and the Kernel. These commands
are sent through the this class and are specified within different message types (e.g., **ModuleData**, **KernelData**).

- **Module Commands**: These commands are sent to specific modules to perform certain actions. There are two main types:
  - **RepeatedModuleCommand**: A command that runs repeatedly or in cycles. It allows you to control module behavior on
    a timed interval.
  - **OneOffModuleCommand**: A single execution command that runs once and completes before returning control.

- **Kernel Commands**: These are commands sent to the Kernel to perform system-level operations. These commands are
  typically one-off and execute immediately upon receipt.

Each command typically contains a **command code**, which is a unique identifier for the operation to perform. Commands 
can also include **return codes** to notify the sender that the command was received and processed successfully.


#### Send Data

Use `SendDataMessage` to send data messages to the PC.

```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t module_type = 112        // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const uint8_t prototype = 1;           // Prototype codes are available from communication_assets namespace.
const uint8_t placeholder_object = 0;  // Meaningless, placeholder object

comm.SendDataMessage(module_type, module_id, command, event_code, prototype, placeholder_object);
```

#### Receive Data

Use `ReceiveMessage` to check for incoming data and automatically parse it.

```
bool ReceiveMessage();
```
#### Extract Parameters

For **ModuleParameters** messages, use `ExtractModuleParameters` to parse and extract parameter data.

```
template <typename ObjectType>
bool ExtractModuleParameters(ObjectType& prototype, const uint16_t& bytes_to_read = sizeof(ObjectType));
```

## Developers

This section provides additional installation, dependency and build-system instructions for the developers that want to
modify the source code of this library.

### Installing the library

1. If you do not already have it installed, install [Platformio](https://platformio.org/install/integration) either as 
a standalone IDE or as a plugin for your main C++ IDE. As part of this process, you may need to install a standalone 
version of [Python](https://www.python.org/downloads/). Note, for the rest of this guide, installing platformio CLI is 
sufficient.
2. Download this repository to your local machine using your preferred method, such as git-cloning.
3. ```cd``` to the root directory of the project using your CLI of choice.
4. Run ```pio --target upload -e ENVNAME``` command to compile and upload the library to your microcontroller. Replace 
the ```ENVNAME``` with the environment name for your board. Currently, the project is preconfigured to work for 
```mega```, ```teensy41``` and ```due``` environments.
5. Optionally, run ```pio test -e ENVNAME``` command using the appropriate environment to test the library on your 
target platform

Note, if you are developing for a board that the project is not explicitly configured for, you will first need to edit
the platformio.ini file to support your target microcontroller by configuring a new environment.

### Additional Dependencies

In addition to installing platformio and main project dependencies, additionally install the following dependencies:

- [Tox](https://tox.wiki/en/4.15.0/user_guide.html), if you intend to use preconfigured tox-based project automation.
- [Doxygen](https://www.doxygen.nl/manual/install.html), if you want to generate C++ code documentation. Note, if you 
are not installing Tox, you will also need [Breathe](https://breathe.readthedocs.io/en/latest/) and 
[Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html).

### Development Automation

To assist developers, this project comes with a set of fully configured 'tox'-based pipelines for verifying and building
the project. Each of the tox commands builds the necessary project dependencies in the isolated environment prior to 
carrying out its tasks.

Below is a list of all available commands and their purpose:

- ```tox -e test-ENVNAME``` Builds the project and executes the tests stored in the /test directory using 'Unity' test 
framework. Note, replace the ```ENVNAME``` with the name of the tested environment. By default, the tox is configured to
run tests for 'teensy41,' 'mega' and 'due' platforms. To add different environments, edit the tox.ini file.
- ```tox -e docs``` Uses Doxygen, Breathe and Sphinx to build the source code documentation from Doxygen-formatted 
docstrings, rendering a static API .html file.
- ```tox -e build-ENVNAME``` Builds the project for the specified environment (platform). Does not upload the built hex
file to the board. Same ```ENVNAME``` directions apply as to the 'test' command.
- ```tox -e upload-ENVNAME``` Builds the project for the specified environment (platform) and uploads it to the 
connected board. Same ```ENVNAME``` directions apply as to the 'test' command.
- ```tox --parallel``` Carries out all commands listed above in-parallel (where possible). Remove the '--parallel'
argument to run the commands sequentially. Note, this command will test, build and upload the library for all 
development platforms, which currently includes: 'teensy41,' 'mega' and 'due.'

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))

## License

This project is licensed under the GPL3 License: see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- All Sun Lab [WEBSITE LINK STUB] members for providing the inspiration and comments during the development of this
  library.
- My NBB Cohort for answering 'random questions' pertaining to the desired library functionality.
- [PowerBroker2](https://github.com/PowerBroker2) and his 
[SerialTransfer](https://github.com/PowerBroker2/SerialTransfer) for inspiring this library and serving as an example 
and benchmark. This library is in some ways similar and in other ways different from his library, and it is 
highly encouraged to check SerialTransfer as a good alternative with non-overlapping functionality that may be better 
for your project.
