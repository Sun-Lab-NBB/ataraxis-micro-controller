// Arena Micro Controller (AMC) Main Executable

// General Description
//======================================================================================================================
// AMC controller code is designed to make use of the commonly available DIY circuit boards such as Arduino and Teensy.
// It acts as a low-level physical controller that can be communicated with using either Serial, MQTT or both
// communication protocols. Regardless of the communication protocol, the PC sends binary commands in a predetermined
// format that trigger specific, encapsulated controller behaviors. All low-level logic should preferentially be
// implemented on the controller, and all high-level logic should be implemented in Python. This way, the fast execution
// speeds of controller microcode are matched with the versatility of Python running on faster PC cores.
//======================================================================================================================

// Microcode Architecture
//======================================================================================================================
// The microcode is subdivided into multiple blocks, each corresponding to unique General Mode the controller can
// operate in. Each GM has a special purpose, Rx/Tx data packaging and functions that are specifically tuned to support
// the intended GM function. Many functions are further affected by particular physical implementations they are coupled
// with (specific experimental apparatus designs). Where feasible, the functions are designed to use configurable
// parameters that can be set via Python to support adjusting the controller code to the particular controlled apparatus
// without a manual code rewrite. The benefit of the architecture is the optimization of rx/tx protocols which cuts down
// the communication time between the controller and PC, albeit at the cost of more manual tweaking compared to
// solutions like Firmata protocol. Note, the architecture currently adopts the GM-based controller layout policy and
// Query-based communication standards. This allows for overall code standardization and optimization across all
// controller implementations, despite being less efficient in some particular use cases.

// Currently, the used binary protocol supports 254 unique general modes, 25 commands (1 through 25) for each general
// mode with 5 unique state markers (1 through 5). Overall, this should be sufficient for the vast majority of cases,
// but these parameters can be further increased in the future by changing the protocol transmission variable types.

// Code Version 2.0.0
// This code is primarily designed as a companion for the main runtime written in python and running on the control PC.
// See project https://github.com/Inkaros/Ataraxis for more details.
// Author: Ivan Kondratyev (ik278@cornell.edu).
//======================================================================================================================

// Dependencies
#include <digitalWriteFast.h>
#include "Arduino.h"
#include "transport_layer.h"
#include "communication.h"
#include "kernel.h"
#include "module.h"
#include "stream_mock.h"
/**
 * @struct StaticRuntimeParameters
 * @brief Stores global runtime variables that are unlikely to or are incapable of changing during runtime.
 *
 * These variables broadly alter controller runtime behavior such as the resolution of Analog pins or communication
 * baudrate. Manual in-code modification followed by codebase recompiling and re-uploading is necessary to change the
 * values of these variables.
 *
 * @attention This is a Core structure that is intended to be instantiated inside the main.cpp file and used strictly
 * within that file. It is declared inside the Kernel file for general organization purposes so that it is automatically
 * available to all other Core and Module classes that all import Kernel.
 */
struct StaticRuntimeParameters
{
    /**
     * @brief Bit-resolution of ADC (Analog-to-Digital) readouts.
     *
     * Arduino 32-bit boards support 12, 8-bit boards support 10. Teensy boards support up to 16 in hardware.
     * It is generally advised not to exceed 12/13 bits as higher values are typically made unusable by noise.
     */
    uint8_t analog_resolution = 12;

    /**
     * @brief The baudrate (data transmission rate) of the Serial port used for the PC-Controller communication via
     * SerialTransfer library.
     *
     * @note All Teensy boards  default to the maximum USB rate, which is most likely 480 Mbit/sec, ignoring the
     * baudrate setting. Arduino's have much lower baudrates as they use a UART instead of the USB and the maximum
     * baudrate will depend on the particular board.
     *
     * @attention For systems that actually do use baudrate, the value specified here should be the same as the value
     * used in the python companion code or the PC and Controller will not be able to communicate to each-other.
     */
    uint32_t baudrate = 115200;

    /**
     * @brief Determines whether incoming data start byte detection errors are logged (result in an error message being
     * sent to PC) or not.
     *
     * Generally, it is advised to keep this option set to @b false as start byte detection failures are often a natural
     * byproduct of the algorithm clearing out error (jitter) data that results from physical interference in the
     * transmission environment. This option is primarily designed to assist developers debugging communication issues,
     * generally as a last-resort when all other potential error sources have been ruled out.
     */
    bool log_start_byte_detection_errors = false;
} kStaticRuntimeParameters;

// Instantiates all classes used during runtime

constexpr uint8_t maximum_tx_payload_size = 254;
constexpr uint8_t maximum_rx_payload_size = 200;
constexpr uint16_t polynomial             = 0x1021;
constexpr uint16_t initial_value          = 0xFFFF;
constexpr uint16_t final_xor_value        = 0x0000;
constexpr uint8_t start_byte              = 129;
constexpr uint8_t delimiter_byte          = 0;
constexpr uint8_t minimum_payload_size    = 1;
constexpr uint32_t timeout                = 20000; // In microseconds
bool allow_start_byte_errors    = false;
TransportLayer<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size, minimum_payload_size> tl_class(
     Serial,
     polynomial,
     initial_value,
     final_xor_value,
     start_byte,
     delimiter_byte,
     timeout,
     allow_start_byte_errors
     );

// Bidirectional serial connection interface to PC (Uses SerialTransfer under the hood). Note, this initializes the
// class instances, but to support proper function it has to be started during setup() loop by calling
// BeginCommunication() method.
SerialPCCommunication uplink(false, Serial);

//KERNEL
AMCKernel kernel(uplink);

void setup()
{
    Serial.begin(115200);
}

void loop()
{}
