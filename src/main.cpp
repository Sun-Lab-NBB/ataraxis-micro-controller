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
#include "Arduino.h"
#include "communication.h"
#include "module.h"
#include "modules/io_communication.h"
#include "kernel.h"
#include "shared_assets.h"

shared_assets:: DynamicRuntimeParameters DynamicRuntimeParameters;

constexpr shared_assets::StaticRuntimeParameters kStaticRuntimeParameters = {
    .baudrate = 115200,
    .analog_resolution = 12,
    .idle_message_interval = 1000000,
};

// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
Communication axmc_communication(Serial);

IOCommunication<1, 2> io_instance(4, 5, axmc_communication, DynamicRuntimeParameters);
IOCommunication<1, 2> io_instance_2(6, 7, axmc_communication, DynamicRuntimeParameters);

Module* modules[] = {&io_instance, &io_instance_2};

Kernel<123, 2> kernel_instance(axmc_communication, DynamicRuntimeParameters, modules);

void setup()
{
    Serial.begin(115200);
    kernel_instance.SetupModules();
    // io_instance.QueueCommand(3, true, true, 5000000);
    // io_instance_2.QueueCommand(4,false,false,1000);
}

void loop()
{
    kernel_instance.ReceiveData();
    // kernel_instance.RunCommands();
}
