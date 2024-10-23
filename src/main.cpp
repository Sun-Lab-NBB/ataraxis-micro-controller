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
#include "kernel.h"
#include "module.h"
#include "modules/ttl_module.h"
#include "shared_assets.h"

// Pre-initializes global assets
shared_assets::DynamicRuntimeParameters DynamicRuntimeParameters;  // Shared controller-wide parameters
constexpr uint8_t kControllerID = 101;                             // Unique ID for the controller

// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
Communication axmc_communication(Serial);  // Shared class that manages all incoming and outgoing communications

// Instantiates module classes. Each module class manages a specific type and instance of physical hardware, e.g.:
// a treadmill motor.
TTLModule<1, 2> mesoscope_ttl(4, 5, axmc_communication, DynamicRuntimeParameters);

// Packages all modules into an array to be managed by the Kernel class.
Module* modules[] = {&mesoscope_ttl};

// Instantiates the Kernel class using the assets instantiated above.
Kernel axmc_kernel(kControllerID, axmc_communication, DynamicRuntimeParameters, modules);

// This function is only executed once for each power cycle. For most Ataraxis runtimes, only 3 major setup functions
// need to be executed here.
void setup()
{
    Serial.begin(115200);  // Initializes the serial port at 115200 bauds, the baudrate is ignored for teensy boards.
    // Sets ADC resolution to 12 bits. Teensies can support 16 bits too, but 12 often produces cleaner readouts.
    analogReadResolution(12);
    axmc_kernel.Setup();  // Carries out the rest of the setup depending on the module configuration.
}

// This function is executed continuously while the controller is powered. Since Kernel manages the runtime, only the
// RuntimeCycle() function needs to be called here.
void loop()
{
    axmc_kernel.RuntimeCycle();
}
