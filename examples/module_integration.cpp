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
#include "communication.h"
#include "kernel.h"
#include "module.h"

// Specifies the unique identifier for the test microcontroller
constexpr uint8_t kControllerID = 222;

// Keepalive interval in milliseconds. If the keepalive interval is greater than 0, the Kernel expects the PC to send
// keepalive messages at that interval. If the Kernel does not receive a keepalive message in time, it assumes that the
// microcontroller-PC communication has been lost and resets the microcontroller, aborting the runtime.
constexpr uint32_t kKeepaliveInterval = 5000;  // Sets the keepalive interval to 5 seconds.

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
Kernel axmc_kernel(kControllerID, axmc_communication,  modules, kKeepaliveInterval);

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
