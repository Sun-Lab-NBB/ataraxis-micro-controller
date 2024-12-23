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
// is the module type (family), which is teh same (1) for both, the second argument is the module ID (instance), which
// is different. Both type-codes and id-codes are assigned by the user at instantiation.
TestModule<5> test_module_1(1, 1, axmc_communication, DynamicRuntimeParameters);

// Also uses the template to override the digital pin controlled by the module instance from teh default (5) to 6.
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
