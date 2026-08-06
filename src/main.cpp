/**
 * @file
 *
 * @brief Provides the main entry point that integrates custom hardware modules with the communication interface running
 * on the companion host-computer (PC).
 *
 * Mirrors the module_integration.cpp example exactly and is excluded from the compiled library. It is kept
 * here to facilitate library development. Runs together with the companion ataraxis-communication-interface library
 * running on the host-computer (PC).
 */

#include <Arduino.h>
#include <communication.h>
#include <kernel.h>
#include <module.h>
#include "../examples/example_module.h"

/// Specifies the unique identifier for the test microcontroller.
static constexpr uint8_t kControllerID = 222;

/// Stores the keepalive interval in milliseconds. A value of 0 disables keepalive monitoring.
static constexpr uint32_t kKeepaliveInterval = 5000;

/// Specifies the digital pin managed by the second TestModule instance.
static constexpr uint8_t kSecondModulePin = 6;

/// Specifies the serial port baud rate, which has to match the monitor_speed configured for the target board in
/// platformio.ini. The value below matches the teensy41 environment, while the due and mega environments use others.
static constexpr uint32_t kSerialBaudRate = 115200;

/// Specifies the resolution, in bits, requested from boards that support an adjustable analog-to-digital converter.
static constexpr uint8_t kAnalogReadResolution = 12;

// Initializes the Communication class. Shares this instance with all other classes and manages incoming and outgoing
// communication with the companion host-computer (PC). The Communication has to be instantiated first.
Communication axmc_communication(Serial);  // NOLINT(*-interfaces-global-init)

// Creates two instances of the TestModule class. The first argument is the module type (family), which is the same (1)
// for both, the second argument is the module ID (instance), which is different. The type and id codes do not have
// any inherent meaning, they are defined by the user and are only used to ensure specific module instances can be
// uniquely addressed during runtime.
TestModule<> test_module_1(1, 1, axmc_communication);

// Also uses the template parameter to override the digital pin controlled by the module instance.
TestModule<kSecondModulePin> test_module_2(1, 2, axmc_communication);

// Packages all module instances into an array to be managed by the Kernel class.
Module* modules[] = {&test_module_1, &test_module_2};

// Instantiates the Kernel class. The Kernel has to be instantiated last.
Kernel axmc_kernel(kControllerID, axmc_communication, modules, kKeepaliveInterval);

/// Runs once at controller startup. Since the Kernel manages the setup for each module, there is no need to set up
/// each module's hardware individually.
void setup()
{
    Serial.begin(kSerialBaudRate);

    // AVR boards (for example, Arduino Mega) have a fixed 10-bit ADC and do not provide analogReadResolution(), so the
    // resolution is configured only for architectures that support an adjustable ADC.
#if !defined(__AVR__)
    analogReadResolution(kAnalogReadResolution);
#endif

    // Sets up the hardware and software for the Kernel and all managed modules.
    axmc_kernel.Setup();
}

/// Runs repeatedly while the microcontroller is powered.
void loop()
{
    // Since the Kernel instance manages the runtime of all modules, the only method that needs to be called
    // here is the RuntimeCycle method.
    axmc_kernel.RuntimeCycle();
}
