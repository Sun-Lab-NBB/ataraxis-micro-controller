/**
 * @file
 * @brief Arena Micro Controller (AXMC) main executable.
 *
 * This executable showcases how to use this library and also provides the default functionality used by the
 * SunLab.
 *
 * @notes This codebase is written to be integrated with the ataraxis-transport-layer PC library.
 *
 * See https://github.com/Sun-Lab-NBB/ataraxis-micro-controller for more details.
 * API documentation: https://ataraxis-micro-controller-api-docs.netlify.app/
 * Author: Ivan Kondratyev (Inkaros).
 */

// Dependencies
#include "Arduino.h"
#include "axmc_shared_assets.h"
#include "communication.h"
#include "kernel.h"
#include "module.h"
#include "modules/break_module.h"
#include "modules/encoder_module.h"
#include "modules/lick_module.h"
#include "modules/torque_module.h"
#include "modules/ttl_module.h"
#include "modules/valve_module.h"

// Pre-initializes global assets
axmc_shared_assets::DynamicRuntimeParameters DynamicRuntimeParameters;  // Shared controller-wide parameters
constexpr uint8_t kControllerID = 123;                                  // Unique ID for the controller

// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
Communication axmc_communication(Serial);  // Shared class that manages all incoming and outgoing communications

// Instantiates module classes. Each module class manages a specific type and instance of physical hardware, e.g.:
// a treadmill motor.
TTLModule<33> mesoscope_frame(1, 1, axmc_communication, DynamicRuntimeParameters);            // Sensor
TTLModule<33> mesoscope_trigger(1, 2, axmc_communication, DynamicRuntimeParameters);          // Actor
EncoderModule<33, 34, 35> wheel_encoder(2, 1, axmc_communication, DynamicRuntimeParameters);  // Encoder
BreakModule<29, false> wheel_break(3, 1, axmc_communication, DynamicRuntimeParameters);       // Actor
LickModule<15> lick_sensor(4, 1, axmc_communication, DynamicRuntimeParameters);               // Sensor
ValveModule<28, true> reward_valve(5, 2, axmc_communication, DynamicRuntimeParameters);       // Actor
TorqueModule<16, 2048> torque_sensor(6, 1, axmc_communication, DynamicRuntimeParameters);     // Sensor

// Packages all modules into an array to be managed by the Kernel class.
Module* modules[] = {&mesoscope_frame, &wheel_break, &reward_valve};

// Instantiates the Kernel class using the assets instantiated above.
Kernel axmc_kernel(kControllerID, axmc_communication, DynamicRuntimeParameters, modules);

// This function is only executed once for each power cycle. For most Ataraxis runtimes, only 3 major setup functions
// need to be executed here.
void setup()
{

#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_NRF52) || defined(CORE_TEENSY)
    analogReadResolution(12);
#endif

    Serial.begin(115200);  // Initializes the serial port at 115200 bauds, the baudrate is ignored for teensy boards.

    axmc_kernel.Setup();  // Carries out the rest of the setup depending on the module configuration.

    DynamicRuntimeParameters.action_lock = false;
    DynamicRuntimeParameters.ttl_lock    = false;
    reward_valve.QueueCommand(2, true);
}

// This function is executed continuously while the controller is powered. Since Kernel manages the runtime, only the
// RuntimeCycle() function needs to be called here.
void loop()
{
    axmc_kernel.RuntimeCycle();
}
