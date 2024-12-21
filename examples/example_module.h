// The header file showcasing the implementation of an AXMC-compatible hardware module.

// This file demonstrates the proces of writing custom hardware module classes that use the AXMC library to integrate
// with the centralized communication interface running on the host-computer (PC). The module is also used to test the
// runtime behavior of Kernel and Module classes during library development.

// This implementation showcases one of the many possible module design patterns. The main advantage of the AXMC
// library is that it is designed to work with any class design and layout, as long as it subclasses the base Module
// class and overloads the three pure virtual methods: SetCustomParameters, RunActiveCommand and SetupModule.

// The module uses LED pin manipulation to demonstrate the use of the AXMC library. The LED pin is typically RESERVED
// for Kernel / Communication class uses and should NEVER be accessed by custom hardware modules. This class is written
// solely for demonstration purposes and violates that rule. During production runtimes, the LED is used to visually
// communicate communication errors that would otherwise be impossible to detect or debug.

// This example module is designed to be used together with a companion TestModuleInterface class defined in the
// ataraxis-communication-interface library:
// https://github.com/Sun-Lab-NBB/ataraxis-communication-interface#quickstart.
// See https://github.com/Sun-Lab-NBB/ataraxis-micro-controller for more details.
// API documentation: https://ataraxis-micro-controller-api-docs.netlify.app/.
// Authors: Ivan Kondratyev (Inkaros), Jasmine Si.

// Inclusion guards to prevent multiple inclusions
#ifndef AXMC_SENSOR_MODULE_H
#define AXMC_SENSOR_MODULE_H

// Imports the Arduino framework as the basis for all Teensy and Arduino projects.
#include <Arduino.h>

// Includes the necessary AXMC assets.
#include "axmc_shared_assets.h"
#include "module.h"

// Defines the TestModule class by subclassing the base Module class. Declares the class final, as we do not intend to
// subclass it any further. The TestModule interfaces with the built-in LED of the microcontroller and showcases the
// implementation of a popular 'Blink' example with AXMC support.
class TestModule final : public Module
{
    public:
        // Defines a structure to store PC-addressable runtime parameters. The companion ModulInterface class uses
        // ModuleParameters messages to dynamically change the values of the parameters stored in this structure and
        // adjust class behavior. Note, the structure format is chosen for convenience, the class supports all valid
        // object types as parameters' container.
        struct CustomRuntimeParameters
        {
                uint32_t on_duration  = 2000000;  // The time, in microseconds, the LED is ON when blinking.
                uint32_t off_duration = 2000000;  // The time, in microseconds, the LED is OFF when blinking.
                uint16_t echo_value   = 666;      // The value that is sent to the PC in during Echo() command runtime.
        } PACKED_STRUCT parameters;

        // As a minimum, the class constructor has to accept 4 arguments and pass them to the superclass (Module)
        // constructor. See the module_integration.cpp for notes on how to initialize the module class using this
        // constructor. As long as the initializer includes these 4 arguments and superclass initialization, you are
        // free to modify it as you see fit.
        TestModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const axmc_shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters)
        {}

        // This method is a wrapper around the ExtractModuleParameters() method call inherited from the base Module
        // class. The Kernel calls it when it receives a message with new runtime parameters addressed to this module.
        bool SetCustomParameters() override
        {
            // This overwrites the memory of the 'parameters' structure with new data.
            return _communication.ExtractModuleParameters(parameters);
        }

        // This method is used to execute specific module commands (methods), depending on the command code received
        // from the PC
        bool RunActiveCommand() override
        {
            // This is the most efficient implementation for this method. It accesses the currently active command (if
            // any) and executes the associated class method. During runtime, the Kernel class handles command
            // activation and cyclically calls this method to execute the activated command(s).
            switch (GetActiveCommand())
            {
                // Blinks the LED
                case 1:
                    Blink();
                    return true;  // Note, the method HAS to return true if it recognized the command.
                // Unrecognized command. It is essential to return false if the command is not recognized.
                default: return false;
            }
        }

        // This method is called by the Kernel as part of the setup() loop and when it is instructed to reset the
        // microcontroller. Use it to initialize module hardware and set runtime parameters to default values.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinMode(LED_BUILTIN, OUTPUT);
            digitalWrite(LED_BUILTIN, LOW);  // Turns the LED off by default.

            // Note, there is 1000 microseconds in one millisecond and 1000000 microseconds in one second.
            parameters.on_duration  = 2000000;  // Keeps the LED ON for 2 seconds when blinking.
            parameters.off_duration = 2000000;  // Keeps the LED OFF for at least 2 seconds when blinking.

            return true;  // Has to return True to notify the Kernel setup was complete.
        }

        // Also implements a virtual destructor.
        ~TestModule() override = default;

    private:

        // Blinks the built-in LED. This command demonstrates writing 'noblock'-capable commands and using SendData to
        // communicate module states to the PC interface.
        void Blink()
        {
            // One major feature of AXMC runtime control is that it allows concurrent execution of multiple commands.
            // Specifically, while the microcontroller delays the execution of a command for some time, it
            // can run other commands to maximize throughput. While there are trade-offs associated with this use
            // pattern, it may be desirable in certain runtime scenarios. To support the 'noblock' execution, the
            // command haa to implement the 'stage-based' design pattern and use the methods inherited from the base
            // Module class to transition between stages.

            // Stage 1: Turns the LED on.
            if (execution_parameters.stage == 1)
            {
                // This method is part of the base Module API. It incorporates global microcontroller locks used to
                // enable or disable the ability of modules to change pin states. This is a global safety feature.
                if (DigitalWrite(LED_BUILTIN, HIGH, false))
                {
                    // It is essential to call this method to advance command stage after activating the LED. Otherwise,
                    // the microcontroller will get stuck re-executing this stage.
                    AdvanceCommandStage();
                }
                else
                {
                    // If writing to output pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(51);    // Event code 51 == OutputLocked error
                    AbortCommand();  // Aborts the current and all future command executions.
                    return;
                }
            }

            // Stage 2: delays while the LED is kept ON.
            if (execution_parameters.stage == 2)
            {
                // WaitForMicros can either delay in-place (block) or cycle through this check until the delay passes,
                // enabling non-blocking delays.
                if (!WaitForMicros(parameters.on_duration)) return;
                AdvanceCommandStage();
            }

            // Stage 3: disables the LED
            if (execution_parameters.stage == 3)
            {
                if (DigitalWrite(LED_BUILTIN, LOW, false))
                {
                    AdvanceCommandStage();
                }
                else
                {
                    SendData(51);
                    AbortCommand();
                }
            }

            // Stage 4: delays while the LED is kept OFF.
            if (execution_parameters.stage == 2)
            {
                // WaitForMicros can either delay in-place (block) or cycle through this check until the delay passes,
                // enabling non-blocking delays.
                if (!WaitForMicros(parameters.on_duration)) return;

                // Calling this method ends command execution. If it is not called at the end of the last command
                // stage, the microcontroller will get stuck re-executing the last stage of this command.
                CompleteCommand();
            }
        }

        // Sends the current value of the echo_value runtime parameter to the PC. This command demonstrates the use of
        // SendData specialization for sending data objects alongside communicating module states to the PC interface.
        void Echo()
        {
            SendData(52, axmc_communication_assets::kPrototypes::kOneUint16, parameters.echo_value);
            CompleteCommand();
        }
};

#endif  //AXMC_SENSOR_MODULE_H
