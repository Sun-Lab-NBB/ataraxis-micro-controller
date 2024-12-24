// The header file showcasing the implementation of an AXMC-compatible hardware module.

// This file demonstrates the proces of writing custom hardware module classes that use the AXMC library to integrate
// with the communication interface running on the host-computer (PC). The module is also used to test the runtime
// behavior of Kernel and Module classes during library development.

// This implementation showcases one of the many possible module design patterns. The main advantage of the AXMC
// library is that it is designed to work with any class design and layout, as long as it subclasses the base Module
// class and overloads the three pure virtual methods: SetCustomParameters, RunActiveCommand and SetupModule.

// The module is designed to manipulate a digital output pin as part of its runtime. Make sure the configured pin is not
// connected to any physical hardware that may be triggered by receiving the digital signals emitted by the pin.

// This example module is designed to be used together with a companion TestModuleInterface class defined in the
// ataraxis-communication-interface library: https://github.com/Sun-Lab-NBB/ataraxis-communication-interface#quickstart.
// See https://github.com/Sun-Lab-NBB/ataraxis-micro-controller for more details.
// API documentation: https://ataraxis-micro-controller-api-docs.netlify.app/.
// Authors: Ivan Kondratyev (Inkaros), Jasmine Si.

// Inclusion guards to prevent multiple inclusions
#ifndef AXMC_EXAMPLE_MODULE_H
#define AXMC_EXAMPLE_MODULE_H

// Imports the Arduino framework as the basis for all Teensy and Arduino projects.
#include <Arduino.h>

// Includes the necessary AXMC assets.
#include "axmc_shared_assets.h"
#include "module.h"

// Defines the TestModule class by subclassing the base Module class. Declares the class final, as we do not intend to
// subclass it any further. This class is designed to do two things. First, it sends square digital pulses using the
// managed pin, in response to receiving the 'pulse' command from the PC. Second, it sends the 'echo_value' parameter
// (see below) to the PC in response to the 'echo' command.
template <const uint8_t kPin = 5>
class TestModule final : public Module
{
    public:
        // Defines a structure to store PC-addressable runtime parameters. The PC interface uses 'ModuleParameters'
        // messages to dynamically change the values of the parameters stored in this structure to adjust class
        // behavior. Note, the 'structure' format is chosen for convenience, the class supports all valid
        // object types as parameters' container.
        struct CustomRuntimeParameters
        {
                uint32_t on_duration  = 2000000;  // The time, in microseconds, to keep the pin HIGH when pulsing.
                uint32_t off_duration = 2000000;  // The time, in microseconds, to keep the pin LOW when pulsing.
                uint16_t echo_value   = 666;      // The value that is sent to the PC in during Echo() command runtime.
        } PACKED_STRUCT parameters;

        // This enumeration contains all state (aka: event / status) codes used by the class when communicating with the
        // PC. Each state has to be unique for each given type (family) of modules. You do not have to store your states
        // in an enumeration, but this usually makes it easier to maintain the codebase. Since the base Module class
        // also has state codes, AVOID using values below 51 (0-50) for your state codes. Those values are reserved for
        // the AXTL library. The companion Interface class running on the PC has to contain an identical code-state
        // mapping structure to interpret the messages sent from the microcontroller.
        enum class kStates : uint8_t
        {
            kOutputLocked = 51,  // Unable to change the managed pin state, as the microcontroller is in a locked state.
            kHigh         = 52,  // The managed pin is currently set to output a HIGH signal.
            kLow          = 53,  // The managed pin is currently set to output a LOW signal.
            kEcho         = 54,  // This code is used by messages transmitting the 'echo_value' to the PC.
        };

        // This enumeration contains the codes for the commands that can be executed by the module. It works similar
        // to the kStates enumeration, but has almost no restrictions on the possible values. The only restricted
        // command code is 0, this value is universally reserved across the library to mean the absence of a valid
        // value.
        enum class kCommands : uint8_t
        {
            kPulse = 1,  // Sends a square digital pulse using the managed pin.
            kEcho  = 2,  // Sends the 'echo_value' parameter to the PC.
        };

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
            // This overwrites the memory of the 'parameters' structure with new data, received from the PC.
            return ExtractParameters(parameters);
        }

        // This method is used to execute specific module commands (methods), depending on the command code received
        // from the PC
        bool RunActiveCommand() override
        {
            // Accesses the currently active command (if any) and executes the associated class method. During runtime,
            // the Kernel class handles command activation and cyclically calls this method to execute the activated
            // command(s).
            switch (static_cast<kCommands>(GetActiveCommand()))  // Casts the command code to the enumeration type.
            {
                // Emits a square pulse via the managed digital pin.
                case kCommands::kPulse:
                    Pulse();
                    return true;  // This means the method has recognized the command, NOT that the command succeeded.

                // Packages and sends the current value of the echo_parameter to the PC.
                case kCommands::kEcho: Echo(); return true;

                // Unrecognized command. It is essential to return 'false' if the command is not recognized.
                default: return false;
            }
        }

        // This method is called by the Kernel from the setup() function and when it is instructed to reset the
        // microcontroller. Use it to initialize module hardware and set runtime parameters to default values.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinMode(kPin, OUTPUT);
            digitalWrite(kPin, LOW);  // Sets the pin to LOW by default.

            // Note, there is 1000 microseconds in one millisecond and 1000000 microseconds in one second.
            parameters.on_duration  = 2000000;  // The HIGH phase of the pin is 2 seconds by default.
            parameters.off_duration = 2000000;  // Ensures the LOW phase of the pin is also at least 2 seconds.
            parameters.echo_value   = 123;      // Sets the default echo value.

            return true;  // Has to return True to notify the Kernel setup was complete.
        }

        // Also implements a virtual destructor.
        ~TestModule() override = default;

    private:

        // Emits a square digital pulse using the managed pin. This command demonstrates writing 'noblock'-capable
        // commands and using SendData to communicate module states to the PC interface.
        void Pulse()
        {
            // One major feature of AXMC runtime control is that it allows concurrent execution of multiple commands.
            // Specifically, while the microcontroller delays the execution of a command (see below), it can run other
            // modules' commands to maximize overall throughput. While there are trade-offs associated with this use
            // pattern, it may be desirable in certain runtime scenarios. To support the 'noblock' execution, the
            // command haa to implement the 'stage-based' design pattern and use the methods inherited from the base
            // Module class to transition between stages.

            // Stage 1: Activates the pin.
            // Note, stages automatically start with 1 and are incremented with each call to AdvanceCommandStage().
            if (GetCommandStage() == 1)  // GetCommandStage(0 returns the current command stage.
            {
                // Sets the pin to deliver a HIGH signal. DigitalWrite is a utility method inherited from the base
                // Module class. Internally, it uses the efficient digitalWriteFase library to maximize the speed of
                // triggering the pin. It also respects global controller TTL and Output pin locks.
                if (DigitalWrite(kPin, HIGH, false))  // 'false' tells the method this is NOT a TTL pin.
                {
                    // Notifies the PC that the pin is now HIGH.
                    SendData(static_cast<uint8_t>(kStates::kHigh));  // The value needs to be cast to uint8_t!

                    // It is essential to call this method to advance command stage after activating the pin. Otherwise,
                    // the microcontroller will get stuck re-executing this stage.
                    AdvanceCommandStage();
                }
                else
                {
                    // If writing to output pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime. This transmits the state-message to the
                    // PC. Note, the method also has an overloaded version for sending data in addition to state codes.
                    SendData(static_cast<uint8_t>(kStates::kOutputLocked));

                    // If the command runs into an error, it is usually a good idea to abort the command. This method
                    // ends the command without executing any further stages and ensures the command is NOT executed
                    // again until this is explicitly requested from the PC.
                    AbortCommand();
                    return;
                }
            }

            // Stage 2: delays while the pin emits the HIGH signal.
            if (GetCommandStage() == 2)
            {
                // WaitForMicros is another utility method inherited from the base Module class. The method
                // automatically determines whether the command runs in a blocking or non-blocking mode. If the mode is
                // non-blocking, the microcontroller will execute other modules' commands while delaying this command.
                if (!WaitForMicros(parameters.on_duration)) return;  // Return is essential for non-blocking mode.
                AdvanceCommandStage();
            }

            // Stage 3: disables the pin.
            // This is essentially the inverse of stage 1.
            if (GetCommandStage() == 3)
            {
                if (DigitalWrite(LED_BUILTIN, LOW, false))
                {
                    // Notifies the PC that the pin is now LOW.
                    SendData(static_cast<uint8_t>(kStates::kLow));
                    AdvanceCommandStage();
                }
                else
                {
                    SendData(static_cast<uint8_t>(kStates::kOutputLocked));
                    AbortCommand();
                }
            }

            // Stage 4: ensures the pin is kept off for at least the specified off_duration. This is the final stage of
            // this command, so also finishes command execution once the delay is up.
            if (GetCommandStage() == 4)
            {
                if (!WaitForMicros(parameters.on_duration)) return;

                // Calling this method ends command execution. If it is not called at the end of the last command
                // stage, the microcontroller will get stuck re-executing the last stage of this command.
                CompleteCommand();
            }
        }

        // Sends the current value of the 'echo_value' parameter to the PC. This command demonstrates the use of
        // SendData specialization for sending data objects alongside communicating module states to the PC interface.
        void Echo()
        {
            // Data objects transmitted to the PC might match one of the supported 'prototypes'. The PC has to know
            // how to read the serialized object data, and making it always match one of the prototypes ensures the PC
            // can deserialize the data. While the number of supported prototypes is limited, the available range
            // should cover most uses cases. If you need to add custom prototypes, modify the kPrototypes enumeration
            // in the axmc_communication_assets namespace (axmc_shared_assets.h source file).
            SendData(
                static_cast<uint8_t>(kStates::kEcho),
                axmc_communication_assets::kPrototypes::kOneUint16,
                parameters.echo_value
            );
            CompleteCommand();
        }
};

#endif  //AXMC_EXAMPLE_MODULE_H
