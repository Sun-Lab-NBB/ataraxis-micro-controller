// This example demonstrates the implementation of an AXMC-compatible hardware module class.

// This file demonstrates the proces of writing custom hardware module classes that use the AXMC library to integrate
// with the communication interface running on the host-computer (PC). This implementation showcases one of the many
// possible module design patterns. The main advantage of the AXMC library is that it is designed to work with any
// class design and layout, as long as it subclasses the base Module class and overloads the three pure virtual
// methods: SetCustomParameters, RunActiveCommand and SetupModule.

// For the best learning experience, it is recommended to review this code side-by-side with the implementation
// of the companion TestModuleInterface class defined in the ataraxis-communication-interface library:
// https://github.com/Sun-Lab-NBB/ataraxis-communication-interface#quickstart.

// See https://github.com/Sun-Lab-NBB/ataraxis-micro-controller for more details.
// API documentation: https://ataraxis-micro-controller-api-docs.netlify.app/.
// Authors: Ivan Kondratyev (Inkaros), Jasmine Si.

// Inclusion guards to prevent multiple inclusions
#ifndef AXMC_EXAMPLE_MODULE_H
#define AXMC_EXAMPLE_MODULE_H

// Imports the Arduino framework as the basis for all Teensy and Arduino projects.
#include <Arduino.h>

// Includes the necessary AXMC assets.
#include "module.h"

// Defines the TestModule class by subclassing the base Module class. This class is designed to do two things. First,
// it sends square digital pulses using the managed pin in response to receiving the 'pulse' command from the PC.
// Second, it sends the 'echo_value' parameter (see below) to the PC in response to receiving the 'echo' command from
// the PC.
template <const uint8_t kPin = 5>
class TestModule final : public Module
{
    public:
        // Defines a structure to store PC-addressable runtime parameters. The PC interface uses 'ModuleParameters'
        // messages to dynamically change the values of the parameters stored in this structure to adjust the instance's
        // runtime behavior. Storing all PC-addressable runtime parameters in a structure object is the best practice
        // for most use cases.
        struct CustomRuntimeParameters
        {
                uint32_t on_duration  = 2000000;  // The time, in microseconds, to keep the pin HIGH when pulsing.
                uint32_t off_duration = 2000000;  // The time, in microseconds, to keep the pin LOW when pulsing.
                uint16_t echo_value   = 666;      // The value sent to the PC as part of the Echo() command's runtime.
        } PACKED_STRUCT parameters;

        // This enumeration contains all state codes used by the class when communicating with the PC. The state codes
        // are used to communicate that the module has encountered a specific runtime event or error. How these codes
        // are used (or not) depends entirely on the implementation of the companion PC-side module interface. Avoid
        // using values below 51 (0-50) inside custom modules, as they are reserved for the system's use.
        enum class kStates : uint8_t
        {
            kHigh = 52,  // The managed digital pin is currently set to output a HIGH signal.
            kLow  = 53,  // The managed digital pin is currently set to output a LOW signal.
            kEcho = 54,  // This code is used by messages transmitting the 'echo_value' to the PC.
        };

        // This enumeration contains the codes for the commands that can be executed by the module. The command codes
        // are used to interpret the command messages received from the PC. Specifically, they are used to mpa the
        // received command codes to the appropriate class methods. Avoid using the value '0' inside custom modules, as
        // it is universally reserved by the system for error-coding.
        enum class kCommands : uint8_t
        {
            kPulse = 1,  // Sends a square digital pulse using the managed digital pin.
            kEcho  = 2,  // Sends the 'echo_value' parameter to the PC.
        };

        // As a minimum, the class constructor has to accept three arguments and pass them to the superclass (Module)
        // constructor. See the module_integration.cpp for notes on how to initialize the module class using this
        // constructor.
        TestModule(
            const uint8_t module_type,    // This value must be unique for each hardware module class.
            const uint8_t module_id,      // This value must be unique for each hardware module class instance.
            Communication& communication  // All module instances and the Kernel share the same Communication instance.
        ) :
            Module(module_type, module_id, communication)
        {}

        // The Kernel calls this method when it receives a parameters' message addressed to the module instance.
        // All custom modules should copy the implementation below, replacing the 'parameters' input with the
        // name of the structure used to store the PC-addressable runtime parameters.
        bool SetCustomParameters() override
        {
            // This overwrites the memory of the 'parameters' structure with new data received from the PC.
            return ExtractParameters(parameters);
        }

        // The Kernel calls this method to execute specific module commands (methods) associated with the currently
        // active command executed by the module.
        bool RunActiveCommand() override
        {
            // Accesses the currently active command (if any) and executes the associated class method. During runtime,
            // the Kernel class handles command activation and cyclically calls this method to execute the activated
            // command(s).
            switch (static_cast<kCommands>(GetActiveCommand()))  // Casts the command code to the enumeration type.
            {
                // Emits a square pulse via the managed digital pin.
                case kCommands::kPulse: Pulse(); return true;

                // Packages and sends the current value of the echo_parameter to the PC.
                case kCommands::kEcho: Echo(); return true;

                // Unrecognized command. The method must return 'false' if it does not recognize the command and
                // 'true' otherwise.
                default: return false;
            }
        }

        // The Kernel calls this method from the setup() function and when it is instructed to reset the
        // microcontroller. Use it to initialize module hardware and set runtime parameters to default values.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinMode(kPin, OUTPUT);
            digitalWrite(kPin, LOW);  // Sets the pin to LOW by default.

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
            // modules' commands to maximize the overall throughput. To support the 'noblock' execution, the
            // command MUST adhere to the 'stage-based' design pattern and use the methods inherited from the base
            // Module class to transition between stages.

            // Gets the current command stage.
            switch (GetCommandStage())
            {
                // Stage 1: Activates the pin.
                // Note, stages automatically start with 1 and are incremented with each call to AdvanceCommandStage().
                case 1:

                    // Sets the pin to deliver a HIGH signal.
                    digitalWrite(kPin, HIGH);

                    // Notifies the PC that the pin is now HIGH.
                    SendData(static_cast<uint8_t>(kStates::kHigh));

                    // It is essential to call this method to advance the command stage after activating the pin.
                    // Otherwise, the module gets stuck re-executing this stage.
                    AdvanceCommandStage();
                    break;

                // Stage 2: waits for the specified on_duration.
                case 2:

                    // The WaitForMicros() method either blocks in-place until the duration passes or returns with a
                    // boolean state to support non-blocking execution.
                    if (WaitForMicros(parameters.on_duration)) AdvanceCommandStage();
                    break;

                // Stage 3: disables the pin.
                case 3:
                    // Sets the pin to deliver a LOW signal.
                    digitalWrite(LED_BUILTIN, LOW);

                    // Notifies the PC that the pin is now LOW.
                    SendData(static_cast<uint8_t>(kStates::kLow));

                    AdvanceCommandStage();
                    break;

                // Stage 4: ensures the pin is kept off for at least the specified off_duration.
                case 4:
                    // Calling CompleteCommand() method ends command execution. If it is not called at the end of
                    // the last command stage, the module gets stuck re-executing the last stage of the command.
                    if (WaitForMicros(parameters.off_duration)) CompleteCommand();
                    break;

                // If the command stage does not match any expected stage, terminates the command's runtime with an
                // error and ensures that the command is not executed again until explicitly requested by the PC.
                default: AbortCommand(); break;
            }
        }

        // Sends the current value of the 'echo_value' parameter to the PC. This command demonstrates the use of
        // SendData specialization for sending data objects alongside communicating module states to the PC interface.
        void Echo()
        {
            // Data objects transmitted to the PC have to match one of the supported prototype objects. The PC has to
            // know how to read the serialized object data, and making it always match one of the prototypes ensures
            // the PC can deserialize the data.
            SendData(
                static_cast<uint8_t>(kStates::kEcho),
                kPrototypes::kOneUint16,
                parameters.echo_value
            );
            CompleteCommand();
        }
};

#endif  //AXMC_EXAMPLE_MODULE_H
