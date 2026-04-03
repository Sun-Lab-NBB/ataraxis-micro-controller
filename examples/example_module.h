/**
 * @file
 * @brief Demonstrates the implementation of an AXMC-compatible hardware module class.
 *
 * This file demonstrates the process of writing custom hardware module classes that use the AXMC library to integrate
 * with the communication interface running on the host-computer (PC). This implementation showcases one of the many
 * possible module design patterns. The main advantage of the AXMC library is that it is designed to work with any
 * class design and layout, as long as it subclasses the base Module class and overloads the three pure virtual
 * methods: SetCustomParameters, RunActiveCommand and SetupModule.
 *
 * @note For the best learning experience, it is recommended to review this code side-by-side with the implementation
 * of the companion TestModuleInterface class defined in the ataraxis-communication-interface library.
 */

#ifndef AXMC_EXAMPLE_MODULE_H
#define AXMC_EXAMPLE_MODULE_H

#include <Arduino.h>
#include "module.h"

/**
 * @brief Sends square digital pulses and echoes parameter values to the PC in response to commands received from the
 * communication interface.
 *
 * @tparam kPin The digital pin managed by this module instance.
 */
template <const uint8_t kPin = 5>
class TestModule final : public Module
{
    public:
        /// Stores the instance's PC-addressable runtime parameters.
        struct CustomRuntimeParameters
        {
                uint32_t on_duration  = 2000000;  ///< The time, in microseconds, to keep the pin HIGH when pulsing.
                uint32_t off_duration = 2000000;  ///< The time, in microseconds, to keep the pin LOW when pulsing.
                uint16_t echo_value   = 666;      ///< The value sent to the PC as part of the Echo() command's runtime.
        } PACKED_STRUCT parameters;

        /// Defines the state codes used by the class when communicating with the PC.
        enum class kStates : uint8_t
        {
            kHigh = 52,  ///< The managed digital pin is currently set to output a HIGH signal.
            kLow  = 53,  ///< The managed digital pin is currently set to output a LOW signal.
            kEcho = 54,  ///< Communicates the 'echo_value' to the PC.
        };

        /// Defines the codes for the commands that can be executed by the module.
        enum class kCommands : uint8_t
        {
            kPulse = 1,  ///< Sends a square digital pulse using the managed digital pin.
            kEcho  = 2,  ///< Sends the 'echo_value' parameter to the PC.
        };

        /// Initializes the base Module class with the provided type, id, and communication instance.
        TestModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication
        ) :
            Module(module_type, module_id, communication)
        {}

        /// Overwrites the module's runtime parameters structure with the data received from the PC.
        bool SetCustomParameters() override
        {
            return ExtractParameters(parameters);
        }

        /// Resolves and executes the currently active command.
        bool RunActiveCommand() override
        {
            switch (static_cast<kCommands>(get_active_command()))
            {
                case kCommands::kPulse: Pulse(); return true;
                case kCommands::kEcho: Echo(); return true;
                default: return false;
            }
        }

        /// Sets up the instance's hardware and software assets to default values.
        bool SetupModule() override
        {
            pinMode(kPin, OUTPUT);
            digitalWrite(kPin, LOW);

            parameters.on_duration  = 2000000;
            parameters.off_duration = 2000000;
            parameters.echo_value   = 123;

            return true;
        }

        /// Destroys the instance during cleanup.
        ~TestModule() override = default;

    private:

        /// Emits a square digital pulse using the managed pin.
        void Pulse()
        {
            switch (get_command_stage())
            {
                // Stage 1: Activates the pin.
                case 1:
                    digitalWrite(kPin, HIGH);
                    SendData(static_cast<uint8_t>(kStates::kHigh));
                    AdvanceCommandStage();
                    break;

                // Stage 2: Waits for the specified on_duration.
                case 2:
                    if (WaitForMicros(parameters.on_duration)) AdvanceCommandStage();
                    break;

                // Stage 3: Disables the pin.
                case 3:
                    digitalWrite(kPin, LOW);
                    SendData(static_cast<uint8_t>(kStates::kLow));
                    AdvanceCommandStage();
                    break;

                // Stage 4: Ensures the pin is kept off for at least the specified off_duration.
                case 4:
                    if (WaitForMicros(parameters.off_duration)) CompleteCommand();
                    break;

                default: AbortCommand(); break;
            }
        }

        /// Sends the current value of the 'echo_value' parameter to the PC.
        void Echo()
        {
            SendData(
                static_cast<uint8_t>(kStates::kEcho),
                kPrototypes::kOneUint16,
                parameters.echo_value
            );
            CompleteCommand();
        }
};

#endif  //AXMC_EXAMPLE_MODULE_H
