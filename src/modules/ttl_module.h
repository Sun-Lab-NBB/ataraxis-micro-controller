/**
 * @file
 *
 * @brief The header-only file for the TTLModule class. This class allows establishing bidirectional TTL
 * communication with other hardware systems, such as microcontrollers, cameras and data recording devices.
 *
 * @section ttl_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_TTL_MODULE_H
#define AXMC_TTL_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "axmc_shared_assets.h"
#include "module.h"

/**
 * @brief Sends or receives Transistor-to-Transistor Logic (TTL) signals using the specified digital pin.
 *
 * The class is statically configured to either receive or output TTL signals, it cannot do both at the same time!
 *
 * @tparam kPin the digital pin that will be used to output or receive ttl signals. The mode of the pin (input or
 * output) depends on kOutput flag value and is currently not changeable during runtime.
 * @tparam kOutput determines whether the pin will be used to output TTL signals (if set to true) or receive TTL signals
 * from other systems (if set to false).
 * @tparam kStartOn determines the initial state of the pin when the class is configured to output TTL signals. If set
 * to true, the TTL pin will be set to High during hardware initialization. Otherwise (by default) it will be set to
 * Low. This parameter is ignored if the class is configured to receive TTL signals.
 */
template <const uint8_t kPin, const bool kOutput = true, const bool kStartOn = false>
class TTLModule final : public Module
{
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for TTLModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kOutputLocked   = 51,  ///< The output ttl pin is in a global locked state and cannot be toggled on or off.
            kInputOn        = 52,  ///< The input ttl pin is receiving a HIGH signal.
            kInputOff       = 53,  ///< The input ttl pin is receiving a LOW signal.
            kInvalidPinMode = 54   ///< The pin mode (input or output) is not valid for the requested command.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse  = 1,  ///< Sends a ttl pulse through the output pin.
            kToggleOn   = 2,  ///< Sets the output pin to HIGH.
            kToggleOff  = 3,  ///< Sets the output pin to LOW.
            kCheckState = 4,  ///< Checks the state of the input pin.
        };

        /// Initializes the class by subclassing the base Module class.
        TTLModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const axmc_shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            // Extracts the received parameters into the _custom_parameters structure of the class. If extraction fails,
            // returns false. This instructs the Kernel to execute the necessary steps to send an error message to the
            // PC.
            return _communication.ExtractModuleParameters(_custom_parameters);
        }

        /// Executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // SendPulse
                case kModuleCommands::kSendPulse: SendPulse(); return true;
                // ToggleOn
                case kModuleCommands::kToggleOn: ToggleOn(); return true;
                // ToggleOff
                case kModuleCommands::kToggleOff: ToggleOff(); return true;
                // CheckState
                case kModuleCommands::kCheckState: CheckState(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Depending on the output flag, configures the pin as either input or output.
            if (kOutput)
            {
                pinModeFast(kPin, OUTPUT);

                // Depending on the class configuration, initializes the pin to the desired state.
                if (!kStartOn) digitalWriteFast(kPin, LOW);
                else digitalWriteFast(kPin, HIGH);
            }
            else pinModeFast(kPin, INPUT);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 10000;  // 10000 microseconds == 10 milliseconds.
            _custom_parameters.average_pool_size = 0;      // 0 or 1 == no averaging.

            return true;
        }

        ~TTLModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 10000;  ///< The time, in microseconds, the pin outputs HIGH during pulses.
                uint8_t average_pool_size = 0;  ///< The number of digital readouts to average when checking pin state.
        } PACKED_STRUCT _custom_parameters;

        /// Sends a digital pulse through the output pin, using the preconfigured pulse_duration of microseconds.
        /// Supports non-blocking execution and respects the global ttl_lock state.
        void SendPulse()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
                return;
            }

            // Initializes the pulse
            if (execution_parameters.stage == 1)
            {
                // Toggles the pin to send a HIGH signal. If the pin is successfully set to HIGH, as indicated by the
                // DigitalWrite returning true, advances the command stage.
                if (DigitalWrite(kPin, HIGH, true)) AdvanceCommandStage();
                else
                {
                    // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                    return;
                }
            }

            // Delays for the requested number of microseconds before inactivating the pulse.
            if (execution_parameters.stage == 2)
            {
                // Blocks for the pulse_duration of microseconds, relative to the time of the last AdvanceCommandStage()
                // call.
                if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                AdvanceCommandStage();
            }

            // Inactivates the pulse
            if (execution_parameters.stage == 3)
            {
                // Once the pulse duration has passed, inactivates the pin by setting it to LOW. Finishes command
                // execution if inactivation is successful.
                if (DigitalWrite(kPin, LOW, true)) CompleteCommand();
                else
                {
                    // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                }
            }
        }

        /// Sets the output pin to continuously send HIGH signal. Respects the global ttl_lock state.
        void ToggleOn()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
                return;
            }

            // Sets the pin to HIGH and finishes command execution
            if (DigitalWrite(kPin, HIGH, true)) CompleteCommand();
            else
            {
                // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Sets the output pin to continuously send LOW signal. Respects the global ttl_lock state.
        void ToggleOff()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
                return;
            }

            // Sets the pin to LOW and finishes command execution
            if (DigitalWrite(kPin, LOW, true)) CompleteCommand();  // Finishes command execution
            else
            {
                // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Checks the state of the input pin and, if the state does not match the previous state, sends the new state
        /// to the PC.
        void CheckState()
        {
            // Tracks the previous input_pin status. This is used to optimize data transmission by only reporting input
            // pin state changes to the PC.
            static bool previous_input_status = false;

            // Calling this command when the class is configured to output TTL pulses triggers an invalid
            // pin mode error.
            if (kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
                return;
            }

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // state-value. To optimize communication, only sends data to the PC if the state has changed.
            if (const bool current_state = GetRawDigitalReadout(kPin, _custom_parameters.average_pool_size);
                previous_input_status != current_state)
            {
                // Updates the state tracker.
                previous_input_status = current_state;

                // If the state is true, sends InputOn message, otherwise sends InputOff message.
                if (current_state) SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOn));
                else SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOff));
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_TTL_MODULE_H
