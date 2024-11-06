/**
 * @file
 * @brief The header file for the TTLModule class, which enables bidirectional TTL communication with other hardware
 * systems.
 */

#ifndef AXMC_TTL_MODULE_H
#define AXMC_TTL_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Sends or receives Transistor-to-Transistor Logic (TTL) signals using two digital pins.
 *
 * This module is specifically designed to send and receive TTL signals, which are often used to synchronize and
 * communicate between hardware devices. Currently, this module is designed to exclusively output or receive
 * TTL signals, depending on the pin configuration flag value selected at class compilation.
 *
 * @tparam kPin the digital pin that will be used to output or receive ttl signals. The mode of the pin
 * (input or output) depends on kOutput flag value.
 * @tparam kOutput determines whether the pin will be used to output TTL signals (if set to true) or receive TTL signals
 * from other systems (if set to false).
 */
template <const uint8_t kPin, const bool kOutput = true>
class TTLModule final : public Module
{
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Please select a different "
            "pin for TTLModule class."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kStandBy        = 51,  ///< The code used to initialize custom status trackers.
            kOutputOn       = 52,  ///< The output ttl pin is set to HIGH.
            kOutputOff      = 53,  ///< The output ttl pin is set to LOW.
            kInputOn        = 54,  ///< The input ttl pin is receiving a HIGH signal.
            kInputOff       = 55,  ///< The input ttl pin is receiving a LOW signal.
            kOutputLocked   = 56,  ///< The output ttl pin is in a global locked state and cannot be toggled on or off.
            kInvalidPinMode = 57   ///< The pin mode (input or output) is not valid for the requested command.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse  = 1,  ///< Sends a ttl pulse through the output pin.
            kToggleOn   = 2,  ///< Sets the output pin to HIGH.
            kToggleOff  = 3,  ///< Sets the output pin to LOW.
            kCheckState = 4,  ///< Checks the state of the input pin, and if it changed since last check, informs the PC
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
        TTLModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters
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

        /// Executes the currently active command. Unlike other overloaded API methods, this one does not set the
        /// module_status. Instead, it expects the called command to set the status as necessary.
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
                digitalWriteFast(kPin, LOW);
            }
            else pinModeFast(kPin, INPUT);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 10000;  // The default output pulse duration is 10 milliseconds
            _custom_parameters.average_pool_size = 0;      // The default average pool size is 0 readouts (no averaging)

            return true;
        }

        ~TTLModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 10000;  ///< The time, in microseconds, for the HIGH phase of emitted pulses.
                uint8_t average_pool_size = 0;  ///< The number of digital readouts to average when checking pin state.
        } __attribute__((packed)) _custom_parameters;

        /// Tracks the previous input_pin status. This is used to optimize data transmission by only reporting input
        /// pin state changes to the PC
        uint8_t _previous_input_status = static_cast<uint8_t>(kCustomStatusCodes::kStandBy);

        /// Sends a digital pulse through the output pin, using the preconfigured pulse_duration of microseconds.
        /// Supports non-blocking execution and respects the global ttl_lock state.
        void SendPulse()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                CompleteCommand();  // Finishes the command upon error detection
            }

            // Activation
            if (execution_parameters.stage == 1)
            {
                if (DigitalWrite(kPin, HIGH, true))
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOn));  // Informs the PC about pin status
                    AdvanceCommandStage();                                          // Advances to the next stage
                }
                else
                {
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked)
                    );                  // Informs the PC about pin status
                    CompleteCommand();  // Since the output pin is locked, the command is essentially aborted
                }
            }

            // Inactivation
            if (execution_parameters.stage == 2)
            {
                // Blocks for the pulse_duration of microseconds, relative to time of the AdvanceCommandStage() call.
                if (!WaitForMicros(_custom_parameters.pulse_duration)) return;

                // Once the pulse duration has passed, inactivates the pin
                if (!DigitalWrite(kPin, LOW, true)) SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOff));
                else SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));

                CompleteCommand();  // Finishes command execution
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
                CompleteCommand();  // Finishes the command upon error detection
            }

            // Note, this call respects the ttl_lock flag!
            if (DigitalWrite(kPin, HIGH, true))
            {
                // This clause will not be triggered if the global ttl_lock setting is ON.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOn));
            }
            else
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
            }
            CompleteCommand();  // Finishes command execution
        }

        /// Sets the output pin to continuously send LOW signal. Respects the global ttl_lock state.
        void ToggleOff()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                CompleteCommand();  // Finishes the command upon error detection
            }

            // While DigitalWrite may not set the pin to HIGH, depending on the ttl_lock global argument, it will always
            // set the pin to LOW. However, the method is written to support both conditions to match ToggleOn logic and
            // run time.
            if (DigitalWrite(kPin, LOW, true))
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputOff));  // Records the status
            }
            else
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));  // Records the status
            }
            CompleteCommand();  // Finishes command execution
        }

        /// Checks the state of the input pin and if the state does not match the recorded state, sends the new state
        /// to the PC.
        void CheckState()
        {
            // Calling this command when the class is configured to output TTL pulses triggers an invalid
            // pin mode error.
            if (kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                CompleteCommand();  // Finishes the command upon error detection
            }

            // Evaluates the state of the pin. Note, averages the requested number of readouts to produce the final
            // state-value (by default, averaging is disabled).
            if (const bool current_state = GetRawDigitalReadout(kPin, _custom_parameters.average_pool_size);
                current_state && _previous_input_status != static_cast<uint8_t>(kCustomStatusCodes::kInputOn))
            {
                _previous_input_status = static_cast<uint8_t>(kCustomStatusCodes::kInputOn);
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOn));
            }
            else if (!current_state && _previous_input_status != static_cast<uint8_t>(kCustomStatusCodes::kInputOff))
            {
                _previous_input_status = static_cast<uint8_t>(kCustomStatusCodes::kInputOff);
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInputOff));
            }

            // To optimize communication, only sends data to the PC if the status has changed.
            CompleteCommand();
        }
};

#endif  //AXMC_TTL_MODULE_H
