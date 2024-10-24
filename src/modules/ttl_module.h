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
 * @brief Sends and receives Transistor-to-Transistor Logic (TTL) signals using two digital pins.
 *
 * This module is specifically designed to send and receive TTL signals, which are often used to synchronize and
 * communicate between low-level hardware equipment. Currently, this module can work with one input and one output pin
 * at the same time and the two pins must be different.
 *
 * @tparam kOutputPin the digital pin that will be used to output the ttl signals.
 * @tparam kInputPin the digital pin whose state will be monitored to detect incoming ttl signals.
 */
template <const uint8_t kOutputPin, const uint8_t kInputPin>
class TTLModule final : public Module
{
        // The only reason why pins are accessed via template parameter is to enable this static assert here
        static_assert(kOutputPin != kInputPin, "Input and output pins cannot be the same.");

    public:

        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kStandBy      = 51,  ///< The code used to initialize custom status trackers.
            kOutputOn     = 52,  ///< The output ttl pin is set to HIGH.
            kOutputOff    = 53,  ///< The output ttl pin is set to LOW.
            kInputOn      = 54,  ///< The input ttl pin is set to HIGH.
            kInputOff     = 55,  ///< The input ttl pin is set to LOW.
            kOutputLocked = 56,  ///< The output ttl pin is in a locked state (global ttl_lock is ON).
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
            if (!_communication.ExtractParameters(_custom_parameters)) return false;
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kParametersSet);  // Records the status
            return true;
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
            pinMode(kOutputPin, OUTPUT);
            pinMode(kInputPin, INPUT);
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kSetupComplete);  // Records the status
            return true;
        }

        /// Resets the custom_parameters structure fields to their default values.
        bool ResetCustomAssets() override
        {
            _custom_parameters.pulse_duration    = 10000;  // The default pulse duration is 10 milliseconds
            _custom_parameters.average_pool_size = 0;      // The default average pool size is 0 readouts (no averaging)
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kModuleAssetsReset);  // Records the status
            return true;
        }

        ~TTLModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration   = 0;  ///< The time, in microseconds, for the HIGH phase of emitted pulses.
                uint8_t average_pool_size = 0;  ///< The number of digital readouts to average when checking pin state.
        } _custom_parameters;

        /// Tracks the previous input_pin status. This is used to optimize data transmission by only reporting state
        /// changes to the PC
        uint8_t _previous_input_status = static_cast<uint8_t>(kCustomStatusCodes::kStandBy);

        /// Sends a digital pulse through the output pin, using the preconfigured pulse_duration of microseconds.
        /// Supports non-blocking execution and respects the global ttl_lock state.
        void SendPulse()
        {
            // Activation
            if (execution_parameters.stage == 1)
            {
                if (DigitalWrite(kOutputPin, HIGH, true))
                {
                    module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputOn);  // Records the status
                    SendData(module_status, communication_assets::kDataPlaceholder);  // Informs the PC about pin status
                    AdvanceCommandStage();                                            // Advances to the next stage
                }
                else
                {
                    module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked);  // Records the status
                    SendData(module_status, communication_assets::kDataPlaceholder);  // Informs the PC about pin status
                    CompleteCommand();  // Since the output pin is locked, the command is essentially aborted
                }
            }

            // Inactivation
            if (execution_parameters.stage == 2)
            {
                // Blocks for the pulse_duration of microseconds, relative to time of the AdvanceCommandStage() call.
                if (!WaitForMicros(_custom_parameters.pulse_duration)) return;

                // Once the pulse duration has passed, inactivates the pin
                digitalWriteFast(kOutputPin, LOW);
                module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputOff);  // Records the pin status
                SendData(module_status, communication_assets::kDataPlaceholder);  // Informs the PC about pin status
                CompleteCommand();                                                // Finishes command execution
            }
        }

        /// Sets the output pin to continuously send HIGH signal. Respects the global ttl_lock state.
        void ToggleOn()
        {
            // Since this is technically a 1-stage command, does not make use of the stages.

            // Note, this call respects the ttl_lock flag!
            if (DigitalWrite(kOutputPin, HIGH, true))
            {
                // This clause will not be triggered if the global ttl_lock setting is ON.
                module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputOn);  // Records the status
            }
            else
            {
                module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputOff);  // Records the status
            }
            SendData(module_status, communication_assets::kDataPlaceholder);  // Informs the PC about pin status
            CompleteCommand();                                                // Finishes command execution
        }

        /// Sets the output pin to continuously send LOW signal. Respects the global ttl_lock state.
        void ToggleOff()
        {
            // Since this is technically a 1-stage command, does not make use of the stages.

            // While DigitalWrite may not set the pin to HIGH, depending on the ttl_lock global argument, it will always
            // set the pin to LOW. However, the method is written to support both conditions to match ToggleOn logic and
            // run time.
            if (DigitalWrite(kOutputPin, LOW, true))
            {
                module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputOn);  // Records the status
            }
            else
            {
                module_status = static_cast<uint8_t>(kCustomStatusCodes::kOutputOff);  // Records the status
            }
            SendData(module_status, communication_assets::kDataPlaceholder);  // Informs the PC about pin status
            CompleteCommand();                                                // Finishes command execution
        }

        /// Checks the state of the input pin and if the state does not match the recorded state, sends the new state
        /// to the PC.
        void CheckState()
        {
            // Since this is technically a 1-stage command, does not make use of the stages.

            // Evaluates the state of the pin. Note, averages the requested number of readouts to produce the final
            // state-value (by default, averaging is disabled).
            if (const bool current_state = GetRawDigitalReadout(kInputPin, _custom_parameters.average_pool_size);
                current_state && _previous_input_status != static_cast<uint8_t>(kCustomStatusCodes::kInputOn))
            {
                _previous_input_status = static_cast<uint8_t>(kCustomStatusCodes::kInputOn);  // Records the new status
                module_status          = _previous_input_status;
                SendData(module_status,
                         communication_assets::kDataPlaceholder);  // Informs the PC about pin status
            }
            else if (!current_state && _previous_input_status != static_cast<uint8_t>(kCustomStatusCodes::kInputOff))
            {
                _previous_input_status = static_cast<uint8_t>(kCustomStatusCodes::kInputOff);  // Records the new status
                module_status          = _previous_input_status;
                SendData(module_status,
                         communication_assets::kDataPlaceholder);  // Informs the PC about pin status
            }

            // To optimize communication, only sends data to the PC if the status has changed.
            CompleteCommand();
        }
};

#endif  //AXMC_TTL_MODULE_H
