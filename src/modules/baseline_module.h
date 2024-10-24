/**
 * @file
 * @brief The header file for the TTLModule class, which enables bidirectional TTL communication with other hardware
 * systems.
 */

#ifndef AXMC_QUADRATURE_ENCODER_MODULE_H
#define AXMC_QUADRATURE_ENCODER_MODULE_H

#include <Arduino.h>
#include <Encoder.h>
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
template <const uint8_t kFirstInterruptPin, const uint8_t kSecondInterruptPin>
class TTLModule final : public Module
{
        // The only reason why pins are accessed via template parameter is to enable this static assert here
        static_assert(kFirstInterruptPin != kSecondInterruptPin, "The two encoder interrupt pins cannot be the same.");

    public:

        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kStandBy      = 51,  ///< The code used to initialize custom status trackers
            kOutputOn     = 52,  ///< The output ttl pin is set to HIGH
            kOutputOff    = 53,  ///< The output ttl pin is set to LOW
            kInputOn      = 54,  ///< The input ttl pin is set to HIGH
            kInputOff     = 55,  ///< The input ttl pin is set to LOW
            kOutputLocked = 56,  ///< The output ttl pin is in a locked state (global ttl_lock is ON).
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse  = 1,  ///< Sends a ttl pulse through the output pin.
            kToggleOn   = 2,  ///< Sets the output pin to HIGH.
            kToggleOff  = 3,  ///< Sets the output pin to LOW.
            kCheckState = 4,  ///< Checks the state of the input pin and if it changed since last check, informs the PC
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
                case 0: return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kSetupComplete);  // Records the status
            return true;
        }

        /// Resets the custom_parameters structure fields to their default values.
        bool ResetCustomAssets() override
        {
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
};

#endif  //AXMC_QUADRATURE_ENCODER_MODULE_H
