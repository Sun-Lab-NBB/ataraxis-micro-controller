/**
 * @file
 * @brief The header-only file for the LickModule class. This class allows interfacing with a custom conductive lick
 * sensor used to monitor the licking behavior of animals.
 *
 * This class was specifically designed to monitor the licking behavior of laboratory mice during experiments. This
 * class will likely work for other purposes that benefit from detecting wet or dry contacts, such as touch or lick
 * sensors.
 *
 * @section lck_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_LICK_MODULE_H
#define AXMC_LICK_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "axmc_shared_assets.h"
#include "module.h"

/**
 * @brief Monitors the state of a custom conductive lick sensor for significant state changes and notifies the PC when
 * such changes occur.
 *
 * This module is designed to work with a custom lick sensor. The sensor works by directly injecting a small amount of
 * current through one contact surface and monitoring the pin wired to the second contact surface. When the animal,
 * such as a laboratory mouse, licks the sensor while making contact with the current injector surface, the sensor
 * detects a positive change in voltage across the sensor. The detection threshold can be configured to distinguish
 * between dry and wet touch, which is used to separate limb contacts from tongue contacts.
 *
 * @note This class was calibrated to work for and tested on C57BL6J Wildtype and transgenic mice.
 *
 * @tparam kPin the analog pin whose state will be monitored to detect licks.
 */
template <const uint8_t kPin>
class LickModule final : public Module
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for LickModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kChanged = 51,  /// The signal received by the monitored pin has significantly changed since the last check.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the state of the input pin, and if necessary informs the PC of any changes.
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
        LickModule(
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
                // CheckState
                case kModuleCommands::kCheckState: CheckState(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets pin to Input mode.
            pinModeFast(kPin, INPUT);

            // Resets the custom_parameters structure fields to their default values. Assumes 12-bit ADC resolution.
            _custom_parameters.lower_threshold   = 1000;   // ~0.8 / 1.2 V, depending on CPU voltage rating.
            _custom_parameters.upper_threshold   = 65535;  // 3.3/ 5.0 V, depending on CPU voltage rating.
            _custom_parameters.delta_threshold   = 500;    // Reports changes of ~0.4 / 0.6 V steps.
            _custom_parameters.average_pool_size = 50;     // Averages 50 readouts to eliminate noise.

            return true;
        }

        ~LickModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t lower_threshold = 1000;   ///< The lower boundary for signals to be reported to PC.
                uint16_t upper_threshold = 65535;  ///< The upper boundary for signals to be reported to PC.
                uint16_t delta_threshold = 500;  ///< The minimum signal difference between checks to be reported to PC.
                uint8_t average_pool_size = 50;  ///< The number of readouts to average into pin state value.
        } PACKED_STRUCT _custom_parameters;

        /// Checks the signal received by the input pin and, if necessary, reports it to the PC.
        void CheckState()
        {
            // Stores the previous readout of the analog pin. This is used to limit the number of messages sent to the
            // PC by only reporting significant changes of the pin state (signal). The level that constitutes
            // significant change can be adjusted through the custom_parameters structure.
            static uint16_t previous_readout = 0;

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value. Note, since we statically configure the controller to use 10-14 bit ADC resolution,
            // this value should not use the full range of the 16-bit unit variable.
            const uint16_t signal = GetRawAnalogReadout(kPin, _custom_parameters.average_pool_size);

            // Also calculates the absolute difference between the current signal and the previous readout. This is used
            // to ensure only significant signal changes are reported to the PC. Note, although we are casting both to
            // int32 to support the delta calculation, the resultant delta will always be within the unit_16 range.
            // Therefore, it is fine to cast it back to uint16 to avoid unnecessary future casting in the 'if'
            // statements.
            const auto delta =
                static_cast<uint16_t>(abs(static_cast<int32_t>(signal) - static_cast<int32_t>(previous_readout)));

            // Prevents reporting signals that are not significantly different from the previous readout value.
            if (delta <= _custom_parameters.delta_threshold)
            {
                CompleteCommand();
                return;
            }

            // Optionally allows notch, long-pass or short-pass filtering detected signals.
            if (signal >= _custom_parameters.lower_threshold && signal <= _custom_parameters.upper_threshold)
            {
                // Sends the detected signal to the PC.
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kChanged),
                    axmc_communication_assets::kPrototypes::kOneUnsignedShort,
                    signal
                );

                previous_readout = signal;  // Overwrites the previous readout with the current signal.
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_LICK_MODULE_H
