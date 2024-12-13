/**
 * @file
 * @brief The header-only file for the TorqueModule class. This class allows interfacing with a torque sensor used to
 * monitor the torque applied to the connected object.
 *
 * @attention Since most torque sensors output a differential signal to code for the direction of the torque, this
 * class is designed to work with an AD620 microvolt amplifier to straighten and amplify the output of the torque
 * sensor.
 *
 * @subsection trq_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_TORQUE_MODULE_H
#define AXMC_TORQUE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Monitors the signal sent by an AD620 amplifier connected to the torque sensor
 *
 * This module is specifically designed to send and receive TTL signals, which are often used to synchronize and
 * communicate between low-level hardware equipment. Currently, this module can work with one input and one output pin
 * at the same time and the two pins must be different.
 *
 * @tparam kCWPin the digital pin that will be used to output the ttl signals.
 * @tparam kCCWPin the digital pin whose state will be monitored to detect incoming ttl signals.
 */
template <const uint8_t kCWPin, const uint8_t kCCWPin, const bool kInvertDirection>
class TorqueModule final : public Module
{
        // Ensures that the differential torque pins are different.
        static_assert(kCWPin != kCCWPin, "The two TorqueModule pins cannot be the same.");

        // Also ensures that torque pins do not interfere with LED pin.
        static_assert(
            kCWPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Clockwise pin for TorqueModule "
            "instance."
        );
        static_assert(
            kCCWPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different CounterClockwise pin for "
            "TorqueModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kCCWTorque = 51,  ///< The encoder was rotated in the CCW direction.
            kCWTorque  = 52,  ///< The encoder was rotated in the CW.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the state of the input pin, and if necessary informs the PC of any changes.
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
        TorqueModule(
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
            pinModeFast(kCWPin, INPUT);
            pinModeFast(kCCWPin, INPUT);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.lower_threshold   = 0;      // By default, disables the lower threshold.
            _custom_parameters.upper_threshold   = 65535;  // By default, disables the upper threshold.
            _custom_parameters.delta_threshold   = 1;      // By default, disables delta thresholding.
            _custom_parameters.average_pool_size = 10;     // By default, averages 10 pin readouts.

            return true;
        }

        ~TorqueModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t lower_threshold  = 0;      ///< The lower boundary for signals to be reported to PC.
                uint16_t upper_threshold  = 65535;  ///< The upper boundary for signals to be reported to PC.
                uint16_t delta_threshold  = 1;      ///< The minimum difference between pin checks to be reported to PC.
                uint8_t average_pool_size = 10;     ///< The number of readouts to average into pin state value.
        } __attribute__((packed)) _custom_parameters;

        /// Checks the signal received by the input pin and, if necessary, reports it to the PC.
        void CheckState()
        {
            // Stores the previous readout of the analog pin. This is used to limit the number of messages sent to the
            // PC by only reporting significant changes of the pin state (signal). The level that constitutes
            // significant change can be adjusted through the custom_parameters structure.
            static uint16_t previous_readout = 0;

            // This is used to ensure that the first readout after class reset is always sent to PC
            static bool once = true;

            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value. Note, since we statically configure the controller to use 10-14 bit ADC resolution,
            // this value should not use the full range of the 16-bit unit variable.
            const uint16_t signal = GetRawAnalogReadout(kPin, _custom_parameters.average_pool_size);

            // Also calculates the absolute difference between the current signal and the previous readout. This is used
            // to ensure only significant signal changes are reported to the PC. Note, although we are casting both to
            // int32 to support the delta calculation, the resultant delta will always be within the unit_16 range.
            // Therefore, it is fine to cast it back to uint16 to avoid unnecessary future casting in the if statements.
            const uint16_t delta = abs(static_cast<int32_t>(signal) - static_cast<int32_t>(previous_readout));

            // Prevents reporting signals that are the same as the previous readout value. Also prevents sending signals
            // that are not significantly different from the previous readout value. However, this entire check is
            // bypassed if 'once' flag is True to ensure the first readout is always reported.
            if (!once && (delta == 0 || delta < _custom_parameters.delta_threshold))
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
                    communication_assets::kPrototypes::kOneUnsignedShort,
                    signal
                );

                previous_readout = signal;  // Overwrites the previous readout with the current signal.
                if (once) once = false;     // Sets the once flag to false after the first successful report.
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_TORQUE_MODULE_H
