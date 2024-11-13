/**
 * @file
 * @brief The header file for the SensorModule class, which enables monitoring the input analog pin activity and,
 * depending on the detected signal, notifying the PC about the sensed changes.
 */

#ifndef AXMC_SENSOR_MODULE_H
#define AXMC_SENSOR_MODULE_H

#include <Arduino.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Monitors the managed analog pin activity and, upon detecting a significant change in the recorded signal,
 * notifies the PC.
 *
 * This module is designed to compliment the digital signal sensing functionality offered by TTLModule to extend it to
 * analog signals. In turn, this module can interface with a range of hardware sensors, most commonly current and
 * voltage sensors. In general, any sensor that produces a single-pin voltage signal can be interfaced with through
 * this class.
 *
 * @tparam kPin the analog pin whose state will be monitored to detect incoming sensor-sent signals.
 */
template <const uint8_t kPin>
class SensorModule final : public Module
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different kPin for BreakModule class."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kInput = 51,  /// The monitored pin received a signal within the reporting threshold.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Checks the state of the input pin, and if it changed since last check, informs the PC
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
        SensorModule(
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
            pinModeFast(kPin, INPUT);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.lower_threshold   = 0;      // By default, disables the lower threshold.
            _custom_parameters.upper_threshold   = 65535;  // By default, disables the upper threshold.
            _custom_parameters.delta_threshold   = 1;      // By default, static (unchanging) readouts are not reported.
            _custom_parameters.average_pool_size = 10;     // Averages 10 pin readouts by default.

            _previous_readout = 65535;  // Should be initialized to invalid value, so anything > 5000 is ok here.

            return true;
        }

        ~SensorModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint16_t lower_threshold  = 0;      ///< The lower boundary for signals to be reported to PC.
                uint16_t upper_threshold  = 65535;  ///< The upper boundary for signals to be reported to PC.
                uint16_t delta_threshold  = 1;   ///< The minimum difference between pin readouts to be reported to PC.
                uint8_t average_pool_size = 10;  ///< The number of analog readouts to average when checking pin state.
        } __attribute__((packed)) _custom_parameters;

        /// Stores the previous readout of the analog pin. This is used to limit the number of messages sent to the
        /// PC by only reporting significant changes to the pin state. The level that constitutes significant change
        /// can be adjusted through the custom_parameters structure.
        uint16_t _previous_readout = 65535;  // Should be initialized to invalid value, so anything > 5000 is ok here.

        /// Checks the signal received by the input pin and, if it is within reporting thresholds and is significantly
        /// different from the previous readout, reports it to the PC.
        void CheckState()
        {
            // Evaluates the state of the pin. Averages the requested number of readouts to produce the final
            // analog signal value.
            const uint16_t signal = GetRawAnalogReadout(kPin, _custom_parameters.average_pool_size);

            // Statically prevents reporting signals that are the same as the previous readout value.
            if (signal == _previous_readout)
            {
                CompleteCommand();
                return;
            }

            // To optimize communication, only sends data to the PC if the signal has significantly
            // changed and allows to optionally notch or long-pass / short-pass detected signals.
            if (signal >= _custom_parameters.lower_threshold && signal <= _custom_parameters.upper_threshold)
            {
                // Calculates the change in signal relative to the previous readout and, if it exceeds the change
                // threshold, sends the signal data to the PC.
                if (abs(signal - _previous_readout) >= _custom_parameters.delta_threshold)
                {
                    // Sends the detected signal to the PC.
                    SendData(
                        static_cast<uint8_t>(kCustomStatusCodes::kInput),
                        communication_assets::kPrototypes::kOneUnsignedShort,
                        signal
                    );
                    _previous_readout = signal;  // Overwrites the previous readout with the current signal.
                }
            }

            // Completes command execution
            CompleteCommand();
        }
};

#endif  //AXMC_SENSOR_MODULE_H
