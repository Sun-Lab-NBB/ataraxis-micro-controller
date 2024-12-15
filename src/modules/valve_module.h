/**
 * @file
 * @brief The header-only file for the ValveModule class. This class allows interfacing with a solenoid valve to
 * controllably dispense precise amounts of fluid.
 *
 * @section vlv_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_VALVE_MODULE_H
#define AXMC_VALVE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Sends digital signals to dispense precise amounts of fluid via the managed solenoid valve.
 *
 * This module is specifically designed to send digital signals that trigger Field-Effect-Transistor (FET) gated relay
 * hardware to deliver voltage that opens or closes the controlled valve. Depending on configuration, this module is
 * designed to work with both Normally Closed (NC) and Normally Open (NO) valves.
 *
 * @note This class was calibrated to work with fluid valves that deliver micro-liter-precise amounts of fluid under
 * gravitational driving force. The current class implementation may not work as intended for other use cases.
 * Additionally, the class is designed for dispensing predetermined amounts of fluid and not for continuous flow rate
 * control, which would require a PWM-based approach similar to the one used by the BreakModule class.
 *
 * @tparam kPin the digital pin connected to the valve's FET-gated relay.
 * @tparam kNormallyClosed determines whether the managed valve is opened or closed when unpowered. This is
 * used to adjust the class behavior so that toggle OFF always means the valve is closed and toggle ON means the
 * valve is open.
 * @tparam kStartClosed determines the initial state of the valve during class initialization. This works
 * together with kNormallyClosed parameter to deliver the desired initial voltage level for the valve to either be
 * opened or closed after hardware initialization.
 */
template <const uint8_t kPin, const bool kNormallyClosed, const bool kStartClosed = true>
class ValveModule final : public Module
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for ValveModule instance."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kOutputLocked = 51,  ///< The output pin is in a global locked state and cannot be used to output data.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kSendPulse = 1,  ///< Deliver a precise amount of fluid by cycling valve open and close states.
            kToggleOn  = 2,  ///< Sets the valve to be permanently open.
            kToggleOff = 3,  ///< Sets the valve to be permanently closed.
            kCalibrate =
                4  ///< Repeatedly pulses teh valve to map different pulse_durations to dispensed fluid volumes.
        };

        /// Initializes the class by subclassing the base Module class.
        ValveModule(
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
                // Pulse
                case kModuleCommands::kSendPulse: Pulse(); return true;
                // Open
                case kModuleCommands::kToggleOn: Open(); return true;
                // Close
                case kModuleCommands::kToggleOff: Close(); return true;
                // Calibrate
                case kModuleCommands::kCalibrate: Calibrate(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinModeFast(kPin, OUTPUT);

            // Based on the requested initial valve state and the configuration of the valve (normally closed or open),
            // either opens or closes the valve following setup.
            if (kStartClosed) digitalWriteFast(kPin, kNormallyClosed ? LOW : HIGH);  // Ensures the valve is closed.
            else digitalWriteFast(kPin, kNormallyClosed ? HIGH : LOW);               // Ensures the valve is open.

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pulse_duration    = 10000;  // 10 milliseconds.
            _custom_parameters.calibration_delay = 10000;  // 10 milliseconds.
            _custom_parameters.calibration_count = 100;    // The valve is pulsed 100 times during calibration.

            return true;
        }

        ~ValveModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration    = 10000;  ///< The time, in microseconds, the valve is open during pulses.
                uint32_t calibration_delay = 10000;  ///< The time, in microseconds, to wait between calibration pulses.
                uint16_t calibration_count = 100;    ///< How many times to pulse the valve during calibration.
        } PACKED_STRUCT _custom_parameters;

        /// Depending on the valve configuration, stores the digital signal that needs to be sent to the output pin to
        /// open the valve.
        static constexpr bool kOpen = kNormallyClosed ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Depending on the valve configuration, stores the digital signal that needs to be sent to the output pin to
        /// close the valve.
        static constexpr bool kClose = kNormallyClosed ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Cycles opening and closing the valve to deliver the precise amount of fluid.
        void Pulse()
        {
            // Opens the valve
            if (execution_parameters.stage == 1)
            {
                // Toggles the pin to send the open signal. If the pin is successfully activated, as indicated by the
                // DigitalWrite returning true, advances the command stage.
                if (DigitalWrite(kPin, kOpen, false)) AdvanceCommandStage();
                else
                {
                    // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                    return;
                }
            }

            // Keeps the valve open while the desired amount fo fluid is passing through
            if (execution_parameters.stage == 2)
            {
                // Blocks for the pulse_duration of microseconds, relative to the time of the last AdvanceCommandStage()
                // call.
                if (!WaitForMicros(_custom_parameters.pulse_duration)) return;
                AdvanceCommandStage();
            }

            // Closes the valve
            if (execution_parameters.stage == 3)
            {
                // Once the pulse duration has passed, inactivates the pin by setting it to Close signal. Finishes
                // command execution if inactivation is successful.
                if (DigitalWrite(kPin, kClose, false)) CompleteCommand();
                else
                {
                    // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                    // sends an error message to the PC and aborts the runtime.
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                }
            }
        }

        /// Permanently opens the valve.
        void Open()
        {
            // Sets the pin to Open signal and finishes command execution
            if (DigitalWrite(kPin, kOpen, false)) CompleteCommand();
            else
            {
                // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Permanently closes the valve.
        void Close()
        {
            // Sets the pin to Close signal and finishes command execution
            if (DigitalWrite(kPin, kClose, false)) CompleteCommand();  // Finishes command execution
            else
            {
                // If writing to TTL pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Pulses the valve calibration_count times without blocking or (majorly) delaying. This is used to establish
        /// the relationship between the pulse_duration and the amount of fluid delivered during the pulse. This
        /// calibration is necessary to precisely control the amount of fluid delivered by the valve by using specific
        /// pulse durations.
        void Calibrate()
        {
            // Pulses the valve the requested number of times. Note, the command logic is very similar to the
            // Pulse command, but it is slightly modified to account for the fact that some boards can issue commands
            // too fast for the valve hardware to properly respond to them. Also, this command is blocking by design and
            // will run all requested pulse cycles in one go.
            for (uint16_t i = 0; i < _custom_parameters.calibration_count; ++i)
            {
                // Opens the valve
                if (!DigitalWrite(kPin, kOpen, false))
                {
                    // Respects the global controller lock state
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();
                    return;
                }

                // Blocks in-place until the pulse duration passes.
                while (!WaitForMicros(_custom_parameters.pulse_duration))
                {
                }

                // Closes the valve
                if (!DigitalWrite(kPin, kClose, false))
                {
                    // Respects the global controller lock state
                    SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                    AbortCommand();  // Aborts the current and all future command executions.
                    return;
                }

                // Blocks for calibration_delay of microseconds to ensure the valve closes before initiating the next
                // cycle.
                while (!WaitForMicros(_custom_parameters.calibration_delay))
                {
                }
            }

            // This command completes after running the requested number of cycles.
            CompleteCommand();
        }
};

#endif  //AXMC_VALVE_MODULE_H
