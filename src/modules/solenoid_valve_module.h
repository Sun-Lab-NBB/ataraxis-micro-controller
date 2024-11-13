/**
 * @file
 * @brief The header file for the SolenoidValveModule class, which allows interfacing with a solenoid valve to control
 * gravity-assisted fluid delivery.
 */

#ifndef AXMC_SOLENOID_VALVE_MODULE_H
#define AXMC_SOLENOID_VALVE_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Interfaces with a controllable solenoid water or gas valve by sending DC PWM current to the valve.
 *
 * This module is specifically designed to deliver microliter-precise amounts of fluid via gravity-assisted solenoid
 * valve.
 *
 *
 * @tparam kPin the analog pin connected to the valve trigger. Depending on the command, the pin will be used to
 * output either digital or analog signals. Therefore, the pin has to be capable of analog (pwm-driven) output.
 * @tparam kNormallyClosed determines whether the connected valve is opened or closed when unpowered. In turn, this is
 * used to adjust the class behavior so that the PWM signal of 0 always means valve is closed and PWM signal of 255
 * always means valve is opened.
 * @tparam kStartClosed determines whether the valve is opened when this class initializes its hardware. This is
 * used as a safety feature to configure the initial state of the valve following microcontroller setup or
 * reset.
 */
template <const uint8_t kPin, const bool kNormallyClosed, const bool kStartClosed>
class SolenoidValveModule final : public Module
{
        // Ensures that the pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different kPin for SolenoidValveModule class."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kLocked = 54,  ///< The Logic and PWM pins are in a global locked state and cannot be activated.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kOpenValve           = 1,  ///< Permanently opens the controlled solenoid.
            kCloseValve          = 2,  ///< Permanently closes the controlled solenoid.
            kPulseValve          = 3,  ///< Pulses the valve to deliver a precise amount of fluid.
            kCalibratePulseLevel = 4   ///< Carries out kPulseValve exactly 1000 times in a row without delays.
        };

        /// Initializes the class by subclassing the base Module class.
        SolenoidValveModule(
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

            _previous_input_status = 2;  // 2 is not a valid status, valid status codes are 0 and 1 (LOW and HIGH).

            return true;
        }

        ~SolenoidValveModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 10000;  ///< The time, in microseconds, for the HIGH phase of emitted pulses.
        } _custom_parameters;

        /// Sends a digital pulse through the output pin, using the preconfigured pulse_duration of microseconds.
        /// Supports non-blocking execution and respects the global ttl_lock state.
        void Pulse()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
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
                }
            }

            // HIGH phase delay. This delays for the requested number of microseconds before inactivating the pulse
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
        void Open()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
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
        void Close()
        {
            // Calling this command when the class is configured to receive TTL pulses triggers an invalid
            // pin mode error.
            if (!kOutput)
            {
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kInvalidPinMode));
                AbortCommand();  // Aborts the current and all future command executions.
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
};

#endif  //AXMC_SOLENOID_VALVE_MODULE_H
