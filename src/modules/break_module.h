/**
 * @file
 * @brief The header file for the BreakModule class, which is sued to variably control an electromagnetic breaking
 * system used to tune circular motion.
 */

#ifndef AXMC_BREAK_MODULE_H
#define AXMC_BREAK_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Sends Pulse-Width Modulated (PWM) signals to variably engage the connected breaking system.
 *
 * This module is specifically designed to send and receive PWM signals that trigger Field-Effect Transistor (FET)
 * gated relay hardware to deliver voltage to power up and engage the breaking system. Depending on configuration, this
 * module is designed to work with both Normally Closed (NC) and Normally Open (NO) breaking systems.
 *
 * @tparam kPin the analog pin connected to the break trigger. Depending on the command, the pin will be used to
 * output either digital or analog signals. Therefore, the pin has to be capable of analog (pwm-driven) output.
 * @tparam kNormallyEngaged determines whether the connected break system is engaged (active) or disengaged (inactive)
 * when unpowered. In turn, this is used to adjust the class behavior so that the PWM signal of 0 always means
 * break is disengaged and PWM signal of 255 always means break is engaged.
 * @tparam kStartEngaged determines whether the break system activates when this class initializes its hardware. This is
 * used as a safety feature to configure the initial state of the break system following microcontroller setup or
 * reset.
 */
template <const uint8_t kPin, const bool kNormallyEngaged, const bool kStartEngaged = true>
class BreakModule final : public Module
{
        // Ensures that the output pin does not interfere with LED pin.
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
            kOutputLocked = 51,  ///< The output pin is in a global locked state and cannot be used to output data.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kEnable   = 1,
            kDisable  = 2,
            kSetPower = 3,
        };

        /// Initializes the class by subclassing the base Module class.
        BreakModule(
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
                // EnableBreak
                case kModuleCommands::kEnable: EnableBreak(); return true;
                // DisableBreak
                case kModuleCommands::kDisable: DisableBreak(); return true;
                // SetPower
                case kModuleCommands::kSetPower: SetPower(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Sets pin mode to OUTPUT
            pinModeFast(kPin, OUTPUT);

            // Based on the requested initial break state and the configuration of the break (normally engaged or not),
            // either engages or disengages the breaks following setup.
            if (kStartEngaged)
                digitalWriteFast(kPin, kNormallyEngaged ? LOW : HIGH);   // If engaged when low, low to engage.
            else digitalWriteFast(kPin, kNormallyEngaged ? HIGH : LOW);  // If engaged when low, high to disengage.

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.pwm_strength = 255;  // The default output PWM is to engage the breaks.

            return true;
        }

        ~BreakModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint8_t pwm_strength = 0;  ///< The PWM value of the square pulse wave to send to the break trigger.
        } __attribute__((packed)) _custom_parameters;

        /// Sets the Break to be continuously engaged (enabled) by outputting the appropriate digital signal.
        void EnableBreak()
        {
            // Resolves the signal to send to the break. Specifically, this depends on whether the break is normally
            // engaged (powered on when input current is LOW) or not (powered off when input current is HIGH).
            bool value;
            if (kNormallyEngaged) value = LOW;  // To enable normally engaged break, the power has to be LOW.
            else value = HIGH;

            // Sets the break-connected pin to output the necessary signal to engage the break.
            if (DigitalWrite(kPin, value, false)) CompleteCommand();
            else
            {
                // If writing to Action pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Sets the Break to be continuously disengaged (disabled) by outputting the appropriate digital signal.
        void DisableBreak()
        {
            // Resolves the signal to send to the break. Specifically, this depends on whether the break is normally
            // engaged (powered on when input current is LOW) or not (powered off when input current is HIGH).
            bool value;
            if (kNormallyEngaged) value = HIGH;  // To disable normally engaged break, the power has to be HIGH.
            else value = LOW;

            // Sets the break-connected pin to output the necessary signal to disengage the break.
            if (DigitalWrite(kPin, value, false)) CompleteCommand();
            else
            {
                // If writing to Action pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Variably adjusts the breaking power from none to maximum by sending a square wave pulse with a certain PWM
        /// cycle.
        void SetPower()
        {
            // Resolve the PWM value depending on whether the break is normally engaged (powered on when input current
            // is LOW) or not (powered off when input current is HIGH).
            uint8_t value = _custom_parameters.pwm_strength;
            if (kNormallyEngaged) value = 255 - value;  // Converts the value so that 255 always means breaks are ON.

            // Uses AnalogWrite to make the pin output a hardware-module square wave pulse. This results in the breaks
            // being applied a certain proportion of time, rather than all or none of the time, resulting in variable
            // breaking power.
            if (AnalogWrite(kPin, value, false)) CompleteCommand();
            else
            {
                // If writing to Action pins is globally disabled, as indicated by AnalogWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }
};

#endif  //AXMC_BREAK_MODULE_H
