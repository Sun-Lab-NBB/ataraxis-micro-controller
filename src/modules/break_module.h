/**
 * @file
 * @brief The header-only file for the BreakModule class. This class allows interfacing with a break to dynamically
 * control the motion of break-coupled objects.
 *
 * @section brk_mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - module.h for the shared Module class API access (integrates the custom module into runtime flow).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_BREAK_MODULE_H
#define AXMC_BREAK_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "axmc_shared_assets.h"
#include "module.h"

/**
 * @brief Sends Pulse-Width-Modulated (PWM) signals to variably engage the managed break.
 *
 * This module is specifically designed to send PWM signals that trigger Field-Effect-Transistor (FET) gated relay
 * hardware to deliver voltage that variably engages the break. Depending on configuration, this module is designed to
 * work with both Normally Engaged (NE) and Normally Disengaged (ND) breaks.
 *
 * @tparam kPin the analog pin connected to the break FET-gated relay. Depending on the active command, the pin will be
 * used to output either digital or analog PWM signal to engage the break.
 * @tparam kNormallyEngaged determines whether the managed break is engaged (active) or disengaged (inactive) when
 * unpowered. This is used to adjust the class behavior so that toggle OFF always means the break is disabled and
 * toggle ON means the break is engaged at maximum strength.
 * @tparam kStartEngaged determines the initial state of the break during class initialization. This works
 * together with kNormallyEngaged parameter to deliver the desired initial voltage level for the break to either be
 * maximally engaged or completely disengaged after hardware configuration.
 */
template <const uint8_t kPin, const bool kNormallyEngaged, const bool kStartEngaged = true>
class BreakModule final : public Module
{
        // Ensures that the output pin does not interfere with LED pin.
        static_assert(
            kPin != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different pin for BreakModule instance."
        );

    public:
        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kOutputLocked = 51,  ///< The break pin is in a global locked state and cannot be used to output signals.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kToggleOn  = 1,  ///< Sets the break to permanently engage at maximum strength.
            kToggleOff = 2,  ///< Sets the break to permanently disengage.
            kSetBreakingPower =
                3,  ///< Sets the break to engage with the strength determined by breaking_strength parameter.
        };

        /// Initializes the class by subclassing the base Module class.
        BreakModule(
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
                // EnableBreak
                case kModuleCommands::kToggleOn: EnableBreak(); return true;
                // DisableBreak
                case kModuleCommands::kToggleOff: DisableBreak(); return true;
                // SetBreakingPower
                case kModuleCommands::kSetBreakingPower: SetBreakingPower(); return true;
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
            if (kStartEngaged) digitalWriteFast(kPin, kNormallyEngaged ? LOW : HIGH);  // Ensures the break is engaged.
            else digitalWriteFast(kPin, kNormallyEngaged ? HIGH : LOW);  // Ensures the break is disengaged.

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.breaking_strength = 128;  //  50% breaking strength

            return true;
        }

        ~BreakModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint8_t breaking_strength = 128;  ///< Determines the strength of the break when it uses the PWM mode.
        } PACKED_STRUCT _custom_parameters;

        /// Depending on the break configuration, stores the digital signal that needs to be sent to the output pin to
        /// engage the break at maximum strength.
        static constexpr bool kEngage = kNormallyEngaged ? HIGH : LOW;  // NOLINT(*-dynamic-static-initializers)

        /// Depending on the break configuration, stores the digital signal that needs to be sent to the output pin to
        /// disengage the break.
        static constexpr bool kDisengage = kNormallyEngaged ? LOW : HIGH;  // NOLINT(*-dynamic-static-initializers)

        /// Sets the Break to be continuously engaged (enabled) by outputting the appropriate digital signal.
        void EnableBreak()
        {
            // Engages the break
            if (DigitalWrite(kPin, kEngage, false)) CompleteCommand();
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
            // Disengages the break
            if (DigitalWrite(kPin, kDisengage, false)) CompleteCommand();
            else
            {
                // If writing to Action pins is globally disabled, as indicated by DigitalWrite returning false,
                // sends an error message to the PC and aborts the runtime.
                SendData(static_cast<uint8_t>(kCustomStatusCodes::kOutputLocked));
                AbortCommand();  // Aborts the current and all future command executions.
            }
        }

        /// Flexibly sets the breaking power by sending a square wave pulse with a certain PWM duty-cycle.
        void SetBreakingPower()
        {
            // Resolves the PWM value depending on whether the break is normally engaged (powered on when input current
            // is LOW) or not (powered on when input current is HIGH).
            uint8_t value = _custom_parameters.breaking_strength;  // Initial PWM is determined by break_strength.

            // Normally engaged break strength is inversely proportional to PWM value. This ensures that a PWM of 255
            // means the break is fully engaged regardless of whether it is NE or ND.
            if (kNormallyEngaged) value = 255 - value;

            // Uses AnalogWrite to make the pin output a square wave pulse with the desired duty cycle (PWM). This
            // results in the breaks being applied a certain proportion of time, producing the desired breaking power.
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
