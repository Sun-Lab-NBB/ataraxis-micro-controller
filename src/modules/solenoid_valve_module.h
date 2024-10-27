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
 * @attention This class is expected to work through 5-35V L298N H-bridge relays available from many electronic vendors
 * to power the solenoid vale. Wiring the controller directly to the solenoid will likely not work and may damage the
 * controller.
 *
 * @tparam kPinA the digital pin connected to the A logic terminal of the H-bridge relay.
 * @tparam kPinB the digital pin connected to the B logic terminal of the H-bridge relay.
 * @tparam kPWMPin the analog pin connected to the PWM terminal of the H-bridge relay.
 * @tparam kNormallyClosed determines whether the controlled solenoid is closed when the power is off (true) or opened
 * (false).
 */
template <const uint8_t kPinA, const uint8_t kPinB, const uint8_t kPWMPin, const bool kNormallyClosed>
class SolenoidValveModule final : public Module
{
        static_assert(
            kPinA != kPinB != kPWMPin,
            "The SolenoidValveModule digital and analog logic pins cannot be the same."
        );

        static_assert(
            kPinA != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Please select a different "
            "pin A for SolenoidValveModule class."
        );

        static_assert(
            kPinB != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Please select a different "
            "pin B for SolenoidValveModule class."
        );

        static_assert(
            kPinB != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Please select a different "
            "PWM pin for SolenoidValveModule class."
        );

    public:

        /// Assigns meaningful names to byte status-codes used to track module states. Note, this enumeration has to
        /// use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes enumeration inherited from base
        /// Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kStandBy = 51,  ///< The code used to initialize custom status trackers.
            kOpen    = 52,  ///< The managed solenoid valve is open.
            kClosed  = 53,  ///< The managed solenoid valve is closed.
            kLocked  = 54,  ///< The Logic and PWM pins are in a global locked state and cannot be activated.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kOpenValve           = 1,  ///< Permanently opens the controlled solenoid.
            kCloseValve          = 2,  ///< Permanently closes the controlled solenoid.
            kPulseValve          = 3,  ///< Pulses the valve to deliver a precise amount of fluid.
            kCalibratePulseLevel = 4   ///< Carries out kPulseValve exactly 1000 times in a row without delays.
        };

        /// Initializes the TTLModule class by subclassing the base Module class.
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
            pinModeFast(kPinA, OUTPUT);
            pinModeFast(kPinB, OUTPUT);
            pinModeFast(kPWMPin, OUTPUT);

            // If the solenoid is normally closed, sets both logic pins off to disable voltage input to the solenoid
            if (kNormallyClosed)
            {
                digitalWriteFast(kPinA, LOW);
                digitalWriteFast(kPinB, LOW);
                analogWrite(kPWMPin, 256);
            }

            // For normally open valve,
            else
            {
                digitalWriteFast(kPinA, HIGH);
                digitalWriteFast(kPinB, LOW);
                analogWrite(kPWMPin, 256);
            }
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kSetupComplete);
            return true;
        }

        /// Resets the custom_parameters structure fields to their default values.
        bool ResetCustomAssets() override
        {
            _custom_parameters.pulse_duration = 10000;  // The default output pulse duration is 10 milliseconds;
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kModuleAssetsReset);  // Records the status
            return true;
        }

        ~SolenoidValveModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                uint32_t pulse_duration = 10000;  ///< The time, in microseconds, the valve is kept open during pulses.
        } _custom_parameters;

        OpenValve
};

#endif  //AXMC_SOLENOID_VALVE_MODULE_H
