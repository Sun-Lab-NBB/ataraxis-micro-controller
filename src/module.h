/**
 * @file
 * @brief The header file for the Core AMCModule Core class, which is used as a parent for all custom Module classes to
 * automatically integrate them with the AMCKernel runtime control system.
 *
 * @subsection description Description:
 *
 * @note Every custom Module class should inherit from this class. This class is used to provide a static
 * interface that AMCKernel can use to integrate any class with the rest of the runtime code. Additionally, it shares
 * a number of utility functions that can be used to simplify custom Module development via the use of safe,
 * well-integrated methods that abstract away from low-level functionality like direct pin manipulation or time / sensor
 * threshold waiting.
 *
 * @attention It is highly advised to check one of the test_classes available through AMCKernel library (@em not
 * amc_kernel file!). These classes showcase the principles of constructing custom classes using the available base
 * class methods. Furthermore, it may be beneficial to check the test_AMC_code.cpp file to see the expected and
 * error-triggering class usage patters.
 *
 * This file contains the following items:
 * - The definition of the AMCModule base class with three broad kinds of methods. Utility methods which are designed to
 * be used by the custom commands of children classes to simplify command code writing. Integration methods that are
 * reserved for AMCKernel class usage, as they provide the static interface through which AMCKernel can use static
 * Core resources implemented inside the AMCModule class and inherited by all children classes. Virtual methods, which
 * are used for the same purpose as Integration methods, but provide an interface for the AMCKernel class to access
 * and manage custom commands and assets individually written by each Module-level class developer for that specific
 * Module.
 * - The definition of the kExecutionRuntimeParameters structure, which is a runtime-critical Core structure used to
 * queue and execute commands. It supports running commands in blocking and non-blocking modes and executing them
 * instantaneously (once) or recurrently. That structure should only be modified by AMCKernel developers, if ever and
 * any call to the structure has been abstracted using utility method, so module developers can use utility methods
 * without worrying about the functioning of the core structure.
 * - The definition of the kExecutionRuntimeParameterCodes enumeration, which stores the id_codes for the PC-addressable
 * fields of the kExecutionRuntimeParameters structure. This is a critical enumeration that is used to control the
 * proper modification of PC-addressable parameters in the kExecutionRuntimeParameters structure. Again, this structure
 * should only be modified by Kernel developers.
 * - kReturnedUtilityCodes, an enumeration that lists all byte-codes returned by Utility methods, so that custom
 * commands can interpret and handle each returned code appropriately. This provides an alternative interface to using
 * 'magic numbers', improving ht maintainability fo the codebase.
 * - Other internal trackers and variables used to integrate any class that inherits from the AMCModule base class with
 * the AMCKernel runtime control system.
 *
 * @subsection developer_notes Developer Notes:
 * This is one of the key Core level classes that is critically important for the functioning of the whole AMC codebase.
 * Generally, only AMC Kernel developers with a good grasp of the codebase should be modifying this class and any such
 * modifications should come with ample deprecation warnings and versioning. This is especially relevant for class
 * versions that modify existing functionality and, as such, are likely to be incompatible with existent custom Modules.
 *
 * There are two main ways of using this class to develop custom Modules. The preferred way is to rely on the available
 * utility methods provided by this class to abstract all interactions with the inherited Core methods and variables.
 * Specifically, any class inheriting from this base class should use the methods in the utility sections inside all
 * custom commands and overridden virtual methods where appropriate. The current set of utility methods should be
 * sufficient to support most standard microcontroller use cases, while providing automatic compatibility with the rest
 * of the AMC codebase. As a bonus, this method is likely to improve code maintainability, as utility methods are
 * guaranteed to be supported and maintained even if certain Core elements in the class are modified by Kernel library
 * developers.
 *
 * The second way, which may be useful if specific required functionality is not available through the utility methods,
 * is to directly use the included core structures and variables, such as the ExecutionControlParameters. This requires
 * a good understanding of how the base class and it's method function, as well as an understanding of how AMCKernel
 * works and interacts with AMCModule-derived classes. With sufficient care, this method cab provide more control over
 * the behavior of any custom Module, at the expense of being more verbose and less safe.
 *
 * Regardless of the use method, any custom Module inheriting from the AMCModule class has to implement the following
 * purely virtual methods to become fully compatible with AMCKernel-mediated runtime control system:
 * - SetCustomParameters: For the kernel to properly write any Parameter structures received from the PC to the target
 * custom (class-specific) parameter structures or variables.
 * - RunActiveCommand: For the module to actually execute the custom command using the unique command byte-code stored
 * in the 'command' field of the ExecutionControlParameters structure (active command).
 * - SetupModule: For the module to be able to properly set up its pins, default values and otherwise prepare for the
 * runtime cycle.
 * ResetCustomAssets: For the kernel to properly reset all custom (class-specific) assets, such as parameters, variables
 * and states, before starting a new runtime cycle.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - shared_assets.h for globally shared static message byte-codes and the ControllerRuntimeParameter structure.
 * - communication.h for Communication class, which is used to send module runtime data to the connected system.
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - elapsedMillis.h for the
 *
 * @see amc_module.cpp for method implementation.
 */

#ifndef AMC_MODULE_H
#define AMC_MODULE_H

#include "Arduino.h"
#include "communication.h"
#include "digitalWriteFast.h"
#include "elapsedMillis.h"
#include "shared_assets.h"

/**
 * @brief A major Core-level class that serves as the parent for all Module- level classes and provides them with a set
 * of core utilities and methods necessary to automatically integrate with the rest of the AMC codebase.
 *
 * This class serves as the shared (via inheritance) repository of utility and runtime-control methods that allow to
 * automatically integrate any custom Module into the existing AMC codebase. Specifically, by inheriting from this
 * class, any Module automatically gains access to the methods that queue and execute Module commands based on the
 * input from the PC. Additionally, this allows the AMCKernel to interact with any custom module using the virtual and
 * non-virtual methods inherited from the AMCModule class, effectively embedding any correctly constructed module into
 * the AMC runtime flow control structure.
 *
 * @note This class offers a collection of utility methods that should preferentially be used when writing custom
 * functions. These method abstract the interactions with Core structures that are used to enable many of the distinct
 * AMC codebase functions such as concurrent (non-blocking) execution of many different commands at the same time and
 * error handling. Be sure to check the documentation for all Utility (in contrast to Integration or Virtual) and the
 * examples available through test classes of AMCKernel library prior to writing custom functions.
 *
 * @warning Every Module class @b has to inherit from this base class to be compatible with the rest of the AMC
 * codebase. Moreover, the base class itself uses pure virtual methods and, as such, cannot be instantiated. Only a
 * child class that properly overrides all pure virtual methods of the base class can be instantiated.
 */
class Module
{
    public:
        /**
         * @struct ExecutionControlParameters
         * @brief Stores parameters that are used to dynamically queue, execute and control the execution flow of commands
         * for each Module-level class instance.
         *
         * An instance of this structure is used by the base AMCModule Core class instance from which all custom modules are
         * expected to inherit. All runtime control manipulations that involve changing the variables inside this structure
         * should be carried out via the methods available through the base AMCModule Core class via inheritance.
         *
         * @attention Generally, any modification to this structure or code that makes use of this structure should be
         * reserved for developers with a good grasp of the existing codebase and, if possible, avoided (especially
         * deletions or refactoring of existing variables). More specifically, this should be handed off to AMC project
         * kernel developers when possible.
         */
        struct ExecutionControlParameters
        {
                uint8_t command      = 0;      ///< Currently active (in-progress) command
                uint8_t stage        = 0;      ///< Stage of the currently active command
                bool noblock         = false;  ///< Specifies if the currently active command is blocking
                uint8_t next_command = 0;      ///< A buffer that allows to queue the next command to be executed
                bool next_noblock    = false;  ///< A buffer to store the next noblock flag value
                bool new_command     = false;  ///< Tracks whether next_command is a new or recurrent command
                bool run_recurrently = false;  ///< Specifies whether the queued command runs once and clears or recurs
                uint32_t recurrent_delay = 0;  ///< The minimum amount of microseconds between recurrent command calls
                elapsedMicros recurrent_timer =
                    0;                          ///< A timer class instance to time recurrent command activation delays
                elapsedMicros delay_timer = 0;  ///< A timer class instance to time delays between active command stages
        } execution_parameters;

        // Stores the most recent module status code. This variable is used to communicate the runtime status of the Module,
        // which is used by the Kernel to more effectively communicate any Module errors to the PC and to interface with
        // the Module.
        uint8_t module_status = 0;

        /**
         * @brief Instantiates a new AMCModule Core class objects.
         *
         * @param module_type_id This ID is used to identify module types (classes) during data communication. Each class
         * has a custom set of parameters and commands, but is very likely to use the same parameter and command byte-codes
         * as other classes. Module type IDs solve the issue that would otherwise arise when the same byte-codes are used by
         * different classes with radically different implications, by ensuring any communicated byte-codes are interpreted
         * according to class-specific code enumerations. The IDs are assigned dynamically by the user when configuring the
         * Controller (and PC, they have to match!) runtime layout. This allows to solve the problem that would eventually
         * arise for custom modules, where different developers might have reused the same IDs for different modules.
         * @param module_instance_id This ID is used to identify the specific instance of the module_type_id-specified
         * class. This allows to address specific class instances, which are often mapped to particular physical system.
         * Eg: If module_type_id of 1 corresponds to the 'Door' class, module_instance_id of 1 would mean "Kitchen Door" and
         * 2 would mean "Bathroom Door".
         * @param controller_runtime_parameters A constant reference to the ControllerRuntimeParameters structure instance
         * provided by the AMCKernel class. There should only be one such instance and the only class allowed to modify that
         * instance is the AMCKernel class itself. All Module classes take this instance by reference as runtime parameters
         * control many aspects of Module runtime behavior.
         * @param serial_communication_port A reference to the SerialPCCommunication class instance shared by all AMC
         * codebase classes that receive (AMCKernel only!) or send (AMCKernel and all Modules) data over the serial port.
         * Modules use this class instance to send event (and error) data over to the PC.
         */
        Module(
            uint8_t module_type_id,
            uint8_t module_instance_id,
            Communication<shared_assets::kStaticRuntimeParameters.controller_buffer_size>& serial_communication_port
        ) :
            module_type(module_type_id),
            module_id(module_instance_id),
            runtime_parameters(controller_runtime_parameters),
            communication_port(serial_communication_port)
        {}

        // Declares non-virtual utility methods. These methods should not be overwritten by derived classes, and they are
        // primarily designed to help developers write custom module commands by providing standardized high-level access to
        // the underlying Core structures derived from the AMCModule class.

        /**
         * @brief Polls and (optionally) averages the value(s) of the requested analog sensor pin and returns the resultant
         * raw analog readout value to the caller.
         *
         * This is a non-virtual utility method that is available to all derived classes.
         *
         * @note This method should be used for all analog pin reading tasks as it automatically supports the use of
         * value simulation subroutines, which is considered a standard property for all AMC sensors.
         *
         * @param pin The number of the Analog pin to be polled (in simulation mode will be ignored internally).
         * @param pool_size The number of pin readout values to average to produce the final readout (also ignored in
         * simulation mode). Setting to 0 or 1 means no averaging is performed, 2+ means averaging is performed.
         *
         * @returns uint16_t value of the analog sensor. Currently uses 16-bit resolution as the maximum supported by analog
         * pin hardware.
         */
        static uint16_t GetRawAnalogReadout(
            const uint8_t pin,        // The analog pin to be polled
            const uint16_t pool_size  // The number of pin poll values to average to produce the final readout
        )
        {
            uint16_t average_readout;  // Pre-declares the final output readout as an unit16_t

            // Pool size 0 and 1 mean essentially the same: no averaging, so lumps the check into < 2
            if (pool_size < 2)
            {
                // If averaging is disabled, simply reads and outputs the acquired value. An optimization for teensies can
                // be disabling software averaging and relying on hardware averaging instead of using this execution path
                average_readout = analogRead(pin);
            }
            else
            {
                uint32_t accumulated_readouts = 0;  // Aggregates polled values by self-addition

                // If averaging is enabled, repeatedly polls the pin the requested number of times.
                // Note, while this can be further optimized on teensies by manipulating pin averaging, this is not
                // available for Arduino's, so a more cross-platform-friendly, but less optimal procedure is used here.
                // NOTE, i's type always has to be the same as pool size (currently uint16_t), hence why the auto and
                // decltype check below.
                for (auto i = decltype(pool_size) {0}; i < pool_size; i++)
                {
                    accumulated_readouts += analogRead(pin);  // Aggregates readouts via self-addition
                }

                // Here the averaging and rounding is performed to obtain an integer value rather than dealing with floating
                // point arithmetic. This is another Arduino-favoring optimization as teensy boards come with a hardware FP
                // module that removes a lot of the speed loss associated with FP arithmetic on Arduino boards.
                // Note, adding pool_size/2 before dividing by pool_size forces half-up ('standard') rounding.
                // Also, explicitly ensures that the final value is cast to uint16_t.
                average_readout = static_cast<uint16_t>((accumulated_readouts + pool_size / 2) / pool_size);
            }

            return average_readout;
        }

        /**
         * @brief Polls and (optionally) averages the value(s) of the requested digital sensor pin and returns the resultant
         * pin state (HIGH or LOW) to caller.
         *
         * This is a non-virtual utility method that is available to all derived classes.
         *
         * @note This method should be used for all digital pin reading tasks as it automatically supports the use of
         * value simulation subroutines, which is considered a standard property for all AMC sensors.
         *
         * @param pin The number of the Digital pin to be polled (in simulation mode will be ignored internally).
         * @param pool_size The number of pin readout values to average to produce the final readout (also ignored in
         * simulation mode). Setting to 0 or 1 means no averaging is performed, 2+ means averaging is performed.
         *
         * @returns bool @b true (HIGH) or @b false (LOW), depending on the state of the input pin.
         */
        static bool GetRawDigitalReadout(
            const uint8_t
                pin,  // The digital pin to read. Has to be constant and set to read for the fast reading to work
            const uint16_t pool_size  // The number of pin poll values to average to produce the final readout
        )
        {
            bool digital_readout;  // Pre-declares the final output readout as a boolean

            // Otherwise reads the physical sensor value. Uses pooling as an easy way to implement custom debouncing logic,
            // which often relies on averaging multiple digital readouts to confirm that the pin is stably active or inactive.
            // Generally, this is a built-in hardware feature for many boards, but the logic is maintained here for backward
            // compatibility reasons.
            // Pool size 0 and 1 mean essentially the same: no averaging, so lumps the check into < 2
            if (pool_size < 2)
            {
                // If averaging is disabled, simply reads and outputs the acquired value
                digital_readout = digitalReadFast(pin);
            }
            else
            {
                uint32_t accumulated_readouts = 0;  // Aggregates polled values by self-addition

                // If averaging is enabled, repeatedly polls the pin the requested number of times.
                // NOTE, i's type always has to be the same as pool size (currently uint16_t), hence why the auto and
                // decltype check below.
                for (auto i = decltype(pool_size) {0}; i < pool_size; i++)
                {
                    accumulated_readouts += digitalReadFast(pin);  // Aggregates readouts via self-addition
                }

                // Here the averaging and rounding is performed to obtain an integer value rather than dealing with floating
                // point arithmetic. This is another Arduino-favoring optimization as teensy boards come with a hardware FP
                // module that removes a lot of the speed loss associated with FP arithmetic on Arduino boards.
                // Note, adding pool_size/2 before dividing by pool_size forces half-up ('standard') rounding.
                // Also, explicitly ensures that the final value is cast to boolean true or false.
                digital_readout = static_cast<bool>((accumulated_readouts + pool_size / 2) / pool_size);
            }

            return digital_readout;
        }

        /**
         * @brief Checks if the delay_duration of microseconds has passed since the module's ExecutionControlParameters
         * structure's delay_timer field has been reset.
         *
         * Depending on execution configuration, the function can block in-place until the escape duration has passed or be
         * used as a simple check of whether the required duration of microseconds has passed.
         *
         * This is a non-virtual utility method that is available to all derived classes.
         *
         * @note It is encouraged to use this method for all delay-related tasks (and implement a noblock method design to
         * support concurrent task execution if required). This is because this method properly handles the use of value
         * simulation subroutines, which are considered a standard property for all AMC delay methods.
         *
         * @param delay_duration The duration, in @em microseconds the function should delay / check for.
         *
         * @returns bool @b true if the delay has been successfully enforced (either via blocking or checking) and @b false
         * otherwise. If the elapsed duration equals to the delay, this is considered a passing condition.
         */
        [[nodiscard]]
        bool WaitForMicros(
            const uint32_t delay_duration  // The duration that this function should check for, in microseconds
        ) const
        {
            // If the caller command is executed in blocking mode, blocks in-place (inferred from the noblock flag)
            if (!execution_parameters.noblock)
            {
                // Blocks until delay_duration has passed
                while (execution_parameters.delay_timer <= delay_duration)
                    ;

                return true;  // Returns true, the duration has been enforced
            }

            // Otherwise, if the caller function is executed in non-blocking mode, checks if the minimum duration has passed
            // and returns the check condition
            return (execution_parameters.delay_timer >= delay_duration);
        }

        /**
         * @brief Checks if the evaluated @b Analog sensor pin detects a value that is equal to or exceeds the trigger
         * threshold.
         *
         * @note Trigger threshold is specified using a concrete threshold value and the comparison is carried out either
         * using the >= (default) or <= operator. As such, the value can exceed the threshold by being greater than or equal
         * to the threshold or less than or equal to the threshold, which is specified via the invert_condition flag.
         *
         * Depending on configuration, the function can block in-place until the escape condition is met or be used as a
         * simple check of whether the condition is met.
         *
         * This is a non-virtual utility method that is available to all derived classes.
         *
         * @note It is encouraged to use this method for all analog sensor-threshold related tasks (and implement a noblock
         * method design to support concurrent task execution if required). This is because this method properly handles the
         * use of value simulation subroutines, which are considered a standard property for all AMC sensor
         * threshold-related methods.
         *
         * @attention Currently only supports open-ended threshold. The rationale here is that even with the best analog
         * resolution analog sensors will fluctuate a little and having a concrete (==) threshold comparison would very
         * likely not work as intended. Having a one-side open-ended threshold offers the best of both worlds, where the
         * value still has to be within threshold boundaries, but also can be fluctuate above or below it and still
         * register.
         *
         * @param sensor_pin The number of the analog pin whose' readout value will be evaluated against the threshold.
         * @param invert_condition A boolean flag that determines whether the default comparison operator (>=, false) is
         * inverted to (<=, true).
         * @param threshold The concrete threshold value against which the parameter will be evaluated. Regardless of the
         * operator, if pin readout equals to threshold, the function will return @b true.
         * @param timeout The number of microseconds after the function breaks and returns kWaitForAnalogThresholdTimeout,
         * this is necessary to prevent soft-blocking.
         * @param pool_size The number of sensor readouts to average into the final value.
         *
         * @returns uint8_t kWaitForAnalogThresholdFailure byte-code if the condition was not met and
         * kWaitForAnalogThresholdPass if it was met. Returns kWaitForAnalogThresholdTimeout if timeout duration has
         * been exceeded (to avoid soft-blocking). All byte-codes should be interpreted using the kReturnedUtilityCodes
         * enumeration.
         */
        uint8_t WaitForAnalogThreshold(
            const uint8_t sensor_pin,     // The input pin (number) of the sensor to be evaluatedsensor to be evaluated
            const bool invert_condition,  // Allows inverting the default >= comparison to use <= operator instead
            const uint16_t threshold,     // The value of the sensor that is considered 'passing', depending on operator
            const uint32_t timeout,       // The number of microseconds after which method returns code 13 (times-out)
            const uint16_t pool_size      // The number of sensor readouts to average to give the final readout
        )
        {
            uint16_t value;  // Precreates the variable to store the sensor value
            bool passed;     // Tracks whether the threshold check is passed

            uint32_t min_delay = 0;  // Pre-initializes to 0, which disables the minimum delay if simulation is not used
            if (runtime_parameters.simulate_responses)
            {
                // If simulate_responses flag is true, uses minimum_lock_duration to enforce the preset minimum delay
                // duration
                min_delay = runtime_parameters.minimum_lock_duration;
            }

            // Obtains the real or simulated sensor value (the function comes with built-in simulation resolution)
            value = GetRawAnalogReadout(sensor_pin, pool_size);

            if (!invert_condition)
            {
                // If inversion is disabled, passing condition is value being equal to or greater than the threshold
                // Also factors in the minimum delay as otherwise it is not properly assessed when the method runs in non-blocking
                // mode
                passed = (value >= threshold) && (execution_parameters.delay_timer > min_delay);
            }
            else
            {
                // If inversion is enabled, passing condition is value being less than or equal to the threshold
                // Also factors in the minimum delay as otherwise it is not properly assessed when the method runs in non-blocking
                // mode
                passed = (value <= threshold) && (execution_parameters.delay_timer > min_delay);
            }

            // If the caller command is executed in blocking mode, blocks in-place until either a timeout duration of
            // microseconds passes or the sensor value exceeds the threshold
            if (!execution_parameters.noblock)
            {
                // This blocks until BOTH the min_delay has passed AND the sensor value exceeds the threshold
                while (execution_parameters.delay_timer <= min_delay || !passed)
                {
                    // Obtains the sensor readout and checks it against the threshold on every iteration of the while loop
                    value = GetRawAnalogReadout(sensor_pin, pool_size);
                    if (!invert_condition)
                    {
                        passed = (value >= threshold);
                    }
                    else
                    {
                        passed = (value <= threshold);
                    }

                    // If the delay timer exceeds timeout value, interrupts and returns kWaitForAnalogThresholdTimeout (13),
                    // which is a special timeout-quit code
                    if (execution_parameters.delay_timer > timeout)
                    {
                        return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdTimeout);
                    }
                }

                // Returns kWaitForAnalogThresholdPass (12) to indicate that the check has passed
                return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdPass);
            }

            // Otherwise, if the caller function is executed in non-blocking mode, determines what code to return
            else
            {
                // If the delay timer exceeds timeout, returns kWaitForAnalogThresholdTimeout (13) to indicate timeout condition
                if (execution_parameters.delay_timer > timeout)
                    return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdTimeout);

                // Otherwise, if check passed, returns kWaitForAnalogThresholdPass (12)
                else if (passed) return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdPass);

                // Otherwise, returns kWaitForAnalogThresholdFailure (11)
                else return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdFailure);
            }
        }

        /**
         * @brief Checks if the evaluated @b Digital sensor pin detects a value that matches the trigger threshold.
         *
         * This is an equivalent of the WaitForAnalogThreshold with considerably streamlined logic due to the binary nature
         * of digital pin threshold checking.
         *
         * @note Unlike WaitForAnalogThreshold() method, this method uses exact thresholds. This is because it works with
         * digital sensors that generally do not fluctuate (bouncing noise aside) and, therefore, are a lot more reliable
         * for using them with a concrete threshold.
         *
         * Depending on configuration, the function can block in-place until the escape condition is met or be used as a
         * simple check of whether the condition is met.
         *
         * This is a non-virtual utility method that is available to all derived classes.
         *
         * @note It is encouraged to use this method for all digital sensor-threshold related tasks (and implement a noblock
         * method design to support concurrent task execution if required). This is because this method properly handles the
         * use of value simulation subroutines, which are considered a standard property for all AMC sensor
         * threshold-related methods.
         *
         * @param sensor_pin The number of the pin whose' readout value will be evaluated against the threshold.
         * @param threshold A boolean @b true or @b false which is used as a threshold for the pin value. Passing condition
         * is evaluated using == operator, so the readout value has to match the threshold for the function to return @b
         * true.
         * @param timeout The number of microseconds after the function breaks and returns kWaitForDigitalThresholdTimeout,
         * this is necessary to prevent soft-blocking.
         * @param pool_size The number of sensor readouts to average into the final value. While not strictly necessary for
         * digital pins, the option is maintained to allow for custom debouncing logic (which is often conducted through
         * averaging).
         *
         * @returns uint8_t kWaitForDigitalThresholdFailure byte-code if the condition was not met and
         * kWaitForDigitalThresholdPass if it was met. Returns kWaitForDigitalThresholdTimeout if timeout duration has
         * been exceeded (to avoid soft-blocking). All byte-codes should be interpreted using the kReturnedUtilityCodes
         * enumeration.
         */
        uint8_t WaitForDigitalThreshold(
            const uint8_t sensor_pin,  // The input pin (number) of the digital sensor to be evaluated
            const bool threshold,      // The value of the sensor that is considered 'passing'
            const uint32_t timeout,    // The number of microseconds after which function returns code 13 (times-out)
            const uint16_t pool_size   // The number of sensor readouts to average to give the final readout
        )
        {
            bool value;  // Precreates the variable to store the sensor value

            uint32_t min_delay = 0;  // Pre-initializes to 0, which disables the minimum delay if simulation is not used
            if (runtime_parameters.simulate_responses)
            {
                // If simulate_responses flag is true, uses minimum_lock_duration to enforce the preset minimum delay
                // duration
                min_delay = runtime_parameters.minimum_lock_duration;
            }

            // Obtains the real or simulated sensor value (the function comes with built-in simulation resolution)
            value = GetRawDigitalReadout(sensor_pin, pool_size);

            // If the caller command is executed in blocking mode, blocks in-place until either a timeout duration of
            // microseconds passes or the sensor value matches the threshold
            if (!execution_parameters.noblock)
            {
                // This blocks until BOTH the min_delay has passed AND the sensor value exceeds the threshold
                while (execution_parameters.delay_timer <= min_delay || value != threshold)
                {
                    // Obtains the sensor readout and checks it against the threshold on every iteration of the while loop
                    value = GetRawDigitalReadout(sensor_pin, pool_size);

                    // If the delay timer exceeds timeout value, interrupts and returns kWaitForDigitalThresholdTimeout (16),
                    // which is a special timeout-quit code
                    if (execution_parameters.delay_timer > timeout)
                    {
                        return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdTimeout);
                    }
                }

                // Returns kWaitForDigitalThresholdPass (15) to indicate that the check has passed
                return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdPass);
            }

            // Otherwise, if the caller function is executed in non-blocking mode, determines what code to return
            else
            {
                // If the delay timer exceeds timeout, returns kWaitForDigitalThresholdTimeout (16) to indicate timeout
                // condition
                if (execution_parameters.delay_timer > timeout)
                    return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdTimeout);

                // Otherwise, if check passed, returns kWaitForDigitalThresholdPass (15)
                // Note, factors in the minimum delay to ensure it is properly assessed when the method runs in non-blocking
                // mode
                else if (value == threshold && execution_parameters.delay_timer >= min_delay)
                    return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdPass);

                // Otherwise, returns kWaitForDigitalThresholdFailure (14)
                else return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdFailure);
            }
        }

        /**
         * @brief Advances the stage of the currently running Module command.
         *
         * This is a simple wrapper interface that helps writing noblock-compatible commands by modifying the stage tracker
         * inside the ExecutionControlParameters structure by 1 each time this method is called. Use it to properly advance
         * noblock execution command stages
         *
         * @note See one of the test_class_1 from AMCKernel library for examples.
         */
        void AdvanceActiveCommandStage()
        {
            execution_parameters.stage++;
        }

        /**
         * @brief Extracts and returns the stage of the currently running Module command.
         *
         * This is a simple wrapper interface that helps writing noblock-compatible commands by providing convenient access
         * to the stage tracker field of the ExecutionControlParameters structure. Use it to extract and evaluate the stage
         * number to execute the proper portion of each noblock-compatible command.
         *
         * @returns uint8_t The stage number of the currently active command. The returned number is necessarily greater
         * than 0 for any active command and 0 if there is currently noa active command.
         */
        [[nodiscard]]
        uint8_t GetActiveCommandStage() const
        {
            // If there is an actively executed command, returns its stage
            if (execution_parameters.command != 0)
            {
                return execution_parameters.stage;
            }

            // Otherwise returns 0 to indicate there is no actively running command
            return 0;
        }

        /**
         * @brief Sets the command and stage fields of the ExecutionControlParameters structure to 0, indicating that the
         * command has been completed.
         *
         * @warning It is essential that this method is called at the end of every command execution to allow the Module to
         * execute other commands. Failure to do so can completely deadlock the Module and, if command is blocking, the
         * whole Controller.
         *
         * @note The 'end' here means the algorithmic end: when the command completes everything it sets out to do. For
         * noblock commands, the Controller may need to loop through the command code multiple times, so this methods call
         * should be protected by an 'if' statement that ensures it is only called when the command has finished it's work.
         */
        void CompleteCommand()
        {
            execution_parameters.command = 0;
            execution_parameters.stage   = 0;
        }

        /**
         * @brief Extracts and returns the byte-code of the currently active command stored in ExecutionControlParameters
         * structure.
         *
         * This is a simple wrapper interface primarily designed to assist writing RunActiveCommand() virtual method
         * implementation by providing easy access to the command-code that needs to be verified and executed by that
         * method.
         *
         * @returns uint8_t The byte-code of the currently active command. The returned number is necessarily greater than
         * 0 if a command is active and 0 if there is currently no active command.
         */
        [[nodiscard]]
        uint8_t GetActiveCommand() const
        {
            return execution_parameters.command;
        }

        /**
         * @brief Sets the input digital pin to the specified value (High or Low).
         *
         * This utility method is used to control the value of a digital pin configured to output a constant High or Low
         * level signal. Internally, the method handles the correct use of global runtime parameters that control whether
         * action and ttl pin activity is allowed.
         *
         * @note It is recommend to use this method for all digital pin control takes.
         *
         * @param pin The value of the digital pin to be set to the input value.
         * @param value The boolean value to set the pin to.
         * @param ttl_pin The boolean flag that determines whether the pin is used to drive active systems or for TTL
         * communication. This is necessary to properly comply with action_lock and ttl_lock field settings of the global
         * ControllerRuntimeParameters structure.
         *
         * @returns bool Current value of the pin (the value the pin has actually been set to).
         */
        [[nodiscard]]
        bool DigitalWrite(const uint8_t pin, const bool value, const bool ttl_pin) const
        {
            // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to LOW and returns
            // false to indicate that the pin is currently set to low
            if ((ttl_pin && runtime_parameters.ttl_lock) || (!ttl_pin && runtime_parameters.action_lock))
            {
                digitalWriteFast(pin, LOW);
                return false;
            }

            // If the pin is not locked, sets it to the appropriate value and returns the value to indicate that the pin has
            // been set to the target value
            digitalWriteFast(pin, value);
            return value;
        }

        /**
         * @brief Sets the input analog pin to the specified duty-cycle value (from 0 to 255).
         *
         * This utility method is used to control the value of aan analog pin configured to output a PWM-pulse with the
         * defined duty cycle from 0 (off) to 255 (always on). Internally, the method handles the correct use of global
         * runtime parameters that control whether action and ttl pin activity is allowed.
         *
         * @note Currently, the method only supports standard 8-bit resolution to maintain backward-compatibility with AVR
         * boards. In the future, this may be adjusted.
         *
         * @param pin The value of the analog pin to be set to the input value.
         * @param value A uint8_t duty cycle value from 0 to 255 that controls the proportion of time during which the pin
         * is On. Note, the exact meaning of each duty-cycle step depends on the clock that is used to control the analog
         * pin cycling behavior.
         * @param ttl_pin The boolean flag that determines whether the pin is used to drive active systems or for TTL
         * communication. This is necessary to properly comply with action_lock and ttl_lock field settings of the global
         * ControllerRuntimeParameters structure.
         *
         * @returns uint8_t Current value (duty cycle) of the pin (the value the pin has actually been set to).
         */
        [[nodiscard]]
        uint8_t AnalogWrite(const uint8_t pin, const uint8_t value, const bool ttl_pin) const
        {
            // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to 0 (permanently off) and
            // returns 0 to indicate that the pin is currently switched off (locked)
            if ((ttl_pin && runtime_parameters.ttl_lock) || (!ttl_pin && runtime_parameters.action_lock))
            {
                analogWrite(pin, 0);
                return 0;
            }

            // If the pin is not locked, sets it to the appropriate value and returns the value to indicate that the pin has
            // been set to the target value
            analogWrite(pin, value);
            return value;
        }

        // Declares integration methods and variables to be used by AMCKernel class to manage every class instance that
        // inherits from the AMCModule class (In other words: every module class instance).

        /// The ID byte-code for the class (module type) to be shared by all class instances. Used as the 'module' field
        /// of the incoming and outgoing data payload header structures. Intended to be used by the AMCKernel class to
        /// properly address incoming commands and internally to properly address the outgoing event data. This ID is
        /// crucial as it changes the interpretation of the rest of the byte-codes used during communication.
        uint8_t module_type = 0;

        /// The ID byte-code for the specific instance of the class (module_type). Used as the 'system' field of the
        /// incoming and outgoing data payload header structures. Intended to be used by the AMCKernel class to
        /// properly address incoming commands and internally to properly address the outgoing event data. This ID is
        /// dependent on the module_id and has to be unique for every class instance (every instance that shares the same
        /// module_id).
        uint8_t module_id = 0;

        /**
         * @brief Saves the input command to the appropriate field inside the local ExecutionControlParameters structure, so
         * that it can be executed during as soon as the Module is done with the currently running command (if any).
         *
         * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
         * method to queue Module commands for execution when they are received from the PC. This method is primarily
         * designed to abstract away direct structure manipulation.
         *
         * @attention This method does not check whether the input command code is valid, it simply saves it to the
         * appropriate field. The validity check is carried out by the virtual RunActiveCommand() method.
         *
         * @param command The id byte-code for the command to execute.
         */
        void QueueCommand(const uint8_t command)
        {
            execution_parameters.next_command = command;

            // Sets the module status appropriately
            module_status = static_cast<uint8_t>(kCoreModuleStatusCodes::kModuleCommandQueued);
        }

        /**
         * @brief Overwrites the value specified by the input id_code of the PC-addressable field inside the module-specific
         * ExecutionControlParameters structure instance with the input value.
         *
         * Casts the input value to the appropriate type before modifying the value. Can only target PC-addressable fields
         * of the structure, as not fields are by-design PC-addressable.
         *
         * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
         * method to set execution parameters when they are received from the PC. This method is intended to be used via the
         * SetParameter() method of this class to properly unify Custom and non-Custom parameter setting logic (and error /
         * success handling).
         *
         * @param id_code The unique byte-code for the specific field inside the ExecutionControlParameters structure to be
         * modified. These codes are defined in the kExecutionControlParameterCodes private enumeration of this class and
         * the id-code of the parameter structure has to match one of these codes to properly target an execution parameter
         * value.
         * @param value The specific value to set the target parameter variable to. Uses 32-bit format to support the entire
         * range of possible input values.
         *
         * @returns bool @b true if the value was successfully set, @b false otherwise. The only realistic reason for
         * returning false is if the input id_code does not match any of the id_codes available through the
         * kExecutionControlParameterCodes enumeration. SetParameter() class method uses the returned values to determine
         * whether to issue a success or failure message to the PC and handles PC messaging.
         */
        bool SetAddressableExecutionParameter(
            const uint8_t id_code,  // The ID of the parameter to overwrite
            const uint32_t value    // The value to overwrite the parameter with
        )
        {
            // Selects and overwrites the appropriate execution parameter, based on the input id_code
            switch (static_cast<kExecutionControlParameterCodes>(id_code))
            {
                // Noblock mode toggle
                case kExecutionControlParameterCodes::kNextNoblock:
                    execution_parameters.next_noblock = static_cast<bool>(value);
                    return true;

                // Recurrent execution mode toggle
                case kExecutionControlParameterCodes::kRunRecurrently:
                    execution_parameters.run_recurrently = static_cast<bool>(value);
                    return true;

                // Recurrent delay time (microseconds)
                case kExecutionControlParameterCodes::kRecurrentDelay:
                    execution_parameters.recurrent_delay = static_cast<uint32_t>(value);
                    return true;

                // If the input code did not match any of the (addressable) execution parameter byte-codes, returns 'false'
                // to indicate that value assignment has failed
                default: return false;
            }
        }

        /**
         * @brief An integration method that allows the AMCKernel to handle parameter assignments regardless of whether they
         * are intended for the kExecutionControlParameters or a Custom parameters structure.
         *
         * @note This is a non-virtual method that is used by the AMCKernel to assign new parameter values received from
         * PC to appropriate parameter structure fields. This method handles generating the necessary success or error
         * messages that are sent to the PC to indicate method performance.
         *
         * This method is called both when setting kExecutionControlParameters and any custom parameters. The method uses
         * the same success and failure codes for 'System' execution parameter and custom parameters, derived from the
         * kCoreModuleStatusCodes structure. This allows to seamlessly unify Core and Custom parameter setting status and
         * handled them identically regardless of the source on the PC side.
         *
         * @attention The only method that every developer should write themselves for this entire aggregation of methods
         * (SetParameter, SetAddressableExecutionParameter and SetCustomParameter) to function as expected is the
         * SetCustomParameter(), which overwrites custom parameters. The rest of the structure would integrate
         * automatically, provided that the SetCustomParameter() is written correctly.
         *
         * @param id_code The byte-code that points to a specific parameter to be overwritten inside either the Core
         * kExecutionControlParameters or any Custom parameters structure of the Module.
         * @param value The new value to set the target parameter to.
         *
         * @returns bool @b true if the method succeeds and @b false otherwise. Handles any required PC messaging in-place and
         * only returns the status to assist the caller AMCKernel method in deciding how to proceed with its own runtime.
         */
        bool SetParameter(const uint8_t id_code, const uint32_t value)
        {
            bool success;  // Tracks the assignment status

            if (id_code < 11)
            {
                // If the id code is below 11, this indicates that the targeted parameter is se Execution parameter (only these
                // parameters can be addressed using system-reserved codes).
                success = SetAddressableExecutionParameter(id_code, value);
            }
            else
            {
                // Otherwise, uses the developer-provided virtual interface to override the requested custom parameter with the
                // input value
                success = SetCustomParameter(id_code, value);
            }

            // Handles the parameter setting result status. If parameter writing fails, sends an error message to the PC and
            // returns 'false' so that the AMCKernel method that called this method can resolve its runtime too.
            if (!success)
            {
                // Sets the module_status appropriately
                module_status = static_cast<uint8_t>(kCoreModuleStatusCodes::kInvalidModuleParameterIDError);

                // Error message to indicate the method was not able to match the input id_code to any known code. Uses
                // NoCommand code to indicate that the message originates from a non-command method.
                const uint8_t error_values[1] = {id_code};
                communication_port.SendErrorMessage(
                    module_id,
                    system_id,
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    module_status,
                    error_values
                );
                return false;
            }
            // Otherwise, sends the success message to the PC to communicate the new value of the overwritten parameter.
            else
            {
                // Sets the module_status appropriately
                module_status = static_cast<uint8_t>(kCoreModuleStatusCodes::kModuleParameterSet);

                // Success message to indicate that the parameter targeted by the ID code has been set to a new value (also
                // includes the value). Uses NoCommand code to indicate that the message originates from a non-command method.
                communication_port.CreateEventHeader(
                    module_id,
                    system_id,
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    module_status
                );
                communication_port.PackValue(id_code, value);
                communication_port.SendData();
                return true;
            }
        }

        /**
         * @brief If the module is not already executing a command, sets a new command to be executed.
         *
         * Specifically, first checks whether the module is currently executing a command. If the module is not executing a
         * command and a new command is available, transfers command data from queue (buffer) into the active directory. If
         * no new command is available, but run_recursively ExecutionControlParameters flag is @b true, transfers (the same
         * command as presumably has just finished running) into the active directory to re-run it, provided that the
         * specified recurrent_delay of microseconds has passed since the last command activation.
         *
         * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
         * method to set execution parameters when they are received from the PC. It is expected that the AMCKernel handles
         * any error / success messaging for this method internally, using Kernel-specific event codes where necessary.
         *
         * @returns bool @b true if a new command was successfully set and @b false otherwise.
         */
        bool SetActiveCommand()
        {
            // If the command field is not 0, this means there is already an active command being executed and no further action
            // is needed (returns false to indicate no new command was set). Placing this check at the top of the loop
            // minimizes delays when there is an active command in-progress.
            if (execution_parameters.command != 0) return false;

            // If the new_command flag is set to true and there is no active command (checked above), immediately activates the
            // next_command
            if (execution_parameters.new_command)
            {
                // Transfers the command and the noblock flag from buffer fields to active fields
                execution_parameters.command = execution_parameters.next_command;
                execution_parameters.noblock = execution_parameters.next_noblock;

                // Sets active command stage to 1, which is a secondary activation mechanism. All multi-stage functions start
                // with stage 1 and will be deadlocked if the stage is kept at 0 (default reset state)
                execution_parameters.stage = 1;

                // Removes the new_command flag to indicate that the new command has been consumed
                execution_parameters.new_command = false;

                // Resets recurrent timer to 0 whenever a command is activated
                execution_parameters.recurrent_timer = 0;

                // Returns 'true' to indicate that a new command was activated
                return true;
            }

            // There is no good reason for extracting this into a variable other than to make the 'if' check below not exceed
            // the column-width boundaries. Otherwise, the auto-formatter unwinds it to be beyond limits. So purely a cosmetic
            // step.
            bool recurrent = execution_parameters.run_recurrently;

            // If no new command is available, but recurrent activation is enabled, re-queues a command (if any is available)
            // from the buffer, if recurrent_delay of microseconds has passed since the last reset of the recurrent_timer
            if (recurrent && execution_parameters.recurrent_timer > execution_parameters.recurrent_delay)
            {
                // Repeats the activation steps from above, minus the new_command flag modification, if next_command is not 0
                if (execution_parameters.next_command != 0)
                {
                    execution_parameters.command         = execution_parameters.next_command;
                    execution_parameters.noblock         = execution_parameters.next_noblock;
                    execution_parameters.stage           = 1;
                    execution_parameters.recurrent_timer = 0;
                    return true;
                }
                else return false;  // If next_command buffer is 0, returns false to indicate there is no command to set
            }

            // Returns 'false' if to indicate no command was set if the case is not handled by the conditionals above
            return false;
        }

        /**
         * @brief Resets the local instance of the ExecutionControlParameters structure to default values.
         *
         * This method is designed primarily for Teensy boards that do not reset on UART / USB cycling. As such, this method
         * is designed to reset the Controller between runtimes to support proper runtime setup and execution.
         *
         * Has no returns as there is no way this method can fail if it compiles.
         *
         * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
         * method to set execution parameters when they are received from the PC. It is expected that the AMCKernel handles
         * any error / success messaging for this method internally, using Kernel-specific event codes where necessary.
         */
        void ResetExecutionParameters()
        {
            // Sets execution_parameters to a default instance of ExecutionControlParameters structure
            execution_parameters = ExecutionControlParameters();
        }

        // Declares virtual methods to be overwritten by each child class. Virtual methods are critical for integrating
        // the custom logic of each child class with the centralized runtime flow control functionality realized through the
        // AMCKernel class. Specifically, each module developer should define a custom implementation for each of these
        // methods, which AMCKernel will call to properly handle custom class-specific commands and parameters.
        // Uses pure virtual approach to enforce that each child overrides these methods with custom logic implementation.

        /**
         * @brief Sets the custom parameter specified by the input id_code to the input value. This method provides an
         * interface for the AMCKernel class to work with custom parameters of any custom class.
         *
         * Specifically, use this method to provide an interface the AMCKernel class can use to translate incoming
         * Parameter data-structures (See SerialPCCommunication implementation) into overwriting the target parameter
         * values. The exact realization (codes, parameters, how many parameter structures and how to name them, etc.) is
         * entirely up to you, as long as the interface behaves in a way identical to the SetAddressableExecutionParameter()
         * method of the AMCModule base class.
         *
         * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
         * from base AMCModule class. This method is intended to be used via the inherited SetParameter() method of this
         * class to properly unify Custom and non-Custom parameter setting logic (and error / success handling).
         *
         * @param id_code The unique byte-code that specifies the custom parameter to be modified.
         * @param value The value to write to the target parameter variable. Should be converted to the appropriate type
         * for each custom parameter.
         *
         * @returns bool @b true if the new parameter value was successfully set, @b false otherwise. SetParameter() class
         * method uses the returned values to determine whether to issue a success or failure message to the PC and handles
         * PC messaging. The only reason for receiving the 'false' response should be if the input id_code does not match
         * any of the valid id_codes used to identify specific parameter values.
         */
        virtual bool SetCustomParameter(uint8_t id_code, uint32_t value) = 0;

        /**
         * @brief Triggers the currently active command. This method provides the interface for the AMCKernel class to
         * activate custom commands and verifies the module actually recognizes the command.
         *
         * Specifically, use this method to allow AMCKernel to activate (physically run) custom commands specified by the
         * unique byte-code written to the active_command field of the local ExecutionControlParameters structure by the
         * non-virtual SetActiveCommand() method.
         *
         * It is highly advised to write custom commands using the utility methods available through inheritance from the
         * AMCModule class, especially the CompleteCommand() method that has to be called at the end of each command. The
         * available collection of utility methods offer a seamless integration with the core features of the AMCModule
         * class and the broader Ataraxis framework (which includes the AMC system). See one of the test classes available
         * in the AMCKernel library for an example of how to write custom functions.
         *
         * @note Use GetActiveCommand() non-virtual method in your custom implementation to determine the currently active
         * command code. 0 is not a valid active command code! The suggested use for this method is a basic switch statement
         * that executes the requested command that matches the active_command field of the ExecutionControlParameters
         * structure and returns 'false' if the command does not match any of the predefined custom command IDs.
         *
         * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
         * from base AMCModule class.
         *
         * @returns bool @b true if the command was successfully triggered, @b false otherwise. AMCKernel uses the returned
         * value of this method to determine whether to issue an error message to the PC (if command fails verification),
         * so it is important that the method properly handles verification failure cases and returns false.
         */
        virtual bool RunActiveCommand() = 0;

        /**
         * @brief Carries out all setup operations for the current module. This method provides the interface for the
         * AMCKernel class to setup all custom elements of the Module class at the beginning of each runtime.
         *
         * Specifically, use this method as you would typically use a 'setup' loop of the Arduino default code layout. Use
         * it to set pin-modes, set various local assets to default values, adjust analog resolution, etc. This method is
         * called at the beginning of each runtime, following the reset of all assets, as a way to mimic the setup()
         * behavior for Teensy boards that do not reset on USB / UART cycling.
         *
         * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
         * from base AMCModule class.
         *
         * @attention Ideally, this should not contain any logic that can fail in any way as error handling for setup
         * methods is not implemented at this time.
         */
        virtual void SetupModule() = 0;

        /**
         * @brief Resets all local assets that needs to be reset back to provided defaults. This method provides the
         * interface for the AMCKernel class to reset all active assets to simulate USB / UART cycling on Arduino boards.
         *
         * Specifically, use this method to reset any class-specific custom assets (parameter structures, variables,
         * trackers, etc.) that need to be reset between runtime cycles. On Arduino boards, this happens naturally whenever
         * the USB / UART connection is re-established, but this is not true for Teensy boards. Since it is advantageous to
         * start each runtime from a well-known default values, this method is used to reset all custom assets. All Core
         * assets have compatible methods so that they can (and are!) reset before the beginning fo each new runtime.
         *
         * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
         * from base AMCModule class.
         */
        virtual void ResetCustomAssets() = 0;

        /**
         * @brief A pure virtual destructor method to ensure proper cleanup.
         *
         * No extra cleanup steps other than class deletion itself (which also does not happen as everything in the
         * codebase so far is static)
         */
        virtual ~Module() = default;

    protected:
        /// This is a reference to the global ControllerRuntimeParameters structure instantiated and controlled by the
        /// AMCKernel class. Many methods of the base AMCModule and (expectedly) derived classes require the information
        /// from this structure to function properly. However, the only class that is allowed to modify the instance is the
        /// AMCKernel class itself. To protect the base instance from modification, this reference is provided as a
        /// constant member.
        const shared_assets::ControllerRuntimeParameters& runtime_parameters = shared_assets::DynamicRuntimeParameters;

        /// This is a reference to the shared instance of the SerialPCCommunication class that handles the bidirectional
        /// Controller-PC communication via the serial UART or USB interface. While any data reception methods of the
        /// class are intended to be used solely by the AMCKernel class, all data transmission methods should be used
        /// in-place by the modules that need to send data to the PC. As such, this reference is not constant and any Module
        /// that inherits from the base class is allowed to use the internal methods that modify the local variables of the
        /// referenced class (this is required fro the proper functioning of the communication protocols).
        SerialPCCommunication& communication_port;

        /**
         * @enum kExecutionControlParameterCodes
         * @brief Provides the id byte-codes for the variables inside the ExecutionControlParameters structure that are
         * addressable by the PC.
         *
         * This enumeration is used to address and set specific execution parameters when the PC sends values, such as a
         * new command to execute, for any of these variables. Fields that are by design @b not PC-addressable do not have
         * an id-code assigned to them in an effort to discourage accidental overwriting. If a particular field is not
         * PC-addressable, that does not mean it is constant. Many of the local methods use non-PC-addressable fields to
         * correctly control module command execution flow.
         *
         * @warning To support correct command handling behavior by the AMCKernel class, this enumeration should @b only
         * use codes 1 through 10. Since no other enumeration uses these codes to target specific parameter values, this
         * ensures that critical execution control parameters can be precisely addressed in all Modules derived from the
         * AMCModule base class.
         *
         * @attention All variables inside this enumeration should @b not use code 0.
         */
        enum class kExecutionControlParameterCodes : uint8_t
        {
            kNextNoblock    = 1,  ///< Allows to adjust execution mode between blocking and non-blocking
            kRunRecurrently = 2,  ///< Allows to select whether the queued command runs once and clears or recurs
            kRecurrentDelay =
                3,  ///< Allows to specify the minimum amount of microseconds between recurrent command calls
        };

        /**
         * @enum kReturnedUtilityCodes
         * @brief Codifies byte-codes that are returned by certain utility methods available through the base AMCModule
         * class via inheritance.
         *
         * Use the enumerators inside this enumeration when writing result-handling code for the derived class methods that
         * make use of base AMCModule class utility methods that return byte-codes to correctly interpret the resultant
         * codes. Each variable inside this enumeration is prefixed the name of the method whose' return byte-code it
         * describes.
         *
         * @attention This enumeration only covers the base class utility function return codes. You need to provide a
         * custom enumeration when writing your own classes that are derived from the base class to function as the source
         * of the status byte-codes (if you intend to support that functionality).
         *
         * @note All variables inside this enumeration should NOT use codes 0 through 10 (inclusive) and 250 through 255.
         * These codes are reserved for system-use.
         */
        enum class kReturnedUtilityCodes : uint8_t
        {
            kWaitForAnalogThresholdFailure  = 11,  ///< Analog threshold check failed
            kWaitForAnalogThresholdPass     = 12,  ///< Analog threshold check passed
            kWaitForAnalogThresholdTimeout  = 13,  ///< Analog threshold check aborted due to timeout
            kWaitForDigitalThresholdFailure = 14,  ///< Digital threshold check failed
            kWaitForDigitalThresholdPass    = 15,  ///< Digital threshold check passed
            kWaitForDigitalThresholdTimeout = 16   ///< Digital threshold check aborted due to timeout
        };

        /**
         * @enum kCoreModuleStatusCodes
         * @brief Stores byte-codes used to communicate the status of the Module runtime which is communicated to other
         * classes (AMCKernel) and to the PC.
         *
         * @attention These codes only apply to Core methods inherited from the base AMCModule class and it is expected that
         * a separate enumeration is defined and used to communicate the status codes related to custom commands and
         * methods of each class inheriting from the AMCModule class. In this case, the custom code enumeration should use
         * complimentary codes with this enumeration to avoid code overlap. Having the two enumerations overlap will likely
         * result in unexpect4ed behavior and inability of the PC / other classes to correctly process status codes.
         *
         * @note This enumerator should use codes 0 through 10 where applicable to make critical system statuses stand out
         * from the more generic codes. Only developers with a good grasp of the codebase should be changing / introducing
         * these codes as they can lead to broad runtime failures if used incorrectly.
         */
        enum class kCoreModuleStatusCodes : uint8_t
        {
            kInvalidModuleParameterIDError = 11,  ///< Unable to recognize incoming Kernel parameter id
            kModuleParameterSet            = 12,  ///< Module parameter has been successfully set to input value
            kModuleCommandQueued           = 13   ///< The next command to execute has been successfully queued
        };
};

#endif  //AMC_MODULE_H
