/**
 * @file
 * @brief The header file for the base Module class, which is used as a parent for all custom module classes.
 *
 * @subsection mod_description Description:
 *
 * @note Every custom module class should inherit from this class. This class serves two major purposes. First, it
 * provides a static interface used by Kernel and Communicaztion classes. This enables Core classes from this library to
 * reliably interface with any custom module. Additionally, this class provides utility functions that abstract many
 * routine tasks in a way that is compatible with concurrent runtime enforced by the Kernel module.
 *
 * @attention It is highly advised to check one of the 'default' custom classes shipped with this library. These classes
 * showcase the principles of constructing custom classes using the available base Module class methods.
 *
 * The Module class available through this file is broadly divided into three sections:
 * - Utility methods. Provides utility functions that implement routinely used features, such as waiting for time or
 * sensor activation. Methods from this section will be inherited when subclassing this class and should be used when
 * writing custom class methods where appropriate.
 * - Integration methods. These methods are also inherited from the parent class, but they should be used exclusively by
 * the Kernel and Communication classes. These methods form the static interface that natively integrates any custom
 * module with the Kernel and Communication classes.
 * - Virtual Methods. These methods serve the same purpose as the Core methods, but they provide an interface between
 * the custom logic of each module and the Kernel class. These methods are implemented as pure virtual methods and have
 * to be defined by developers for each custom module. Due to their consistent signature, the Kernel can use these
 * methods regardless of the specific implementation of each method.
 *
 * @subsection mod_developer_notes Developer Notes:
 * This is one of the key Core level classes that is critically important for the functioning of the whole AMC codebase.
 * Generally, only Kernel developers with a good grasp of the codebase should be modifying this base class. This is
 * especially relevant for class versions that modify existing functionality and, as such, are likely to be incompatible
 * with existent custom modules.
 *
 * There are two main ways of using this class to develop custom modules. The preferred way is to rely on the available
 * utility methods provided by this class to abstract all interactions with the inherited Core methods and variables.
 * Specifically, any class inheriting from this base class should use the methods in the utility sections inside all
 * custom commands and overridden virtual methods where appropriate. The current set of utility methods should be
 * sufficient to support most standard microcontroller use cases, while providing automatic compatibility with the rest
 * of the AMC codebase. As a bonus, this method is likely to improve code maintainability, as utility methods are
 * guaranteed to be supported and maintained even if certain Core elements in the class are modified by Kernel library
 * developers.
 *
 * The second way, which may be useful if specific required functionality is not available through the utility methods,
 * is to directly use the included core structures and variables, such as the ExecutionControlParameters structure.
 * This requires a good understanding of how the base class, and it's method function, as well as an understanding of
 * how Kernel works and interacts with Module-derived classes. With sufficient care, this method can provide more
 * control over the behavior of any custom module, at the expense of being harder to master and less safe.
 *
 * Regardless of the use method, any custom Module inheriting from the Module class has to implement ALL purely virtual
 * method to make the resultant class usable by the Kernel class.
 *
 * @subsection mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - shared_assets.h for globally shared static message byte-codes and the ControllerRuntimeParameter structure.
 * - communication.h for Communication class, which is used to send module runtime data to the connected system.
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - elapsedMillis.h for millisecond and microsecond timers.
 */

#ifndef AXMC_MODULE_H
#define AXMC_MODULE_H

#include <Arduino.h>
#include "communication.h"
#include "shared_assets.h"

/**
 * @brief Serves as the parent for all custom module classes, providing methods for other Core classes to interface with
 * any custom module.
 *
 * This class serves as the shared (via inheritance) repository of utility and runtime-control methods that
 * automatically integrate any custom module into the existing AMC codebase. Specifically, by inheriting from this
 * class, any module gains an interface making it possible for the Kernel and Communication classes to work with the
 * module. This way, developers can focus on specific module logic, as Core classes handle runtime flow control and
 * communication.
 *
 * @note This class offers a collection of utility methods that should be used when writing custom functions. These
 * methods abstract the interactions with Core module structures and enable functions such as concurrent (non-blocking)
 * command execution and error-handling.
 *
 * @warning Every custom module class @b has to inherit from this base class to be compatible with the rest of the AMC
 * codebase. Moreover, the base class itself uses pure virtual methods and, as such, cannot be instantiated. Only a
 * child class that properly overrides all pure virtual methods of the base class can be instantiated.
 */
class Module
{
    public:
        /**
         * @struct ExecutionControlParameters
         * @brief Stores parameters that are used to dynamically queue, execute and control the execution flow of
         * module commands.
         *
         * All runtime control manipulations that involve changing the variables inside this structure
         * should be carried out either by the Kernel class or via Core methods inherited from the base Module class.
         *
         * @attention Any modification to this structure or code that makes use of this structure should be reserved
         * for developers with a good grasp of the existing codebase. If possible, modifying this structure should be
         * avoided, especially deletions or refactoring of existing variables.
         */
        struct ExecutionControlParameters
        {
                uint8_t command      = 0;      ///< Currently executed (in-progress) command.
                uint8_t stage        = 0;      ///< Stage of the currently executed command.
                bool noblock         = false;  ///< Specifies if the currently executed command is blocking.
                uint8_t next_command = 0;      ///< A buffer that allows queuing the next command to be executed.
                bool next_noblock    = false;  ///< A buffer that stores the noblock flag for the queued command.
                bool new_command     = false;  ///< Tracks whether next_command is a new or recurrent command.
                bool run_recurrently = false;  ///< Specifies whether the queued command runs once and clears or recurs.
                uint32_t recurrent_delay = 0;  ///< The minimum number of microseconds between recurrent command calls.
                elapsedMicros recurrent_timer =
                    0;  ///< A timer class instance to time recurrent command activation delays.
                elapsedMicros delay_timer =
                    0;           ///< A timer class instance to time delays between active command stages.
        } execution_parameters;  ///< Stores module-specific runtime flow control parameters.

        /**
         * @enum kCoreStatusCodes
         * @brief Assigns meaningful names to status codes used by the module class.
         *
         * @attention This enumeration only covers status codes used by non-virtual methods inherited from the base
         * Module class. All custom modules should use a separate enumeration to define status codes specific to the
         * custom logic of the module.
         * @note To support unified status code reporting, this enumeration reserves values 0 through 100. All custom
         * status codes should use values from 101 through 255. This way, status codes derived from this enumeration
         * will never clash with 'custom' status codes.
         */
        enum class kCoreStatusCodes : uint8_t
        {
            kStandBy                  = 0,  ///< The code used to initialize the module_status variable.
            kWaitForMicrosStageEscape = 1,  ///< Microsecond timer check aborted due to mismatching command stage.
            kWaitForMicrosSuccess     = 2,  ///< Microsecond timer reached requested delay duration.
            kWaitForMicrosFailure     = 3,  ///< Microsecond timer did not reach requested delay duration.

            kInvalidModuleParameterIDError  = 1,   ///< Unable to recognize incoming Kernel parameter id
            kModuleParameterSet             = 2,   ///< Module parameter has been successfully set to input value
            kModuleCommandQueued            = 3,   ///< The next command to execute has been successfully queued
            kWaitForAnalogThresholdFailure  = 50,  ///< Analog threshold check failed
            kWaitForAnalogThresholdPass     = 51,  ///< Analog threshold check passed
            kWaitForAnalogThresholdTimeout  = 52,  ///< Analog threshold check aborted due to timeout
            kWaitForDigitalThresholdFailure = 53,  ///< Digital threshold check failed
            kWaitForDigitalThresholdPass    = 54,  ///< Digital threshold check passed
            kWaitForDigitalThresholdTimeout = 56   ///< Digital threshold check aborted due to timeout
        };

        /// Stores the most recent module status code.
        uint8_t module_status = static_cast<uint8_t>(kCoreStatusCodes::kStandBy);

        /**
         * @brief Instantiates a new Module class object.
         *
         * @param module_type The ID that identifies the type family) of the module. All instances of the same custom
         * module class should share this ID. Has to use a value not reserved by Core classes
         * (check shared_assets::kCoreMessageCodes) or other Module-derived classes.
         * @param module_id This ID is used to identify the specific instance of the module. It can use any value
         * supported by uint8_t range, as long as no other module in the type (family) uses the same value-.
         * @param communication A reference to the Communication class instance that will be used to send module runtime
         * data to the connected system. Usually, a single Communication class instance is shared by all classes of the
         * AMC codebase during runtime.
         * @param dynamic_parameters A reference to the ControllerRuntimeParameters structure that stores
         * dynamically addressable runtime parameters that broadly alter the behavior of all modules used by the
         * controller. This structure is modified via Kernel class, modules only read the data from the structure.
         *
         * @attention Follow the following instantiation order when writing the main .cpp / .ino file for the
         * controller: Communication → Module(s) → Kernel. See the main.cpp included with the library for details.
         */
        Module(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::ControllerRuntimeParameters& dynamic_parameters
        ) :
            _module_type(module_type),
            _module_id(module_id),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters)
        {}

        // Non-virtual utility methods:

        /**
         * @brief Advances the execution stage of the active (running) command.
         *
         * This method should be used by noblock-compatible commands to properly advance the command execution stage
         * where necessary. It modifies the stage tracker field inside the Module's execution_parameters structure by
         * 1 each time this method is called.
         */
        void AdvanceCommandStage()
        {
            execution_parameters.stage++;
        }

        /**
         * @brief Returns the execution stage of the active (running) command.
         *
         * This method should be used by noblock-compatible commands to retrieve the current execution stage of the
         * active command.
         *
         * @warning This method returns 0 if there is no actively executed command. 0 is not a valid stage number!
         */
        [[nodiscard]]
        uint8_t GetCommandStage() const
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
         * @brief Terminates (ends) active (running) command execution.
         *
         * Using a dedicated terminator call is essential to support non-blocking concurrent execution of multiple
         * commands.
         *
         * This method should only be called when the command completes everything it sets out to do. For noblock
         * commands, the Controller may need to loop through the command code multiple times before it reaches the
         * algorithmic endpoint. In this case, the call to this method should be protected by an 'if' statement that
         * ensures it is only called when the command has finished it's work.
         *
         * @warning It is essential that this method is called at the end of every command function to allow executing
         * other commands. Failure to do so can completely deadlock the Module and, in severe cases, the whole
         * Microcontroller.
         */
        void CompleteCommand()
        {
            execution_parameters.command = 0;
            execution_parameters.stage   = 0;
        }

        /**
         * @brief Polls and (optionally) averages the value(s) of the requested analog pin.
         *
         * @note This method will use the global analog readout resolution set during the setup() method runtime.
         *
         * @param pin The number of the Analog pin to be interfaced with.
         * @param pool_size The number of pin readout values to average into the final readout. Set to 0 or 1 to
         * disable averaging.
         *
         * @returns uint16_t value of the analog sensor. Currently, uses 16-bit resolution as the maximum supported by
         * analog pin hardware.
         */
        static uint16_t GetRawAnalogReadout(const uint8_t pin, const uint16_t pool_size = 0)
        {
            uint16_t average_readout;  // Pre-declares the final output readout

            // Pool size 0 and 1 essentially mean the same: no averaging
            if (pool_size < 2)
            {
                // If averaging is disabled, reads and outputs the acquired value.
                average_readout = analogRead(pin);
            }
            else
            {
                uint32_t accumulated_readouts = 0;  // Aggregates polled values by self-addition

                // If averaging is enabled, repeatedly polls the pin the requested number of times. 'i' always uses
                // the same type as pool_size.
                for (auto i = decltype(pool_size) {0}; i < pool_size; i++)
                {
                    accumulated_readouts += analogRead(pin);  // Aggregates readouts
                }

                // Averages and rounds the final readout to avoid dealing with floating point math. This favors Arduino
                // boards without an FP module, Teensies technically can handle floating point arithmetic just as
                // efficiently. Adding pool_size/2 before dividing by pool_size forces half-up ('standard') rounding.
                average_readout = static_cast<uint16_t>((accumulated_readouts + pool_size / 2) / pool_size);
            }

            return average_readout;  // Returns the final averaged or raw readout
        }

        /**
         * @brief Polls and (optionally) averages the value(s) of the requested digital pin.
         *
         * @note Digital readout averaging is primarily helpful for controlling jitter and backlash noise.
         *
         * @param pin The number of the Digital pin to be interfaced with.
         * @param pool_size The number of pin readout values to average into the final readout. Set to 0 or 1 to
         * disable averaging.
         *
         * @returns @b true (HIGH) or @b false (LOW).
         */
        static bool GetRawDigitalReadout(const uint8_t pin, const uint16_t pool_size = 0)
        {
            bool digital_readout;  // Pre-declares the final output readout

            // Reads the physical sensor value. Optionally uses averaging as a means of debouncing the digital signal.
            // Generally, this is a built-in hardware feature for many boards, but the logic is maintained here for
            // backward compatibility. Pool size 0 and 1 essentially mean no averaging.
            if (pool_size < 2)
            {
                digital_readout = digitalReadFast(pin);
            }
            else
            {
                uint32_t accumulated_readouts = 0;  // Aggregates polled values by self-addition

                // If averaging is enabled, repeatedly polls the pin the requested number of times. 'i' always uses
                // the same type as pool_size.
                for (auto i = decltype(pool_size) {0}; i < pool_size; i++)
                {
                    accumulated_readouts += digitalReadFast(pin);  // Aggregates readouts via self-addition
                }

                // Averages and rounds the final readout to avoid dealing with floating point math. This favors Arduino
                // boards without an FP module, Teensies technically can handle floating point arithmetic just as
                // efficiently. Adding pool_size/2 before dividing by pool_size forces half-up ('standard') rounding.
                digital_readout = static_cast<bool>((accumulated_readouts + pool_size / 2) / pool_size);
            }

            return digital_readout;
        }

        /**
         * @brief Checks if the delay_duration of microseconds has passed since the module's delay_timer has been reset.
         *
         * Depending on execution configuration, the function can block in-place until the escape duration has passed or
         * function as a check for whether the required duration of microseconds has passed. This function can be used
         * standalone or together with a stage-based execution control scheme (if stage and advance_stage are provided).
         *
         * @note To properly integrate with the concurrent 'noblock' execution mode provided by the Kernel class, custom
         * methods should use this delay method together with a stage-based method design. See one of the default
         * module classes for examples.
         *
         * @attention Remember to reset the delay_timer by calling ResetDelayTimer() before calling this method. When
         * using stage-based execution control, do this at the end of the stage preceding this method's stage.
         *
         * @param delay_duration The duration, in @em microseconds the method should delay / check for.
         * @param stage The command execution stage associated with this method's runtime. Providing this method with a
         * stage is equivalent to running it inside an 'if' statement that checks for a specific stage. Set to 0 to
         * disable stage-based execution.
         * @param advance_stage If @b true, the method will advance the command execution stage if the delay_duration
         * has passed.
         *
         * @returns bool @b true if the delay has been successfully enforced (either via blocking or checking). If the
         * elapsed duration equals to the delay_duration, this is considered a passing condition. If the method is
         * configured to only execute during a certain stage, it will return without delaying for any other stage.
         * The method can be configured to optionally advance execution stage upon successful runtime.
         */
        bool WaitForMicros(const uint32_t delay_duration, const uint8_t stage = 0, const bool advance_stage = true)
        {
            // If stage control is enabled (stage is not 0) and the current command stage does not match the stage
            // this method instance is meant to be executed at, statically returns true. This allows using the method in
            // custom classes via a one-line call.
            if (stage != 0 && stage != GetCommandStage())
            {
                module_status = static_cast<uint8_t>(kCoreStatusCodes::kWaitForMicrosStageEscape);
                return true;
            }

            // If the caller command is executed in blocking mode, blocks in-place until the requested duration has
            // passed
            if (!execution_parameters.noblock)
            {
                // Blocks until delay_duration has passed
                while (execution_parameters.delay_timer <= delay_duration)
                    ;
            }

            // Evaluates whether the requested number of microseconds has passed. If the duration was enforced above,
            // this check will always be true.
            if (execution_parameters.delay_timer >= delay_duration)
            {
                if (advance_stage) AdvanceCommandStage();  // Advances the command stage, if requested
                module_status = static_cast<uint8_t>(kCoreStatusCodes::kWaitForMicrosSuccess);
                return true;  // Returns true
            }

            // If the requested duration has not passed, returns false
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kWaitForMicrosFailure);
            return false;
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
            const uint8_t sensor_pin,
            const uint32_t timeout,
            const uint16_t upper_boundary = 65535,
            const uint16_t lower_boundary = 0,
            const uint16_t pool_size    = 0,
            const uint8_t stage         = 0,
            const bool advance_stage    = true,
            const bool abort_on_timeout = true
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
                        passed = value >= threshold;
                    }
                    else
                    {
                        passed = value <= threshold;
                    }

                    // If the delay timer exceeds timeout value, interrupts and returns kWaitForAnalogThresholdTimeout (13),
                    // which is a special timeout-quit code
                    if (execution_parameters.delay_timer > timeout)
                    {
                        return static_cast<uint8_t>(kCoreStatusCodes::kWaitForAnalogThresholdTimeout);
                    }
                }

                // Returns kWaitForAnalogThresholdPass (12) to indicate that the check has passed
                return static_cast<uint8_t>(kCoreStatusCodes::kWaitForAnalogThresholdPass);
            }

            // Otherwise, if the caller function is executed in non-blocking mode, determines what code to return
            // If the delay timer exceeds timeout, returns kWaitForAnalogThresholdTimeout (13) to indicate timeout condition
            if (execution_parameters.delay_timer > timeout)
                return static_cast<uint8_t>(kCoreStatusCodes::kWaitForAnalogThresholdTimeout);

            // Otherwise, if check passed, returns kWaitForAnalogThresholdPass (12)
            if (passed) return static_cast<uint8_t>(kCoreStatusCodes::kWaitForAnalogThresholdPass);

            // Otherwise, returns kWaitForAnalogThresholdFailure (11)
            return static_cast<uint8_t>(kCoreStatusCodes::kWaitForAnalogThresholdFailure);
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
            const uint8_t sensor_pin,
            const bool threshold,
            const uint32_t timeout,
            const uint16_t pool_size    = 0,
            const uint8_t stage         = 0,
            const bool advance_stage    = true,
            const bool abort_on_timeout = true
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
                        return static_cast<uint8_t>(kCoreStatusCodes::kWaitForDigitalThresholdTimeout);
                    }
                }

                // Returns kWaitForDigitalThresholdPass (15) to indicate that the check has passed
                return static_cast<uint8_t>(kCoreStatusCodes::kWaitForDigitalThresholdPass);
            }

            // Otherwise, if the caller function is executed in non-blocking mode, determines what code to return
            else
            {
                // If the delay timer exceeds timeout, returns kWaitForDigitalThresholdTimeout (16) to indicate timeout
                // condition
                if (execution_parameters.delay_timer > timeout)
                    return static_cast<uint8_t>(kCoreStatusCodes::kWaitForDigitalThresholdTimeout);

                // Otherwise, if check passed, returns kWaitForDigitalThresholdPass (15)
                // Note, factors in the minimum delay to ensure it is properly assessed when the method runs in non-blocking
                // mode
                else if (value == threshold && execution_parameters.delay_timer >= min_delay)
                    return static_cast<uint8_t>(kCoreStatusCodes::kWaitForDigitalThresholdPass);

                // Otherwise, returns kWaitForDigitalThresholdFailure (14)
                else return static_cast<uint8_t>(kCoreStatusCodes::kWaitForDigitalThresholdFailure);
            }
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

        // Core metods

        /**
         * @brief Returns the code of the currently active (running) command.
         *
         * If there are no active commands, this will return 0.
         *
         * The Kernel class uses this accessor method to infer when the Module is ready to execute the next
         * queued command (if available).
         */
        [[nodiscard]]
        uint8_t GetActiveCommand() const
        {
            return execution_parameters.command;
        }

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
                communication.SendErrorMessage(
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
                communication.CreateEventHeader(
                    module_id,
                    system_id,
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    module_status
                );
                communication.PackValue(id_code, value);
                communication.SendData();
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
        /// Represents the type (family) of the module. All modules in the family share the same type code.
        uint8_t _module_type = 0;

        /// The specific ID of the module. This code has to be unique within the module family, as it identifies
        /// specific module instance.
        uint8_t _module_id = 0;

        /// A reference to the shared instance of the Communication class. This class is used to send runtime data to
        /// the connected Ataraxis system.
        Communication& _communication;

        /// A reference to the shared instance of the ControllerRuntimeParameters structure. This structure stores
        /// dynamically addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        const shared_assets::ControllerRuntimeParameters& _dynamic_parameters;
};

#endif  //AXMC_MODULE_H
