/**
 * @file
 * @brief The header file for the base Module class, which is used as a parent for all custom module classes.
 *
 * @subsection mod_description Description:
 *
 * @note Every custom module class should inherit from this class. This class serves two major purposes. First, it
 * provides a static interface used by Kernel and Communication classes. This enables Core classes from this library to
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
 * - Core methods. These methods are also inherited from the parent class, but they should be used exclusively by
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
         * These codes are used by the Core class methods (Methods expected to be accessed by the Kernel class).
         * Utility methods are not accessed by the Kernel class and, therefore, do not use the module status field.
         *
         * @attention This enumeration only covers status codes used by non-virtual methods inherited from the base
         * Module class. All custom modules should use a separate enumeration to define status codes specific to the
         * custom logic of the module.
         *
         * @note To support unified status code reporting, this enumeration reserves values 0 through 100. All custom
         * status codes should use values from 101 through 255. This way, status codes derived from this enumeration
         * will never clash with 'custom' status codes.
         */
        enum class kCoreStatusCodes : uint8_t
        {
            kStandBy                   = 0,  ///< The code used to initialize the module_status variable.
            kCommandQueued             = 1,  ///< The next command to execute has been successfully queued.
            kCommandAlreadyRunning     = 2,  ///< The module cannot activate new commands as one is already running.
            kNewCommandActivated       = 3,  ///< The module has successfully activated a new command.
            kRecurrentCommandActivated = 4,  ///< The module has successfully activated a recurrent command.
            kCommandNotActivated =
                5,  ///< The module does not have a command to activate, or the recurrent timeout has not expired yet
            kExecutionParametersReset = 6,  ///< The execution_parameters structure of the module has been reset.
            kActiveCommandAborted     = 7,  ///< The active (running) module command has been forcibly terminated.
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
         * @attention Follow this instantiation order when writing the main .cpp / .ino file for the controller:
         * Communication → Module(s) → Kernel. See the main.cpp included with the library for details.
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
         *
         * @note Calling this method also resets the stage timer to 0. Therefore, this method aggregates all steps
         * necessary to properly advance executed command stage and should be used over manually changing command flow
         * trackers where possible.
         */
        void AdvanceCommandStage()
        {
            execution_parameters.stage++;
            ResetStageTimer();  // Also resets the stage delay timer
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
            execution_parameters.command = 0;  // Removes active command code
            execution_parameters.stage   = 0;  // Secondary deactivation step, stage 0 is not a valid command stage
        }

        /**
         * @brief Resets the timer that tracks the delay between active command stages.
         *
         * This timer is primarily used by methods that block for external inputs, such as sensor values. All such
         * methods have a threshold, in microseconds, after which they forcibly end the loop, to prevent deadlocking
         * the controller. This method resets the timer to 0.
         *
         * @warning This method should be called each time before advancing the command stage. Calling
         * AdvanceCommandStage() method automatically calls this method. If this method is not called when advancing
         * command stage, it is very likely that the controller will behave unexpectedly!
         */
        void ResetStageTimer()
        {
            execution_parameters.delay_timer = 0;
        }

        /**
         * @brief Polls and (optionally) averages the value(s) of the requested analog pin.
         *
         * @note This method will use the global analog readout resolution set during the setup() method runtime.
         *
         * @param pin The number of the analog pin to be interfaced with.
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
         * @param pin The number of the digital pin to be interfaced with.
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
         * Depending on execution configuration, the method can block in-place until the escape duration has passed or
         * function as a check for whether the required duration of microseconds has passed.
         *
         * @note The delay_timer can be reset by calling ResetStageTimer() method. The timer is also reset whenever
         * AdvanceCommandStage() method is called. Overall, the delay is intended to be relative to the onset of the
         * current command execution stage.
         *
         * @param delay_duration The duration, in @em microseconds the method should delay / check for.
         *
         * @returns bool @b true if the delay has been successfully enforced (either via blocking or checking) and
         * @b false in all other cases. If the elapsed duration equals to the delay_duration, this is considered a
         * passing condition.
         */
        [[nodiscard]]
        bool WaitForMicros(const uint32_t delay_duration) const
        {
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
                return true;  // Returns true
            }

            // If the requested duration has not passed, returns false
            return false;
        }

        /**
         * @brief Checks if the evaluated @b analog sensor pin detects a value that is within the specified boundaries.
         *
         * Both lower and upper boundaries are inclusive: if the sensor value matches either boundary, it is considered
         * a passing condition. Depending on configuration, the method can block in-place until the escape condition is
         * met or be used as a simple check of whether the condition is met.
         *
         * @note It is highly advised to use this method with a range of acceptable inputs. While it is possible to
         * set the lower_boundary and upper_boundary to the same number, this will likely not work as expected. This is
         * because analog sensor readouts tend to naturally fluctuate within a certain range.
         *
         * @param sensor_pin The number of the analog pin connected to the sensor.
         * @param lower_boundary The lower boundary for the sensor value. If sensor readout is at or above this value,
         * the method returns with a success code (provided upper_boundary condition is met). Minimum valid boundary
         * is 0.
         * @param upper_boundary The upper boundary for the sensor value. If sensor readout is at or below this value,
         * the method returns with a success code (provided lower_boundary condition is met). Maximum valid boundary
         * is 65535.
         * @param timeout The number of microseconds since the last reset of the delay_timer, to wait before forcibly
         * terminating method runtime. Typically, delay_timer is reset when advancing command execution stage. This
         * prevents controller deadlocks.
         * @param pool_size The number of sensor readouts to average before comparing the value to the boundaries.
         *
         * @returns integer code @b 1 if the analog sensor readout is within the specified boundaries and integer code
         * @b 0 otherwise. If timeout number of microseconds has passed since the module's delay_timer has been reset,
         * returns integer code @b 2 to indicate timeout condition.
         */
        [[nodiscard]]
        uint8_t WaitForAnalogThreshold(
            const uint8_t sensor_pin,
            const uint16_t lower_boundary,
            const uint16_t upper_boundary,
            const uint32_t timeout,
            const uint16_t pool_size = 0
        ) const
        {
            // Gets the analog sensor value
            uint16_t value = GetRawAnalogReadout(sensor_pin, pool_size);

            // If the analog value is within the boundaries, returns with a success code
            if (value >= lower_boundary && value <= upper_boundary) return 1;

            // If the command that called this method runs in non-blocking mode, returns with the failure code based on
            // whether timeout has been exceeded or not.
            if (execution_parameters.noblock)
            {
                // If the delay timer exceeds timeout, returns with code 2 to indicate timeout condition
                if (execution_parameters.delay_timer > timeout) return 2;

                // Otherwise, returns 0 to indicate threshold check failure
                return 0;
            }

            // If the command that calls this method runs in blocking mode, blocks until the sensor readout satisfies
            // the conditions or timeout is reached.
            while (execution_parameters.delay_timer < timeout)
            {
                // Repeatedly evaluates the sensor readout. If it matches escape conditions, returns with code 1.
                value = GetRawAnalogReadout(sensor_pin, pool_size);
                if (value >= lower_boundary && value <= upper_boundary) return 1;
            }

            // The only way to escape the loop above is to encounter the timeout condition. Blocking calls do not
            // return code 0.
            return 2;
        }

        /**
         * @brief Checks if the evaluated @b digital sensor pin detects a value that matches the specified threshold.
         *
         * Depending on configuration, the method can block in-place until the escape condition is met or be used as a
         * simple check of whether the condition is met.
         *
         * @param sensor_pin The number of the digital pin connected to the sensor.
         * @param threshold A boolean @b true or @b false which is used as a threshold for the pin value.
         * @param timeout The number of microseconds since the last reset of the delay_timer, to wait before forcibly
         * terminating method runtime. Typically, delay_timer is reset when advancing command execution stage. This
         * prevents controller deadlocks.
         * @param pool_size The number of sensor readouts to average before comparing the value to the threshold.
         *
         * @returns integer code @b 1 if the digital sensor readout matches the threshold and integer code @b 0
         * otherwise. If timeout number of microseconds has passed since the module's delay_timer has been reset,
         * returns integer code @b 2 to indicate timeout condition.
         */
        [[nodiscard]]
        uint8_t WaitForDigitalThreshold(
            const uint8_t sensor_pin,
            const bool threshold,
            const uint32_t timeout,
            const uint16_t pool_size = 0
        ) const
        {
            // Gets the digital sensor value
            bool value = GetRawDigitalReadout(sensor_pin, pool_size);

            // If the digital value matches target state (HIGH or LOW), returns with success code
            if (value == threshold) return 1;

            // If the command that called this method runs in non-blocking mode, returns with the failure code based on
            // whether timeout has been exceeded or not.
            if (execution_parameters.noblock)
            {
                // If the delay timer exceeds timeout, returns with code 2 to indicate timeout condition
                if (execution_parameters.delay_timer > timeout) return 2;

                // Otherwise, returns 0 to indicate threshold check failure
                return 0;
            }

            // If the command that calls this method runs in blocking mode, blocks until the sensor readout satisfies
            // the conditions or timeout is reached.
            while (execution_parameters.delay_timer < timeout)
            {
                // Repeatedly evaluates the sensor readout. If it matches escape conditions, returns with code 1.
                value = GetRawDigitalReadout(sensor_pin, pool_size);
                if (value == threshold) return 1;
            }

            // The only way to escape the loop above is to encounter the timeout condition. Blocking calls do not
            // return code 0.
            return 2;
        }

        /**
         * @brief Sets the input digital pin to the specified value (High or Low).
         *
         * This utility method is used to control the value of a digital pin configured to output a constant High or Low
         * level signal. Internally, the method checks for appropriate output locks, based on the pin type, before
         * setting its value.
         *
         * @param pin The number of the digital pin to interface with.
         * @param value The boolean value to set the pin to.
         * @param ttl_pin Determines whether the pin is used to drive hardware actions or for TTL communication. This
         * is necessary to resolve whether action_lock or ttl_lock dynamic runtime parameters apply to the pin. When
         * applicable, either of these parameters prevents setting the pin to HIGH and, instead, ensures the pin is set
         * to LOW.
         *
         * @returns Current output level (HIGH or LOW) the pin has been set to.
         */
        [[nodiscard]]
        bool DigitalWrite(const uint8_t pin, const bool value, const bool ttl_pin) const
        {
            // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to LOW and returns
            // the current value of the pin (LOW).
            if ((ttl_pin && _dynamic_parameters.ttl_lock) || (!ttl_pin && _dynamic_parameters.action_lock))
            {
                digitalWriteFast(pin, LOW);
                return false;
            }

            // If the pin is not locked, sets it to the requested value and returns the new value of the pin.
            digitalWriteFast(pin, value);
            return value;
        }

        /**
         * @brief Sets the input analog pin to the specified duty-cycle value (from 0 to 255).
         *
         * This utility method is used to control the value of an analog pin configured to output a PWM-pulse with the
         * defined duty cycle from 0 (off) to 255 (always on). Internally, the method checks for appropriate output
         * locks, based on the pin type, before setting its value.
         *
         * @note Currently, the method only supports standard 8-bit resolution to maintain backward-compatibility with
         * AVR boards. In the future, this may be adjusted.
         *
         * @param pin The number of the analog pin to interface with.
         * @param value The duty cycle value from 0 to 255 that controls the proportion of time during which the pin
         * is HIGH. Note, the exact meaning of each duty-cycle step depends on the clock that is used to control the
         * analog pin cycling behavior.
         * @param ttl_pin Determines whether the pin is used to drive hardware actions or for TTL communication. This
         * is necessary to resolve whether action_lock or ttl_lock dynamic runtime parameters apply to the pin. When
         * applicable, either of these parameters prevents setting the pin to any positive value and, instead, ensures
         * the pin is set to 0.
         *
         * @returns Current duty cycle (0 to 255) the pin has been set to.
         */
        [[nodiscard]]
        uint8_t AnalogWrite(const uint8_t pin, const uint8_t value, const bool ttl_pin) const
        {
            // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to 0
            // (constantly off) and returns the current value of the pin (0).
            if ((ttl_pin && _dynamic_parameters.ttl_lock) || (!ttl_pin && _dynamic_parameters.action_lock))
            {
                analogWrite(pin, 0);
                return 0;
            }

            // If the pin is not locked, sets it to the requested value and returns the new value of the pin.
            analogWrite(pin, value);
            return value;
        }

        // Core methods

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
         * @brief Queues the input command to be executed once the Module completes any currently running command.
         *
         * This method queues the command code to be executed and sets the runtime parameters for the command.
         * The Kernel class uses this method to queue commands received from the connected system for execution.
         *
         * @note This method is explicitly written in a way to allow replacing any already queued command. Since the
         * Module buffer is designed to only hold 1 command in queue, this allows rapidly changing the queued command
         * in response to external events.
         *
         * @attention This method does not check whether the input command code is valid. It only saves it to the
         * appropriate field of the execution_parameters structure. The validity check should be carried out by the
         * virtual RunActiveCommand() method.
         *
         * @param command The byte-code of the command to execute.
         * @param noblock Determines whether the queued command will be executed in blocking or non-blocking mode.
         * Non-blocking execution requires the command to make use of the class utility functions that allow
         * non-blocking delays.
         * @param cycle Determines whether to execute the command once or run it recurrently (cyclically).
         */
        void QueueCommand(const uint8_t command, const bool noblock, const bool cycle)
        {
            execution_parameters.next_command    = command;  // Sets the command to be executed
            execution_parameters.next_noblock    = noblock;  // Sets the noblock flag for the command
            execution_parameters.run_recurrently = cycle;    // Sets the recurrent runtime flag for the command
            execution_parameters.new_command     = true;     // Notifies other class methods this is a new command

            // Sets the module status appropriately
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kCommandQueued);
        }

        /**
         * @brief If the module is not executing a command, executes the next available command.
         *
         * If the module is not already executing a command and a new command is queued, activates that command. If
         * new command is not available, but recursive command execution is enabled, repeats the previous command. When
         * repeating previous commands, the method checks whether the specified 'recurrent_delay' of microseconds has
         * passed since the last command activation, before (re)activating the command. The Kernel uses this method to
         * set up command execution.
         *
         * @notes Any queued command is considered new until this method activates that command. All following
         * reactivations of the command are considered recurrent.
         *
         * @returns bool @b true if a new command was successfully set and @b false otherwise. Additional information
         * regarding method runtime status can be obtained from the module_status attribute.
         */
        bool SetActiveCommand()
        {
            // If the command field is not 0, this means there is already an active command being executed and no
            // further action is necessary. Returns false to indicate no new command was set.
            if (execution_parameters.command != 0)
            {
                module_status = static_cast<uint8_t>(kCoreStatusCodes::kCommandAlreadyRunning);
                return false;
            }

            // If the new_command flag is set to true and there is no active command, activates the next_command
            if (execution_parameters.new_command)
            {
                // Transfers the command and the noblock flag from buffer fields to active fields
                execution_parameters.command = execution_parameters.next_command;
                execution_parameters.noblock = execution_parameters.next_noblock;

                // Sets active command stage to 1, which is a secondary activation mechanism. All multi-stage commands
                // should start with stage 1 and deadlock if the stage is kept at 0 (default reset state)
                execution_parameters.stage = 1;

                // Removes the new_command flag to indicate that the new command has been consumed
                execution_parameters.new_command = false;

                // Resets recurrent timer to 0 whenever a command is activated
                execution_parameters.recurrent_timer = 0;

                // Returns 'true' to indicate that a new command was activated
                module_status = static_cast<uint8_t>(kCoreStatusCodes::kNewCommandActivated);
                return true;
            }

            // If previous command is available, recurrent activation is enabled, and the requested recurrent_delay
            // number of microseconds has passed, re-queues the command to be executed again.
            if (execution_parameters.run_recurrently &&
                execution_parameters.recurrent_timer > execution_parameters.recurrent_delay &&
                execution_parameters.next_command != 0)
            {
                // Repeats the activation steps from above, minus the new_command flag modification (command is not new)
                execution_parameters.command         = execution_parameters.next_command;
                execution_parameters.noblock         = execution_parameters.next_noblock;
                execution_parameters.stage           = 1;
                execution_parameters.recurrent_timer = 0;
                module_status = static_cast<uint8_t>(kCoreStatusCodes::kRecurrentCommandActivated);
                return true;
            }

            // Returns 'false' to indicate no command was set. This usually is due to no commands being available or
            // the recurrent activation timeout.
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kCommandNotActivated);
            return false;
        }

        /**
         * @brief Resets the class execution_parameters instance to default values.
         *
         * This method is designed for Teensy boards that do not reset on UART / USB cycling. The Kernel uses this
         * method to reset the Module between runtimes.
         */
        void ResetExecutionParameters()
        {
            // Sets execution_parameters to a default instance of ExecutionControlParameters structure
            execution_parameters = ExecutionControlParameters();
            module_status        = static_cast<uint8_t>(kCoreStatusCodes::kExecutionParametersReset);
        }

        /**
         * @brief Aborts the active command by forcibly terminating its concurrent runtime.
         *
         * This method is used to cancel an actively running command, provided it is executed in non-blocking mode.
         * Kernel class uses this command to 'soft' reset the Module in certain circumstances.
         *
         * @warning This method will not be able to abort blocking commands! Aborting blocking commands requires
         * software or hardware interrupt functionality and needs to be introduced at the Kernel level.
         */
        void AbortCommandExecution()
        {
            CompleteCommand();
            module_status = static_cast<uint8_t>(kCoreStatusCodes::kActiveCommandAborted);
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
         * values. The exact realization (codes, parameters, number of parameter structures and how to name them, etc.)
         * is entirely up to you, as long as the interface behaves in a way identical to the
         * SetAddressableExecutionParameter() method of the AMCModule base class.
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
         * Currently, there are no extra cleanup steps other than class deletion itself, which also does not happen
         * as everything in the codebase so far is static. Generally safe to reimplement without additional logic.
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
