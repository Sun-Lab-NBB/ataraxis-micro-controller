/**
 * @file
 * @brief Provides the Module class that exposes the API for integrating user-defined custom hardware
 * modules with other library components and the interface running on the host-computer (PC).
 *
 * @section mod_description Description:
 *
 * This class defines the API interface used by Kernel and Communication classes to interact with any custom hardware
 * module instance that inherits from the base Module class. Additionally, the class provides the utility functions
 * for routine tasks, such as changing pin states, that support the concurrent (non-blocking) runtime of multiple
 * module-derived instances.
 *
 * @attention Every custom hardware module class should inherit from the base Module class defined in this file and
 * override the pure virtual methods of the parent class with instance-specific implementations.
 */

#ifndef AXMC_MODULE_H
#define AXMC_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "axmc_shared_assets.h"
#include "communication.h"

/**
 * @brief Provides the API used by other library components to integrate any custom hardware module class with
 * the interface running on the companion host-computer (PC).
 *
 * Any class that inherits from this base class gains the API used by the Kernel and Communication classes to enable
 * bidirectionally interfacing with the module via the interface running on the companion host-computer (PC)
 *
 * @note Use the utility methods inherited from the base Module class to ensure that the custom module implementation
 * is compatible with non-blocking runtime mode. See the ReadMe for more information about non-blocking runtime support.
 *
 * @warning Every custom module class @b has to inherit from this base class. Follow this instantiation order when
 * writing the main .cpp / .ino file for the controller: Communication → Module(s) → Kernel. See the /examples folder
 * for details.
 */
class Module
{
    public:
        /**
         * @struct ExecutionControlParameters
         * @brief This structure stores the data that supports executing module-addressed commands sent from the PC
         * interface.
         *
         * @warning End users should not modify any elements of this structure directly. This structure is modified by
         * the Kernel and certain utility methods inherited from the base Module class.
         */
        struct ExecutionControlParameters
        {
                uint8_t command          = 0;      ///< Currently executed (in-progress) command.
                uint8_t stage            = 0;      ///< The stage of the currently executed command.
                bool noblock             = false;  ///< Determines whether the currently executed command is blocking.
                uint8_t next_command     = 0;      ///< Stores the next command to be executed.
                bool next_noblock        = false;  ///< Stores the noblock flag for the next command.
                bool new_command         = false;  ///< Tracks whether next_command is a new or recurrent command.
                bool run_recurrently     = false;  ///< Tracks whether next_command is recurrent (cyclic).
                uint32_t recurrent_delay = 0;      ///< The delay, in microseconds, between command repetitions.
                elapsedMicros recurrent_timer;     ///< Measures recurrent command activation delays.
                elapsedMicros delay_timer;         ///< Measures delays between command stages.
        } execution_parameters;                    ///< Stores instance-specific runtime flow control parameters.

        /**
         * @enum kCoreStatusCodes
         * @brief Stores the status codes used to communicate the states and errors encountered during the shared API
         * method runtimes.
         *
         * @attention This enumeration only covers status codes used by non-virtual methods inherited from the base
         * Module class. These status codes are considered 'system-reserved' and are handled implicitly by the
         * PC-side companion library.
         *
         * @note To support consistent status code reporting, this enumeration reserves values 0 through 50. All custom
         * status codes should use values 51 through 250. This prevents the status codes derived from this enumeration
         * from clashing with custom status codes.
         */
        enum class kCoreStatusCodes : uint8_t
        {
            kStandBy              = 0,  ///< The code used to initialize the module_status variable.
            kTransmissionError    = 1,  ///< Encountered an error when sending data to the PC.
            kCommandCompleted     = 2,  ///< The last active command has been completed and removed from the queue.
            kCommandNotRecognized = 3,  ///< The RunActiveCommand() method did not recognize the requested command.
        };

        /**
         * @brief Initializes all shared assets used to integrate the module with the rest of the library components.
         *
         * @warning This initializer must be called as part of the custom module's initialization sequence for each
         * module that subclasses this base class.
         *
         * @param module_type The code that identifies the type (family) of the module. All instances of the same
         * custom module class should share this ID code.
         * @param module_id The code that identifies the specific module instance. This code must be unique for
         * each instance of the same module family (class) used as part of the same runtime.
         * @param communication The shared Communication instance used to bidirectionally communicate with the PC
         * during runtime.
         * @param dynamic_parameters The shared DynamicRuntimeParameters structure that stores the global runtime
         * parameters shared by all library assets.
         *
         */
        Module(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const DynamicRuntimeParameters& dynamic_parameters
        ) :
            _module_type(module_type),
            _module_id(module_id),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters)
        {}

        // CORE METHODS.
        // These methods are used by the Kernel class to manage the runtime of the custom hardware module instances that
        // inherit from this base class.

        /**
         * @brief Queues the input command to be executed by the Module during the next runtime cycle iteration.
         *
         * @warning If the module already has a queued command, this method replaces that command with the input
         * command data.
         *
         * @param command The command to execute.
         * @param noblock Determines whether the queued command should run in blocking or non-blocking mode.
         * @param cycle_delay The delay, in microseconds, before repeating (cycling) the command. Only provide this
         * argument when queueing a recurrent command.
         */
        void QueueCommand(const uint8_t command, const bool noblock, const uint32_t cycle_delay)
        {
            execution_parameters.next_command    = command;
            execution_parameters.next_noblock    = noblock;
            execution_parameters.run_recurrently = true;
            execution_parameters.recurrent_delay = cycle_delay;
            execution_parameters.new_command     = true;
        }

        /// Overloads the QueueCommand() method for queueing non-cyclic commands.
        void QueueCommand(const uint8_t command, const bool noblock)
        {
            execution_parameters.next_command    = command;
            execution_parameters.next_noblock    = noblock;
            execution_parameters.run_recurrently = false;
            execution_parameters.recurrent_delay = 0;
            execution_parameters.new_command     = true;
        }

        /**
         * @brief Resets the module's command queue.
         *
         * @note Calling this method does not abort already running commands: they are allowed to finish gracefully.
         */
        void ResetCommandQueue()
        {
            execution_parameters.next_command    = 0;
            execution_parameters.next_noblock    = false;
            execution_parameters.run_recurrently = false;
            execution_parameters.recurrent_delay = 0;
            execution_parameters.new_command     = false;
        }

        /**
         * @brief If possible, ensures that the module has an active command to execute.
         *
         * @note Uses the following order of preference to activate (execute) a command:
         * finish already running commands > run new commands > repeat a previously executed recurrent command.
         * When repeating recurrent commands, the method ensures the recurrent timeout has expired before reactivating
         * the command.
         *
         * @returns bool @b true if the module has a command to execute and @b false otherwise.
         */
        bool ResolveActiveCommand()
        {
            // If the command field is not 0, this means there is already an active command being executed and no
            // further action is necessary.
            if (execution_parameters.command != 0) return true;

            // If there is no active command and the next_command field is set to 0, this means that the module does
            // not have any new or recurrent commands to execute.
            if (execution_parameters.next_command == 0) return false;

            // If there is a next command in the queue and the new_command flag is set to true, activates the queued
            // command without any further condition.
            if (execution_parameters.new_command)
            {
                // Transfers the command and the noblock flag from buffer fields to active fields
                execution_parameters.command = execution_parameters.next_command;
                execution_parameters.noblock = execution_parameters.next_noblock;

                // Sets active command stage to 1, which is a secondary activation mechanism. All multi-stage commands
                // should start with stage 1, as stage 0 is reserved for communicating no active commands sate.
                execution_parameters.stage = 1;

                // Removes the new_command flag to indicate that the new command has been consumed.
                execution_parameters.new_command = false;

                return true;  // Returns true to indicate there is a command to run.
            }

            // If no new command is available, recurrent activation is enabled, and the requested recurrent_delay
            // number of microseconds has passed, re-activates the previously executed command. Note, the
            // next_command != 0 check is here to support correct behavior in response to Dequeue command, which sets
            // the next_command field to 0 and should be able to abort cyclic and non-cyclic command execution.
            if (execution_parameters.run_recurrently &&
                execution_parameters.recurrent_timer > execution_parameters.recurrent_delay &&
                execution_parameters.next_command != 0)
            {
                // Repeats the activation steps from above, minus the new_command flag modification (command is not new)
                execution_parameters.command = execution_parameters.next_command;
                execution_parameters.noblock = execution_parameters.next_noblock;
                execution_parameters.stage   = 1;
                return true;  // Indicates there is a command to run.
            }

            // The only way to reach this point is to have a recurrent command with an unexpired recurrent delay timer.
            // Returns false to indicate that no command was activated.
            return false;
        }

        /**
         * @brief Resets the module's command queue and aborts any currently running commands.
         */
        void ResetExecutionParameters()
        {
            // Rests the execution_parameters structure back to default values
            execution_parameters.command         = 0;
            execution_parameters.stage           = 0;
            execution_parameters.noblock         = false;
            execution_parameters.next_command    = 0;
            execution_parameters.next_noblock    = false;
            execution_parameters.new_command     = false;
            execution_parameters.run_recurrently = false;
            execution_parameters.recurrent_delay = 0;
            execution_parameters.recurrent_timer = 0;
            execution_parameters.delay_timer     = 0;
        }

        /**
         * @brief Returns the ID of the instance.
         */
        [[nodiscard]]
        uint8_t GetModuleID() const
        {
            return _module_id;
        }

        /**
         * @brief Returns the type (family ID) of the instance.
         */
        [[nodiscard]]
        uint8_t GetModuleType() const
        {
            return _module_type;
        }

        /**
         * @brief Returns the combined type and id value of the instance.
         */
        [[nodiscard]]
        uint16_t GetModuleTypeID() const
        {
            return _module_type_id;
        }

        /**
         * @brief Sends an error message to notify the PC that the instance did not recognize the active command.
         */
        void SendCommandActivationError() const
        {
            // Sends an error message that uses the unrecognized command code as 'command' and a 'not recognized' error
            // code as the event.
            SendData(static_cast<uint8_t>(kCoreStatusCodes::kCommandNotRecognized));
        }

        // VIRTUAL METHODS.
        // These methods allow the Kernel class to interface with the custom logic of each custom hardware module
        // instance, integrating them with the rest of the library components. The implementation of these methods
        // relies on the end user as it has to be specific to each custom hardware module.

        /**
         * @brief Overwrites the memory of the object used to store the instance's runtime parameters with the data
         * received from the PC.
         *
         * @note This method should call the ExtractParameters() method inherited from the base
         * Module class to unpack the received custom parameters message into the structure (object) used to store the
         * instance's custom runtime parameters.
         *
         * @returns bool @b true if new parameters were parsed successfully and @b false otherwise.
         *
         * Example method implementation:
         * @code
         * uint8_t custom_parameters_object[3] = {}; // Assume this object was created at class instantiation.
         *
         * // Reads the data and returns the operation status to the caller (Kernel).
         * return ExtractParameters(custom_parameters_object);
         * @endcode
         */
        virtual bool SetCustomParameters() = 0;

        /**
         * @brief Executes the instance method associated with the currently active command.
         *
         * @note This method should translate the currently active command returned by the GetActiveCommand()
         * method inherited from the base Module class into the call to the command-specific method that executes the
         * command's logic.
         *
         * @warning This method should not evaluate whether the command ran successfully, only whether the command
         * was recognized and matched to the appropriate method call. The called method should use the inherited
         * SendData() method to report command runtime status to the PC.
         *
         * @returns bool @b true if the currently active module command was matched to a specific custom method and @b
         * false otherwise.
         *
         * Example method implementation:
         * @code
         * uint8_t active_command = GetActiveCommand();  // Returns the code of the currently active command.
         *
         * // Matches the active command to the appropriate method call.
         * switch (active_command) {
         *  case 1:
         *      command_1();
         *      return true; // This returns value means the command was recognized, not that it succeeded!
         *  case 2:
         *      bool success = command_2(11);
         *      if (success) command_3();
         *      return true;
         *  default:
         *      // The only case for returning 'false' should be not recognizing the command code.
         *      return false;
         * }
         * @endcode
         */
        virtual bool RunActiveCommand() = 0;

        /**
         * @brief Sets up the instance's hardware and software assets.
         *
         * @note This method should set the initial (default) state of the instance's custom parameter structures and
         * hardware (pins, timers, etc.).
         *
         * @attention Ideally, this method should not contain any logic that can fail or block, as this method is called
         * as part of the initial library runtime setup procedure, before the communication interface is fully
         * initialized.
         *
         * @returns bool @b true if the setup method ran successfully and @b false otherwise.
         *
         * Example method implementation:
         * @code
         * // Resets custom software assets:
         * uint8_t custom_parameters_object[3] = {5, 5, 5}; // Assume this object was created at class instantiation.
         * custom_parameters_object[0] = 0;  // Reset the first byte of the object to zero.
         * custom_parameters_object[1] = 0;  // Reset the second byte of the object to zero.
         * custom_parameters_object[2] = 0;  // Reset the third byte of the object to zero.
         *
         * // Resets hardware assets:
         * const uint8_t output_pin = 12;
         * pinMode(output_pin, OUTPUT);  // Sets the output pin as output.
         * return true;
         * @endcode
         */
        virtual bool SetupModule() = 0;

        /// Destroys the instance during cleanup.
        virtual ~Module() = default;

    protected:
        /// Stores the byte-code for the type (family) of the module. All modules in the family share the same type
        /// code.
        const uint8_t _module_type;

        /// Stores the ID byte-code of the specific module instance. This code has to be unique within the same module
        /// family.
        const uint8_t _module_id;

        /// Combines type and id byte-codes into a single uint16 value that is expected to be unique for each module
        /// instance active at the same time (across all module types).
        const uint16_t _module_type_id = _module_type << 8 | _module_id;

        /// A reference to the shared instance of the Communication class. This class is used to send module runtime
        /// data to the PC.
        Communication& _communication;

        /// A reference to the shared instance of the DynamicRuntimeParameters structure. This structure stores
        /// PC-addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        const DynamicRuntimeParameters& _dynamic_parameters;

        // UTILITY METHODS.
        // These methods are designed to help developers with writing custom module classes. They are not accessed by
        // the Kernel class and are not required for integrating the custom module with the rest of the library. It is
        // highly recommended to only access inherited (base) Module properties and attributes through these utility
        // methods however, as it adds an extra layer of security.

        /**
         * @brief Returns the code of the currently active (running) command.
         *
         * If there are no active commands, the returned code will be 0.
         *
         * @note Use this method when implementing RunActiveCommand() virtual method to learn the code of the command
         * that needs to be executed.
         */
        [[nodiscard]]
        uint8_t GetActiveCommand() const
        {
            return execution_parameters.command;
        }

        /// This method terminates the currently active command and ensures it will not run again if it is recurrent.
        /// Use this method to abort the runtime of a command that runs into an error that is likely not recoverable to
        /// ensure the command does not run again.
        void AbortCommand()
        {
            // Ensures the failed command is cleared from the recurrent queue. If the new_command flag is true,
            // this indicates that the current command will be replaced with a new command anyway, so clearing
            // the recurrent activation is not necessary.
            if (!execution_parameters.new_command) ResetCommandQueue();
            CompleteCommand();  // Finishes the command execution and sends the completion message to the PC.
        }

        /**
         * @brief Resets the timer that tracks the delay between active command stages.
         *
         * This timer is primarily used by methods that block for a period of time or until an external trigger is
         * encountered. This method resets the timer to 0. It is called automatically when AdvanceCommandStage() method
         * is called, but it is also exposed as a separate method for user convenience.
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
            if (execution_parameters.command != 0) return execution_parameters.stage;

            // Otherwise returns 0 to indicate there is no actively running command
            return 0;
        }

        /**
         * @brief Completes (ends) the currently active (running) command execution.
         *
         * This method should only be called when the command method completes everything it sets out to do. For noblock
         * commands, the microcontroller may need to loop through the command method multiple times before it reaches
         * the algorithmic endpoint. In this case, the call to this method should be protected by an 'if' statement that
         * ensures it is only called when the command has finished it's work (the command stage reached the necessary
         * endpoint).
         *
         * @warning It is essential that this method is called at the end of every command function to allow executing
         * other commands. Failure to do so can completely deadlock the Module and, in severe cases, the whole
         * Microcontroller.
         */
        void CompleteCommand()
        {
            // Resolves and, if necessary, notifies the PC that the active command has been completed. This is only done
            // for commands that have finished their runtime. Specifically, recurrent commands do not report completion
            // until they are canceled or replaced by a new command. One-shot commands always report completion.
            if (execution_parameters.new_command || execution_parameters.next_command == 0 ||
                !execution_parameters.run_recurrently)
            {
                // Since this automatically accesses execution_parameters.command for command code, this has to be
                // called before resetting the command field.
                SendData(static_cast<uint8_t>(kCoreStatusCodes::kCommandCompleted));
            }

            execution_parameters.command = 0;  // Removes active command code
            execution_parameters.stage   = 0;  // Secondary deactivation step, stage 0 is not a valid command stage
            execution_parameters.recurrent_timer =
                0;  // Resets the recurrent command timer when the command is completed

            // If the command that has just been completed is not a recurrent command and there is no new command,
            // resets the command queue to clear out the completed command data.
            if (!execution_parameters.new_command && !execution_parameters.run_recurrently) ResetCommandQueue();
        }

        /**
         * @brief Polls and (optionally) averages the value(s) of the specified analog pin.
         *
         * @param pin The number (id) of the analog pin to read.
         * @param pool_size The number of pin readout values to average into the final readout. Set to 0 or 1 to
         * disable averaging.
         *
         * @returns uint16_t value of the analog sensor. The actual resolution of the returned signal value depends on
         * teh microcontroller's ADC resolution parameter.
         */
        static uint16_t AnalogRead(const uint8_t pin, const uint16_t pool_size = 0)
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
         * @brief Polls and (optionally) averages the value(s) of the specified digital pin.
         *
         * @note Digital readout averaging is primarily helpful for controlling jitter and backlash noise.
         *
         * @param pin The number (id) of the digital pin to read.
         * @param pool_size The number of pin readout values to average into the final readout. Set to 0 or 1 to
         * disable averaging.
         *
         * @returns the value of the pin as @b true (HIGH) or @b false (LOW).
         */
        static bool DigitalRead(const uint8_t pin, const uint16_t pool_size = 0)
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
         * @brief Checks if the input duration of microseconds has passed since the module's delay_timer has been reset.
         *
         * Depending on execution configuration, the method can block in-place until the escape duration has passed or
         * function as a non-blocking check for whether the required duration of microseconds has passed
         * (for noblock runtimes).
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
                while (execution_parameters.delay_timer <= delay_duration);
            }

            // Evaluates whether the requested number of microseconds has passed. If the duration was enforced above,
            // this check will always be true.
            if (execution_parameters.delay_timer >= delay_duration)
            {
                return true;
            }

            // If the requested duration has not passed, returns false
            return false;
        }

        /**
         * @brief Sets the input digital pin to the specified value (High or Low).
         *
         * This utility method is used to control the value of a digital pin configured to output a constant High or Low
         * level signal. Internally, the method checks for appropriate output locks, based on the pin type, before
         * setting its value.
         *
         * @param pin The number (id) of the digital pin to interface with.
         * @param value The boolean value to set the pin to.
         * @param ttl_pin Determines whether the pin is used to drive hardware actions or for TTL communication. This
         * is necessary to resolve whether action_lock or ttl_lock dynamic runtime parameters apply to the pin. When
         * applicable, either of these parameters prevents changing the pin output value.
         *
         * @returns bool @b true if the pin has been set to the requested value and @b false if the pin is locked and
         * has not been set.
         */
        [[nodiscard]]
        bool DigitalWrite(const uint8_t pin, const bool value, const bool ttl_pin) const
        {
            // If the appropriate lock parameter for the pin is enabled, returns false to indicate that the pin is
            // locked.
            if ((ttl_pin && _dynamic_parameters.ttl_lock) || (!ttl_pin && _dynamic_parameters.action_lock))
                return false;

            // If the pin is not locked, sets it to the requested value and returns true to indicate tha the pin was
            // triggered successfully.
            digitalWriteFast(pin, value);
            return true;
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
         * @param pin The number (id) of the analog pin to interface with.
         * @param value The duty cycle value from 0 to 255 that controls the proportion of time during which the pin
         * is HIGH. Note, the exact meaning of each duty-cycle step depends on the clock that is used to control the
         * analog pin cycling behavior.
         * @param ttl_pin Determines whether the pin is used to drive hardware actions or for TTL communication. This
         * is necessary to resolve whether action_lock or ttl_lock dynamic runtime parameters apply to the pin. When
         * applicable, either of these parameters prevents changing the pin output value.
         *
         * @returns bool @b true if the pin has been set to the requested value and @b false if the pin is locked and
         * has not been set.
         */
        [[nodiscard]]
        bool AnalogWrite(const uint8_t pin, const uint8_t value, const bool ttl_pin) const
        {
            // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to 0
            // (constantly off) and returns the current value of the pin (0).
            if ((ttl_pin && _dynamic_parameters.ttl_lock) || (!ttl_pin && _dynamic_parameters.action_lock))
                return false;

            // If the pin is not locked, sets it to the requested value and returns the new value of the pin.
            analogWrite(pin, value);
            return true;
        }

        /**
         * @brief Packages and sends the provided event_code and data object to the PC via the Communication class
         * instance.
         *
         * This method simplifies sending data through the Communication class by automatically resolving most of the
         * payload metadata. This method guarantees that the formed payload follows the correct format and contains
         * the necessary data. In turn, this allows custom module developers to focus on custom module code, abstracting
         * away interaction with the Communication class.
         *
         * @note It is highly recommended to use this method for sending data to the PC instead of using the
         * Communication class directly. If the data you are sending does not need a data object, use the overloaded
         * version of this method that only accepts the event_code to send.
         *
         * @warning If sending the data fails for any reason, this method automatically emits an error message. Since
         * that error message may itself fail to be sent, the method also statically activates the built-in LED of the
         * board to visually communicate encountered runtime error. Do not use the LED-connected pin or LED when using
         * this method to avoid interference!
         *
         * @tparam ObjectType The type of the data object to be sent along with the message. This is inferred
         * automatically by the template constructor.
         * @param event_code The byte-code specifying the event that triggered the data message.
         * @param prototype The prototype byte-code specifying the structure of the data object. Currently, all data
         * objects have to use one of the supported prototype structures. If you need to add additional prototypes,
         * modify the kPrototypes enumeration available from the axmc_communication_assets namespace.
         * @param object Additional data object to be sent along with the message. The structure of the object has to
         * match the object structure declared by the prototype code for the PC to deserialize the object.
         */
        template <typename ObjectType = void>
        void SendData(
            const uint8_t event_code,
            const axmc_communication_assets::kPrototypes prototype,
            const ObjectType& object
        )
        {
            // Packages and sends the data to the connected system via the Communication class
            const bool success = _communication.SendDataMessage(
                _module_type,
                _module_id,
                execution_parameters.command,
                event_code,
                prototype,
                object
            );

            // If the message was sent, ends the runtime
            if (success) return;

            // If the message was not sent, calls a method that attempts to send a communication error message to the
            // PC and turns on the built-in LED to visually indicate the error.
            _communication.SendCommunicationErrorMessage(
                _module_type,
                _module_id,
                execution_parameters.command,
                static_cast<uint8_t>(kCoreStatusCodes::kTransmissionError)
            );
        }

        /**
         * @brief Packages and sends the provided event_code to the PC via the Communication class instance.
         *
         * This method overloads the SendData() method to optimize transmission in cases where there is no additional
         * data to include with the state event-code.
         *
         * @param event_code The byte-code specifying the event that triggered the data message.
         */
        void SendData(const uint8_t event_code) const
        {
            // Packages and sends the data to the connected system via the Communication class
            const bool success =
                _communication.SendStateMessage(_module_type, _module_id, execution_parameters.command, event_code);

            // If the message was sent, ends the runtime
            if (success) return;

            // If the message was not sent, calls a method that attempts to send a communication error message to the
            // PC and turns on the built-in LED to visually indicate the error.
            _communication.SendCommunicationErrorMessage(
                _module_type,
                _module_id,
                execution_parameters.command,
                static_cast<uint8_t>(kCoreStatusCodes::kTransmissionError)
            );
        }

        /**
         * @brief Updates the memory of the provided object with the new PC-addressable runtime parameter data
         * received from the PC.
         *
         * @tparam ObjectType The type of the storage_object. This is inferred automatically by the template
         * constructor.
         * @param storage_object The object used to store PC-addressable custom runtime parameters.
         * @return @bool True if the parameters were successfully extracted, false otherwise.
         */
        template <typename ObjectType>
        bool ExtractParameters(ObjectType& storage_object)
        {
            return _communication.ExtractModuleParameters(storage_object);
        }
};

#endif  //AXMC_MODULE_H
