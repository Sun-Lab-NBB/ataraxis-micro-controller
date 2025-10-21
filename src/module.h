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
         *
         */
        Module(const uint8_t module_type, const uint8_t module_id, Communication& communication) :
            _module_type(module_type), _module_id(module_id), _communication(communication)
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
         * @brief Executes the instance method associated with the active command.
         *
         * @note This method should translate the active command returned by the GetActiveCommand()
         * method inherited from the base Module class into the call to the command-specific method that executes the
         * command's logic.
         *
         * @warning This method should not evaluate whether the command ran successfully, only whether the command
         * was recognized and matched to the appropriate method call. The called method should use the inherited
         * SendData() method to report command runtime status to the PC.
         *
         * @returns bool @b true if the active module command was matched to a specific custom method and @b
         * false otherwise.
         *
         * Example method implementation:
         * @code
         * uint8_t active_command = GetActiveCommand();  // Returns the code of the active command.
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
         *      // If the command is not recognized, returns 'false'
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
        /// Stores the instance's type (family) identifier code.
        const uint8_t _module_type;

        /// Stores the instance's unique identifier code.
        const uint8_t _module_id;

        /// Stores the instance's combined type and id uint16 code expected to be unique for each module instance
        /// active at the same time.
        const uint16_t _module_type_id = _module_type << 8 | _module_id;

        /// The Communication instance used to send module runtime data to the PC.
        Communication& _communication;

        // UTILITY METHODS.
        // These methods are designed to help end users with writing custom module classes. They are not accessed by
        // the Kernel class and are not required for integrating the custom module with the rest of the library. It is
        // highly recommended to use these utility methods where appropriate, as they are required for the custom
        // modules to support the full range of features provided by the library, such as non-blocking module command
        // execution.

        /**
         * @brief Returns the active (running) command's code or 0, if there are no active commands.
         */
        [[nodiscard]]
        uint8_t GetActiveCommand() const
        {
            return execution_parameters.command;
        }

        /**
         * @brief Terminates the active command (if any).
         *
         * If the aborted command is a recurrent command, the method resets the command queue to ensure that the
         * command is not reactivated until it is re-queued from the PC.
         */
        void AbortCommand()
        {
            // Only resets the command queue if there is no other command to replace the currently executed command when
            // it is completed.
            if (!execution_parameters.new_command) ResetCommandQueue();
            CompleteCommand();  // Finishes the command execution and sends the completion message to the PC.
        }

        /**
         * @brief Advances the stage of the currently executed command.
         *
         * As part of its runtime, the method also resets the stage delay timer, making it a one-stop solution
         * for properly transitioning between command stages.
         */
        void AdvanceCommandStage()
        {
            execution_parameters.stage++;
            execution_parameters.delay_timer = 0;
        }

        /**
         * @brief Returns the execution stage of the active (running) command or 0, if there are no active
         * commands.
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
         * @brief Completes (ends) the active (running) command's execution.
         *
         * @warning Only call this method when the command has completed everything it needed to do. To transition
         * between the stages of the same command, use the AdvanceCommandStage() method instead.
         *
         * @note It is essential that this method is called at the end of every command's method (function) to allow
         * executing other commands. Failure to do so can completely deadlock the Module and, in severe cases, the
         * entire Microcontroller.
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
         * @param pin The analog pin to read.
         * @param pool_size The number of pin readout values to average into the returned value. Set to 0 or 1 to
         * disable averaging.
         *
         * @returns uint16_t The read analog value.
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

                // If averaging is enabled, repeatedly polls the pin the requested number of times.
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
         * @param pin The digital pin to read.
         * @param pool_size The number of pin readout values to average into the returned value. Set to 0 or 1 to
         * disable averaging.
         *
         * @returns bool The read digital value as @b true (HIGH) or @b false (LOW).
         */
        static bool DigitalRead(const uint8_t pin, const uint16_t pool_size = 0)
        {
            bool digital_readout;  // Pre-declares the final output readout

            // Reads the physical sensor value.
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
         * @brief Delays the active command execution for the requested number of microseconds.
         *
         * @warning The delay is timed relative to the last command's execution stage advancement.
         *
         * @note Depending on the active command's configuration, the method can block in-place until the
         * delay has passed or function as a non-blocking check for whether the required duration of microseconds has
         * passed.
         *
         * @param delay_duration The delay duration, in microseconds.
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
         * @brief Packages and sends the provided event_code and data object to the PC.
         *
         * @note If the message is intended to communicate only the event code, do not provide the prototype or the
         * data object. SendData() has an overloaded version specialized for sending event codes that is more efficient
         * than the data-containing version.
         *
         * @warning If sending the data fails for any reason, this method automatically emits an error message. Since
         * that error message may itself fail to be sent, the method also statically activates the built-in LED of the
         * board to visually communicate the encountered runtime error. Do not use the LED-connected pin or LED when
         * using this method to avoid interference!
         *
         * @tparam ObjectType The type of the data object to be sent along with the message.
         * @param event_code The event that triggered the data transmission.
         * @param prototype The type of the data object transmitted with the message. Must be one of the kPrototypes
         * enumeration members.
         * @param object The data object to be sent along with the message.
         */
        template <typename ObjectType = void>
        void SendData(const uint8_t event_code, const kPrototypes prototype, const ObjectType& object)
        {
            // Packages and sends the data to the connected system via the Communication class. If the message was sent,
            // ends the runtime
            if (_communication.SendDataMessage(
                    _module_type,
                    _module_id,
                    execution_parameters.command,
                    event_code,
                    prototype,
                    object
                ))
                return;

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
         * @brief Packages and sends the provided event code to the PC.
         *
         * This method overloads the SendData() method to optimize transmitting messages that only need to communicate
         * the event.
         *
         * @param event_code The code of the event that triggered the data transmission.
         */
        void SendData(const uint8_t event_code) const
        {
            // Packages and sends the data to the connected system via the Communication class. If the message was
            // sent, ends the runtime
            if (_communication.SendStateMessage(_module_type, _module_id, execution_parameters.command, event_code))
                return;

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
         * @brief Unpacks the instance's runtime parameters received from the PC into the specified storage object.
         *
         * @tparam ObjectType The type of the object used to store the PC-addressable module's parameters.
         * @param storage_object The object used to store the PC-addressable module's parameters.
         *
         * @return @bool True if the parameters were successfully unpacked, false otherwise.
         */
        template <typename ObjectType>
        bool ExtractParameters(ObjectType& storage_object)
        {
            return _communication.ExtractModuleParameters(storage_object);
        }
};

#endif  //AXMC_MODULE_H
