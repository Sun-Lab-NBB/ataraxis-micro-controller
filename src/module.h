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
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - elapsedMillis.h for millisecond and microsecond timers.
 * - communication.h for Communication class, which is used to send module runtime data to the connected system.
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_MODULE_H
#define AXMC_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <elapsedMillis.h>
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
         * should be carried out either by the Kernel class or via Core / Utility methods inherited from the base
         * Module class.
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
                elapsedMicros recurrent_timer;  ///< A timer class instance to time recurrent command activation delays.
                elapsedMicros delay_timer;  ///< A timer class instance to time delays between active command stages.
        } execution_parameters;             ///< Stores module-specific runtime flow control parameters.

        /**
         * @enum kCoreStatusCodes
         * @brief Assigns meaningful names to status codes used by the module class.
         *
         * These codes are used by the Core class methods (Methods expected to be accessed by the Kernel class).
         * Utility methods are not accessed by the Kernel class and, therefore, do not need a system for communicating
         * nuanced runtime details to upstream callers.
         *
         * @attention This enumeration only covers status codes used by non-virtual methods inherited from the base
         * Module class. All custom modules should use a separate enumeration to define status codes specific to the
         * custom logic of the module.
         *
         * @note To support unified status code reporting, this enumeration reserves values 0 through 50. All custom
         * status codes should use values from 51 through 250. This way, status codes derived from this enumeration
         * will never clash with 'custom' status codes.
         */
        enum class kCoreStatusCodes : uint8_t
        {
            kStandBy              = 0,  ///< The code used to initialize the module_status variable.
            kDataSendingError     = 1,  ///< An error has occurred when sending a Data message to the PC.
            kStateSendingError    = 2,  ///< An error has occurred when sending a State message to the PC.
            kCommandCompleted     = 3,  ///< Currently active command has been completed and will not be cycled again.
            kCommandNotRecognized = 4,  ///< The RunActiveCommand() method was unable to recognize the command.
        };

        /**
         * @brief Instantiates a new Module class object.
         *
         * @param module_type The ID that identifies the type (family) of the module. All instances of the same custom
         * module class should share this ID. Has to use a value not reserved by other Module-derived classes. Note,
         * valid values start with 1 and extend up to 255. 0 is NOT a valid value!
         * @param module_id This ID is used to identify the specific instance of the module. It can use any non-zero
         * value supported by uint8_t range, as long as no other module in the type (family) uses the same value. As
         * with module_type, 0 is NOT a valid value!
         * @param communication A reference to the Communication class instance that will be used to send module runtime
         * data to the connected system. A single Communication class instance is shared by the Kernel and all managed
         * Modules.
         * @param dynamic_parameters A reference to the DynamicRuntimeParameters structure that stores
         * PC-addressable global runtime parameters that broadly alter the behavior of all modules used by the
         * controller. This structure is modified via the Kernel class, Modules only read the data from the structure.
         *
         * @attention Follow this instantiation order when writing the main .cpp / .ino file for the controller:
         * Communication → Module(s) → Kernel. See the main.cpp included with the library for details.
         */
        Module(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            _module_type(module_type),
            _module_id(module_id),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters)
        {}

        // CORE METHODS.
        // These methods are primarily designed to be used by the Kernel class should never be used by any Module logic
        // (they are exclusively for Kernel to use). The Kernel uses these methods to control the microcontroller
        // runtime and schedule the commands to be executed by the Module.

        /**
         * @brief Queues the input command to be executed by the Module.
         *
         * This method queues the command code to be executed and sets the runtime parameters for the command. Once
         * a command is queued, the Module will execute it as soon as it is done with any currently running command.
         * The Kernel class uses this method to queue commands received from the PC for execution.
         *
         * @attention This method is explicitly designed to queue the commands that need to be run cyclically
         * (recurrently)! There is an overloaded version of this method that does not accept the 'cycle_delay' argument
         * and allows queueing non-cyclic commands.
         *
         * @note This method is explicitly written in a way that allows replacing any already queued command. Since the
         * Module buffer is designed to only hold 1 command a time, this allows replacing the queued command in response
         * to external events.
         *
         * @warning This method does not check whether the input command code is valid. It only saves it to the
         * appropriate field of the execution_parameters structure. The validity check should be carried out by the
         * RunActiveCommand() method.
         *
         * @param command The byte-code of the command to execute.
         * @param noblock Determines whether the queued command will be executed in blocking or non-blocking mode.
         * Non-blocking execution requires the command to make use of the protected Utility methods inherited from the
         * base Module class that support non-blocking command execution delays.
         * @param cycle_delay The number of microseconds to delay between command repetitions when it is executed
         * cyclically (recurrently).
         */
        void QueueCommand(const uint8_t command, const bool noblock, const uint32_t cycle_delay)
        {
            execution_parameters.next_command    = command;
            execution_parameters.next_noblock    = noblock;
            execution_parameters.run_recurrently = true;  // This method explicitly queues recurrent commands.
            execution_parameters.recurrent_delay = cycle_delay;
            execution_parameters.new_command     = true;
        }

        /// Overloads the QueueCommand() method to allow queueing non-cyclic commands without unnecessary arguments.
        void QueueCommand(const uint8_t command, const bool noblock)
        {
            execution_parameters.next_command    = command;
            execution_parameters.next_noblock    = noblock;
            execution_parameters.run_recurrently = false;  // This method explicitly queues non-recurrent commands.
            execution_parameters.recurrent_delay = 0;
            execution_parameters.new_command     = true;
        }

        /// The Kernel uses this method when it receives a Dequeue command for one of the modules. The Kernel then uses
        /// this method to clear the variables used to queue and recurrently cycle commands. This means that the module
        /// will remain idle until a new valid command is queued.
        void ResetCommandQueue()
        {
            execution_parameters.next_command    = 0;
            execution_parameters.next_noblock    = false;
            execution_parameters.run_recurrently = false;
            execution_parameters.recurrent_delay = 0;
            execution_parameters.new_command     = false;
        }

        /**
         * @brief If possible, ensures the module has an active command to run.
         *
         * Specifically, uses the following order of preference to activate (execute) a command:
         * finish already running commands > run new commands > repeat a previous cyclic command.
         * When repeating cyclic commands, the method ensures the recurrent timeout has expired before reactivating
         * the command.
         *
         * @notes The Kernel uses this method to set up the command to be executed when RunActiveCommand() method is
         * called. Additionally, as a form of runtime scheduler optimization, RunActiveCommand() is only called if
         * this method returns true (activates a command). This ensures idle modules do not consume CPU time.
         *
         * @attention Any queued command is considered new until this method activates that command.
         *
         * @returns bool @b true if there is a command to run. @b false otherwise.
         */
        bool ResolveActiveCommand()
        {
            // If the command field is not 0, this means there is already an active command being executed and no
            // further action is necessary.
            if (execution_parameters.command != 0) return true;

            // If there is no active command and the next_command field is set to 0, this means that the module does
            // not have any new or recurrent commands to execute.
            if (execution_parameters.next_command == 0) return false;

            // If there is a next command to queue and the new_command flag is set to true, activates the queued
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

                // Note, when a new command is queued, its 'cycle' flag automatically overrides the flag of any
                // currently running command. In other words, a new command, cyclic or not, will always replace any
                // currently running command, cyclic or not.

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
         * @brief Resets the class execution_parameters structure to default values.
         *
         * This method is designed for Teensy boards that do not reset on UART / USB cycling. The Kernel uses this
         * method to reset the Module between runtimes and when it receives the Reset command.
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
         * @brief Returns the ID of the Module instance.
         */
        [[nodiscard]]
        uint8_t GetModuleID() const
        {
            return _module_id;
        }

        /**
         * @brief Returns the type (family ID) of the Module instance.
         */
        [[nodiscard]]
        uint8_t GetModuleType() const
        {
            return _module_type;
        }

        /**
         * @brief Sends an error message to notify the PC that the module did not recognize an executed command.
         *
         * The Kernel uses this method to notify the PC when RunActiveCommand() API method returns 'false'. In turn,
         * this means that the custom logic of the module did not accept the active command code as valid.
         *
         * @attention This way of handling 'core' Module errors is unique to command activation. Errors with Parameter
         * and Setup virtual method runtimes are communicated through Kernel-sent messages. This is ultimately due to
         * the severity of failing parameters and setup method calls being considerably higher than that of not
         * executing a command. A module that does not recognize a command does nothing. A module which is not set (up)
         * correctly or does not have proper parameters may execute a command in a way that damages the managed physical
         * hardware.
         *
         * @notes Experienced developers can replace the 'default' case of the RunActiveCommand() switch statement with
         * a call to this method and return 'true' to the Kernel. This may save some processing time.
         */
        void SendCommandActivationError() const
        {
            // Sends an error message that uses the unrecognized command code as 'command' and a 'not recognized' error
            // code as event.
            SendData(static_cast<uint8_t>(kCoreStatusCodes::kCommandNotRecognized));
        }

        // VIRTUAL METHODS.
        // Like Core methods, the Virtual methods provide the Kernel class with the API to interface with the Module
        // class instance. Unlike Core methods, these methods are specifically designed to encapsulate the custom
        // logic of each module subclass. Therefore, these methods need to be implemented separately for each class
        // derived from the base Module class. Correctly writing these methods is essential for making your custom
        // module work with the Kernel and, by extension, the whole Ataraxis project.

        /**
         * @brief Overwrites the object used to store custom parameters of the class instance with the data received
         * from the PC.
         *
         * Kernel class calls this method when it receives a ModuleParameters message addressed to this module instance.
         * This method is expected to call the ExtractModuleParameters() method of the bound Communication class
         * ('_communication' attribute) to parse the received data into the Module's custom parameters object. Commonly,
         * the parameter object is a Structure, but it can also be any other valid C++ data object.
         *
         * @returns bool @b true if new parameters were parsed successfully and @b false otherwise.
         *
         * This is an example of how to implement this method (what to put in the method's body):
         * @code
         * uint8_t custom_parameters_object[3] = {0, 0, 0}; // Assume this object was created at class instantiation.
         * bool status = _communication.ExtractModuleParameters(custom_parameters_object);  // Reads data.
         * return status;  // Kernel class resolves both error and success outcomes.
         * @endcode
         */
        virtual bool SetCustomParameters() = 0;

        /**
         * @brief Calls the specific method associated with the currently active command code.
         *
         * Kernel class uses this method to access the custom command-associated logic of each Module instance. This
         * method is expected to contain conditional switch-based logic to call the appropriate custom class method,
         * based on the active command code (retrieved using inherited GetActiveCommand() method).
         *
         * @returns bool @b true if active command was matched to a specific custom method and @b false otherwise.
         * Note, this method should NOT evaluate whether the command ran successfully, only that the active command code
         * was matched to specific custom method. The called custom command method should use SendData() method to
         * report command success / failure status to the PC.
         *
         * This is an example of how to implement this method (what to put in the method's body):
         * @code
         * uint8_t active_command = GetActiveCommand();  // Returns the code of the currently active command.
         * switch (active_command) {
         *  case 5:
         *      // If command 5 runs into an error, it should use the SendData() method to send the error message to the
         *      // connected system. All commands are expected to have 'void' return type.
         *      command_5();
         *      return true; // The 'true' here means that the command was matched to method, not that it succeeded!
         *  case 9:
         *      // All commands should not take any arguments. Any static or dynamic runtime parameters should be
         *      // accessible through the custom 'parameters' object of the class.
         *      command_9();
         *      return true;
         *  default:
         *      // If this method does not recognize the active command code, it should return false. The Kernel class
         *      // will then handle this as an error case.
         *      return false;
         * }
         * @endcode
         */
        virtual bool RunActiveCommand() = 0;

        /**
         * @brief Sets up the software and hardware assets used by the module.
         *
         * Kernel class calls this method after receiving the reset command and during initial controller Setup()
         * function runtime. Use this method to reset both hardware (e.g.: pin modes) and software (e.g.: custom
         * parameter structures and class trackers).
         *
         * @attention Ideally, this method should not contain any logic that can fail or block. Many core dependencies,
         * such as USB / UART communication, are initialized during setup, which may interfere with handling setup
         * errors.
         *
         * @returns bool @b true if the setup method ran successfully and @b false otherwise.
         *
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

        /**
         * @brief A pure virtual destructor method to ensure proper cleanup.
         *
         * Currently, there are no extra cleanup steps other than class deletion itself, which also does not happen,
         * as everything in the codebase so far is allocated statically and collected at runtime termination. Generally
         * safe to reimplement without additional logic.
         */
        virtual ~Module() = default;

    protected:
        /// Represents the type (family) of the module. All modules in the family share the same type code.
        const uint8_t _module_type;

        /// The specific ID of the module. This code has to be unique within the module family, as it identifies
        /// specific module instance.
        const uint8_t _module_id;

        /// A reference to the shared instance of the Communication class. This class is used to send runtime data to
        /// the connected Ataraxis system.
        Communication& _communication;

        /// A reference to the shared instance of the ControllerRuntimeParameters structure. This structure stores
        /// dynamically addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        const shared_assets::DynamicRuntimeParameters& _dynamic_parameters;

        // UTILITY METHODS.
        // These methods are designed to help developers with writing custom module classes. They are not accessed by
        // the Kernel class and, consequently, do not interface with the Module's runtime status tracker. It is highly
        // recommended to only access inherited (base) Module properties and attributes through these utility methods.

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

        /// This method completes the currently active command and ensures it will not run again if it is recurrent.
        /// Use this method to abort the runtime of a command that runs into an error that is likely not recoverable.
        /// Otherwise, if the command sends error messages to the PC, it may overwhelm the communication interface by
        /// spamming the same error message.
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
         * @brief Terminates (ends) the currently active (running) command execution.
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
            if (execution_parameters.new_command || execution_parameters.next_command == 0)
            {
                // Since this automatically accesses execution_parameters.command for command code, this has to be
                // called before resetting the command field.
                SendData(static_cast<uint8_t>(kCoreStatusCodes::kCommandCompleted));
            }

            execution_parameters.command = 0;  // Removes active command code
            execution_parameters.stage   = 0;  // Secondary deactivation step, stage 0 is not a valid command stage
            execution_parameters.recurrent_timer =
                0;  // Resets the recurrent command timer when the command is completed
        }

        /**
         * @brief Polls and (optionally) averages the value(s) of the specified analog pin.
         *
         * @note This method will use the global analog readout resolution set during the main.cpp setup() method
         * runtime.
         *
         * @param pin The number (id) of the analog pin to read.
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
         * function as a check for whether the required duration of microseconds has passed (for noblock runtimes).
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
         * modify the kPrototypes enumeration available from the communication_assets namespace.
         * @param object Additional data object to be sent along with the message. The structure of the object has to
         * match the object structure declared by the prototype code for the PC to deserialize the object.
         */
        template <typename ObjectType = void>
        void
        SendData(const uint8_t event_code, const communication_assets::kPrototypes prototype, const ObjectType& object)
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
                static_cast<uint8_t>(kCoreStatusCodes::kDataSendingError)
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
                static_cast<uint8_t>(kCoreStatusCodes::kStateSendingError)
            );
        }
};

#endif  //AXMC_MODULE_H
