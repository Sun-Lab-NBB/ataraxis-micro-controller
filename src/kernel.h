/**
 * @file
 * @brief Provides the Kernel class used to manage the runtime of custom hardware modules and
 * integrate them with the companion host-computer (PC) control interface.
 *
 * @section kern_description Description:
 *
 * This class manages PC-microcontroller communication and schedules and executes commands addressed to custom hardware
 * modules. Due to the static API exposed by the (base) Module class, from which all custom module instances should
 * inherit, Kernel seamlessly integrates custom hardware modules with the centralized interface running on the
 * host-computer (PC).
 *
 * @note A single instance of this class should be created in the main.cpp file and used to manage the runtime. See
 * the example .cpp files included with the distribution of the library or
 * the https://github.com/Sun-Lab-NBB/ataraxis-micro-controller repository for details.
 */

#ifndef AXMC_KERNEL_H
#define AXMC_KERNEL_H

// Dependencies
#include <Arduino.h>
#include <digitalWriteFast.h>
#include "axmc_shared_assets.h"
#include "communication.h"
#include "module.h"

using namespace axmc_shared_assets;

/**
 * @brief Manages the runtime of one or more custom hardware module instances.
 *
 * The Kernel integrates all custom hardware module instances with the centralized control interface running on the
 * companion host-computer (PC). It handles the majority of the microcontroller-PC interactions.
 *
 * @warning After initialization, call the instance's Setup() method in the main setup() function and the RuntimeCycle()
 * method in the main loop() function of the main.cpp / main.ino file.
 *
 * @note During initialization, this class should be provided with an array of hardware module instances
 * that inherit from the Module class.
 *
 * Example Instantiation:
 * @code
 * Communication axmc_communication(Serial);  // Communication class first
 * TestModule test_module(1, 1, axmc_communication);  // Example custom module
 * Module* modules[] = {&test_module};  // Packages the module(s) into an array to be provided to the Kernel class
 *
 * // The Kernel class should always be instantiated last
 * const uint8_t controller_id = 123;  // Example controller ID
 * const uin32_t keepalive_interval = 1000;  // Example keepalive interval (1 second)
 * Kernel kernel_instance(controller_id, axmc_communication, modules, keepalive_interval);
 * @endcode
 */
class Kernel
{
    public:
        /**
         * @enum kKernelStatusCodes
         * @brief Defines the codes used by the Kernel class to communicate its runtime state to the PC.
         */
        enum class kKernelStatusCodes : uint8_t
        {
            kStandBy                = 0,  ///< Currently not used. Statically reserves 0 to NOT be a valid code.
            kSetupComplete          = 1,  ///< Setup() method runtime succeeded.
            kModuleSetupError       = 2,  ///< Setup() method runtime failed due to a module setup error.
            kReceptionError         = 3,  ///< Encountered a communication error when receiving data from the PC.
            kTransmissionError      = 4,  ///< Encountered a communication error when sending data to the PC.
            kInvalidMessageProtocol = 5,  ///< Received message uses an unsupported (unknown) protocol.
            kModuleParametersSet    = 6,  ///< Received and applied the parameters addressed to the module instance.
            kModuleParametersError  = 7,  ///< Unable to apply the received parameters to the module instance.
            kCommandNotRecognized   = 8,  ///< Received and unsupported (unknown) Kernel command.
            kTargetModuleNotFound   = 9,  ///< Unable to find the module with the requested combined type and ID code.
            kKeepAliveTimeout       = 10  ///< The Kernel did not receive a keepalive message within the expected time.
        };

        /**
         * @enum kKernelCommands
         * @brief Defines the codes for the supported Kernel's commands.
         */
        enum class kKernelCommands : uint8_t
        {
            kStandby            = 0,  ///< The standby code used during class initialization.
            kReceiveData        = 1,  ///< Checks and, if possible, receives PC-sent data. Not externally addressable.
            kResetController    = 2,  ///< Resets the software and hardware state of all managed assets.
            kIdentifyController = 3,  ///< Sends the ID of the controller to the PC.
            kIdentifyModules    = 4,  ///< Sequentially sends each managed module's combined Type+ID code to the PC.
            kKeepAlive          = 5,  ///< Resets the keepalive watchdog timer, starting a new keepalive cycle.
        };

        /// Tracks the currently active Kernel command. This is used to send data and error messages to the PC.
        uint8_t kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);

        /**
         * @brief Initializes the necessary assets used to manage the runtime of the input hardware module instances.
         *
         * @param controller_id The unique identifier of the microcontroller that uses this Kernel instance. This
         * ID code has to be unique for all microcontrollers used at the same time.
         * @param communication The shared Communication instance used to bidirectionally communicate with the PC
         * during runtime.
         * @param module_array The array of pointers to custom hardware module instances. Note, each instance must
         * inherit from the base Module class.
         * @param keepalive_interval The interval, in milliseconds, within which the Kernel must receive a keepalive
         * command from the PC. If the Kernel does not receive the command within two consecutive intervals, it
         * conducts an emergency reset procedure and assumes that communication with the PC has been lost. Setting this
         * parameter to 0 disables the keepalive mechanism.
         */
        template <const size_t kModuleNumber>
        Kernel(
            const uint8_t controller_id,
            Communication& communication,
            Module* (&module_array)[kModuleNumber],
            const uint32_t keepalive_interval = 0
        ) :
            _modules(module_array),
            _module_count(kModuleNumber),
            _controller_id(controller_id),
            _keepalive_interval(keepalive_interval * 2),  // Doubles the interval to allow brief communication lapses
            _communication(communication)
        {
            // While compiling an empty array should not be possible, ensures there is always at least one module to
            // manage.
            static_assert(
                kModuleNumber > 0,
                "At least one valid Module-derived class instance must be provided during Kernel class initialization."
            );
        }

        /**
         * @brief Configures the hardware and software assets used by the Kernel and all managed hardware modules.
         *
         * @note This method has to be called as part of the main setup() function.
         *
         * @warning This is the only method that turns off the built-in LED of the controller board. Seeing
         * the LED constantly ON (HIGH) after this method's runtime means the controller experienced a communication
         * error when it tried sending data to the PC. Seeing the LED blinking with ~2-second periodicity indicates that
         * the Kernel failed the setup sequence.
         */
        void Setup()
        {
            kernel_command = static_cast<uint8_t>(kKernelCommands::kResetController);  // Sets active command code

            // Ensures that the setup tracker is inactivated before running the rest of the setup code. This is needed
            // to support correct cycling through Setup() calls on Teensy board that do not reset on USB connection
            // cycling and to properly handle PC-sent 'reset' commands. As a safety feature, this bricks the
            // controller if any managed module reports a failure to setup.
            _setup_complete = false;

            // Loops over each module and calls its SetupModule() virtual method. Note, expects that setup methods
            // generally cannot fail, but supports non-success return codes.
            for (size_t i = 0; i < _module_count; i++)
            {
                if (!_modules[i]->SetupModule())
                {
                    // If the setup fails, sends an error message to notify the PC of the setup failure.
                    const uint8_t error_object[2] = {
                        _modules[i]->GetModuleType(),
                        _modules[i]->GetModuleID(),
                    };

                    SendData(
                        static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupError),
                        kPrototypes::kTwoUint8s,
                        error_object
                    );

                    // Returns without completing the setup. This 'bricks' the controller requiring firmware reset
                    // before it can re-attempt the setup process and receive data from the PC.
                    return;
                }

                // Also resets the execution parameters of each module. This step cannot fail.
                _modules[i]->ResetExecutionParameters();
            }

            // Sets up the hardware managed by the Kernel. This is done last to, if necessary, override any
            // module-derived modifications of the hardware reserved by the Kernel. This method cannot fail.
            SetupKernel();

            _setup_complete = true;  // Sets the setup tracker to indicate that the setup process has been completed.

            // Informs the PC that the setup process has been completed.
            SendData(static_cast<uint8_t>(kKernelStatusCodes::kSetupComplete));
        }

        /**
         * @brief Carries out a single runtime cycle.
         *
         * During each runtime cycle, the instance first receives and processes all messages sent from the PC. All
         * messages other than commands addressed to the managed hardware modules are processed and handled immediately.
         * For example, Kernel-addressed commands are executed as soon as they are received. Module-addressed commands
         * are queued for execution and are executed after all available data is received and parsed.
         *
         * Once all data is received, the method loops over managed modules and executes one command execution stage
         * for each module.
         *
         * @note This method has to be repeatedly called as part of the main loop() function.
         */
        void RuntimeCycle()
        {
            // Tracks whether Kernel hardware setup was completed. This only needs to run once, since if the Kernel ever
            // fails setup, it bricks the controller until the firmware is reset.
            static bool once = true;

            // If the Setup method was not called, sets up the built-in LED control via the Kernel-specific setup
            // sequence known to be fail-prone. This is only done once.
            if (!_setup_complete && once)
            {
                SetupKernel();
                once = false;  // Sets the flag to indicate that the setup was completed.
            }

            // If the method is called before the Setup() method, instead of normal runtime continuously blinks the
            // LED to visually communicate setup error to the user.
            if (!_setup_complete)
            {
                digitalWriteFast(LED_BUILTIN, HIGH);  // Turns the LED on
                delay(2000);                          // Delays for 2 seconds
                digitalWriteFast(LED_BUILTIN, LOW);   // Turns the LED off
                delay(2000);                          // Delays for 2 seconds
                return;                               // Ends cycle. A firmware reset is needed to get out of this loop.
            }

            // Continuously parses the data received from the PC until all data is processed.
            kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);
            while (true)
            {
                const uint8_t protocol = ReceiveData();  // Attempts to receive the data
                bool break_loop = false;  // A flag used to break the while loop once all available data is received.
                int16_t target_module;    // Stores the index of the module targeted by Module-addressed command.
                uint8_t return_code;      // Stores the return code of the received message.

                // Uses the message protocol of the returned message to execute appropriate logic to handle the message.
                switch (static_cast<kProtocols>(protocol))
                {
                    // Returned protocol 0 indicates that the data was not received. This may be due to no data to
                    // receive or due to a reception pipeline error. In either case, this ends the reception loop.
                    case kProtocols::kUndefined: break_loop = true; break;

                    case kProtocols::kModuleParameters:
                        return_code = _communication.module_parameters_header.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // For module-addressed commands, attempts to resolve (discover) the addressed module. If this
                        // method succeeds, it returns an index (>=0) of the target module class inside the _modules
                        // array
                        target_module = ResolveTargetModule(
                            _communication.module_parameters_header.module_type,
                            _communication.module_parameters_header.module_id
                        );

                        // Aborts early if the target module is not found, as indicated by the returned code being
                        // a negative number (-1).
                        if (target_module < 0) break;

                        // Calls the Module API method that processes the parameter object included with the message
                        if (!_modules[static_cast<size_t>(target_module)]->SetCustomParameters())
                        {
                            // If the module fails to process the parameters, as indicated by the API method returning
                            // 'false', sends an error message to the PC to communicate the error.
                            const uint8_t error_object[2] = {
                                _modules[static_cast<size_t>(target_module)]->GetModuleType(),
                                _modules[static_cast<size_t>(target_module)]->GetModuleID(),
                            };
                            SendData(
                                static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersError),
                                kPrototypes::kTwoUint8s,
                                error_object
                            );
                        }

                        // If the parameters were set correctly, notifies the PC.
                        SendData(static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersSet));
                        break;

                    case kProtocols::kKernelCommand:
                        return_code = _communication.kernel_command.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // This method resolves and executes the command logic. It automatically extracts the command
                        // code from the received message stored in the Communication class attribute.
                        RunKernelCommand();
                        break;

                    case kProtocols::kDequeueModuleCommand:
                        return_code = _communication.module_dequeue.return_code;
                        if (return_code) SendReceptionCode(return_code);
                        target_module = ResolveTargetModule(
                            _communication.module_dequeue.module_type,
                            _communication.module_dequeue.module_id
                        );

                        // Aborts early if the target module is not found, as indicated by the returned code being
                        // a negative number (-1).
                        if (target_module < 0) break;

                        // Resets the queue of the target module. Note, this does not abort already running commands:
                        // they are allowed to finish gracefully.
                        _modules[static_cast<size_t>(target_module)]->ResetCommandQueue();
                        break;

                    case kProtocols::kOneOffModuleCommand:
                        return_code = _communication.one_off_module_command.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        target_module = ResolveTargetModule(
                            _communication.one_off_module_command.module_type,
                            _communication.one_off_module_command.module_id
                        );

                        if (target_module < 0) break;

                        // Uses an overloaded QueueCommand method that always sets the input command as non-recurrent.
                        _modules[static_cast<size_t>(target_module)]->QueueCommand(
                            _communication.one_off_module_command.command,
                            _communication.one_off_module_command.noblock
                        );
                        break;

                    case kProtocols::KRepeatedModuleCommand:
                        return_code = _communication.repeated_module_command.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        target_module = ResolveTargetModule(
                            _communication.repeated_module_command.module_type,
                            _communication.repeated_module_command.module_id
                        );

                        if (target_module < 0) break;

                        // Uses the non-overloaded QueueCommand method that always sets the input command to execute
                        // recurrently.
                        _modules[static_cast<size_t>(target_module)]->QueueCommand(
                            _communication.repeated_module_command.command,
                            _communication.repeated_module_command.noblock,
                            _communication.repeated_module_command.cycle_delay
                        );
                        break;

                    default:
                        // If the message protocol is not one of the expected protocols, sends an error message to the
                        // PC.Includes the invalid protocol value in the message.
                        SendData(
                            static_cast<uint8_t>(kKernelStatusCodes::kInvalidMessageProtocol),
                            kPrototypes::kOneUint8,
                            _communication.protocol_code
                        );
                        break;
                }

                // If necessary, breaks the reception loop.
                if (break_loop) break;
            }

            // Once the loop above escapes due to running out of data to receive or a reception error, triggers a method
            // that sequentially executes Module commands in the blocking or non-blocking manner.
            RunModuleCommands();

            // Keepalive status resolution. If the Kernel is configured to require keepalive messages, and it does not
            // receive a keepalive message within the configured interval, sends an error message to the PC and triggers
            // an emergency reset
            if (_keepalive_enabled && (_since_previous_keepalive > _keepalive_interval))
            {
                // Sends an error message to the PC
                SendData(
                    static_cast<uint8_t>(kKernelStatusCodes::kKeepAliveTimeout),
                    kPrototypes::kOneUint32,
                    _keepalive_interval
                );

                // Resets the microcontroller runtime to default parameters, effectively clearing all command buffers
                // and hardware states.
                Setup();
            }
        }

    private:
        /// Stores the managed custom hardware module classes.
        Module** _modules;

        /// Stores the size of the _modules array.
        const size_t _module_count;

        /// Stores the unique identifier code of the microcontroller that uses the Kernel instance.
        const uint8_t _controller_id;

        /// Stores the maximum period of time, in milliseconds, that can separate two consecutive keepalive messages
        /// sent from the PC to the microcontroller.
        const uint32_t _keepalive_interval;

        /// The elapsedMillis instance that tracks the time elapsed since receiving the last keepalive message.
        elapsedMillis _since_previous_keepalive;

        /// Tracks whether the keepalive tracking is enabled.
        bool _keepalive_enabled = false;

        /// The Communication instance  used to bidirectionally communicate with the PC interface.
        Communication& _communication;

        /// Tracks whether the Setup() method has been called to ensure that the instance is properly configured for
        /// runtime.
        bool _setup_complete = false;

        /**
         * @brief If a message sent from the PC is available for reception, decodes it into the Communication's
         * reception buffer.
         *
         * @returns The protocol code of the received message or 0 to indicate that no valid message was received.
         */
        [[nodiscard]]
        uint8_t ReceiveData() const
        {
            // Attempts to receive a message sent by the PC. If reception succeeds, returns the protocol code of the
            // received message
            if (_communication.ReceiveMessage()) return _communication.protocol_code;

            // Data reception can fail for two broad reasons. The first reason is that one of the classes involved in
            // data reception encounters an error. If this happens, the error needs to be reported to the PC. However,
            // it is also not uncommon for the reception method to 'fail' as there is no data to receive. This is not
            // an error and should be handled as a valid 'no need to do anything' case.
            if (_communication.communication_status !=
                static_cast<uint8_t>(kCommunicationStatusCodes::kNoBytesToReceive))
            {
                // For legitimately failed runtimes, sends an error message to the PC.
                _communication.SendCommunicationErrorMessage(
                    static_cast<uint8_t>(kKernelCommands::kReceiveData),
                    static_cast<uint8_t>(kKernelStatusCodes::kReceptionError)
                );
            }

            // Regardless of the source of communication failure, uses 'kUndefined' protocol (0) code return to indicate
            // that no valid data to process was received.
            return static_cast<uint8_t>(kProtocols::kUndefined);
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
        template <typename ObjectType>
        void SendData(const uint8_t event_code, const kPrototypes prototype, const ObjectType& object)
        {
            // Packages and sends the data message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendDataMessage(kernel_command, event_code, prototype, object)) return;

            // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
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
            // Packages and sends the state message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendStateMessage(kernel_command, event_code)) return;

            // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /**
         * @brief Sends the unique identifier code of the microcontroller that uses this Kernel instance to the PC.
         */
        void SendControllerID() const
        {
            _communication.SendServiceMessage<kProtocols::kControllerIdentification>(_controller_id);
        }

        /**
         * @brief Sequentially sends the combined type and ID code for each hardware module instance managed by this
         * Kernel instance to the PC.
         */
        void SendModuleTypeIDs() const
        {
            for (size_t i = 0; i < _module_count; ++i)
            {
                _communication.SendServiceMessage<kProtocols::kModuleIdentification>(_modules[i]->GetModuleTypeID());
            }
        }

        /**
         * @brief Sends the input reception code to the PC.
         *
         * @param reception_code The reception code received as part of an incoming message sent from the PC.
         */
        void SendReceptionCode(const uint8_t reception_code) const
        {
            _communication.SendServiceMessage<kProtocols::kReceptionCode>(reception_code);
        }

        /**
         * @brief Sets up the hardware and software assets managed by the Kernel class.
         */
        void SetupKernel()
        {
            // Configures and deactivates the built-in LED. Currently, this is the only hardware system directly
            // managed by the Kernel.
            pinModeFast(LED_BUILTIN, OUTPUT);
            digitalWriteFast(LED_BUILTIN, LOW);

            // Disables the keepalive tracking.
            _keepalive_enabled = false;
        }

        /**
         * @brief Resolves and calls the method associated with the currently active Kernel command.
         */
        void RunKernelCommand()
        {
            // Resolves and executes the specific requested command
            kernel_command = _communication.kernel_command.command;
            switch (static_cast<kKernelCommands>(kernel_command))
            {
                case kKernelCommands::kResetController: Setup(); return;

                case kKernelCommands::kIdentifyController: SendControllerID(); return;

                case kKernelCommands::kIdentifyModules: SendModuleTypeIDs(); return;

                case kKernelCommands::kKeepAlive:
                    // If necessary, activates the keepalive tracking. Prevents enabling the keepalive tracking if the
                    // interval is set to 0.
                    if (!_keepalive_enabled && _keepalive_interval > 0) _keepalive_enabled = true;

                    // Resets the keepalive interval tracker in-place
                    _since_previous_keepalive = 0;
                    return;

                default:
                    // If the command code was not matched with any valid code, sends an error message.
                    SendData(static_cast<uint8_t>(kKernelStatusCodes::kCommandNotRecognized));
            }
        }

        /**
         * @brief Finds the managed hardware module instance addressed by the input type and id codes.
         *
         * @note If this method is unable to resolve the target module, it automatically sends an error message to the
         * PC in addition to returning the '-1' error code.
         *
         * @param target_type: The type (family) identifier of the addressed module.
         * @param target_id The unique identifier of the addressed module.
         *
         * @return A non-negative integer representing the index of the module in the array of managed modules if the
         * addressed module is found. A '-1' value if the target module was not found.
         */
        int16_t ResolveTargetModule(const uint8_t target_type, const uint8_t target_id)
        {
            // Loops over all managed modules and checks the type and id of each module against the searched parameters.
            for (size_t i = 0; i < _module_count; i++)
            {
                // If the matching module is found, returns its index in the array of managed modules.
                if (_modules[i]->GetModuleType() == target_type && _modules[i]->GetModuleID() == target_id)
                {
                    return static_cast<int16_t>(i);
                }
            }

            // Otherwise, sends an error message to the PC and returns -1 to indicate that the target module was not
            // found.
            const uint8_t errors[2] = {target_type, target_id};
            SendData(static_cast<uint8_t>(kKernelStatusCodes::kTargetModuleNotFound), kPrototypes::kTwoUint8s, errors);
            return -1;
        }

        /**
         * @brief Resolves and, if necessary, executes the active command for each managed hardware module.
         */
        void RunModuleCommands() const
        {
            // Loops over all managed modules
            for (size_t i = 0; i < _module_count; i++)
            {
                // First, determines which command to run, if any. This relies on the following choice hierarchy:
                // finish already active commands > execute a newly queued command > repeat a cyclic command.
                // If this method is able to resolve (activate) a command, it returns 'true'. Otherwise, there is no
                // command to run. If the module has no active command, aborts the iteration early to conserve CPU
                // cycles and speed up looping through modules.
                if (!_modules[i]->ResolveActiveCommand()) continue;

                // If RunActiveCommand is implemented properly, it returns 'true' if it matches the active command
                // code to the method to execute and 'false' otherwise. If the method returns 'false', the Kernel calls
                // an API method to send a predetermined error message to the PC.
                if (!_modules[i]->RunActiveCommand()) _modules[i]->SendCommandActivationError();
            }
        }
};

#endif  //AXMC_KERNEL_H
