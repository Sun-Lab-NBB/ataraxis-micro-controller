/**
 * @file
 * @brief The header file for the Kernel class, which is used to manage the runtime of custom hardware modules and
 * integrate them with the communication interface running on a companion host-computer (PC).
 *
 * @section kern_description Description:
 *
 * This class manages PC-microcontroller communication and schedules and executes commands addressed to custom hardware
 * modules. Due to the static API exposed by the (base) Module class, from which all custom modules are expected to
 * inherit, Kernel seamlessly integrates custom hardware modules with the centralized interface running on the
 * host-computer (PC).
 *
 * @note A single instance of this class should be created in the main.cpp file and used to manage the runtime. See
 * example .cpp files included with the distribution of the library or
 * the https://github.com/Sun-Lab-NBB/ataraxis-micro-controller repository for details.
 *
 * @section kern_developer_notes Developer Notes:
 * This class functions similar to major OS kernels, although it is comparatively limited in scope. Specifically, it
 * manages the runtime of all compatible modules and handles communication with the centralized PC interface. This
 * class is strongly interconnected with the (base) Module class and the Communication class, so modification to one
 * of these classes will likely require modifying other classes accordingly.
 *
 * @section kern_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - communication.h for Communication class, which is used to bidirectionally communicate with host PC.
 * - module.h for the shared Module class API access (allows interfacing with custom modules).
 * - axmc_shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_KERNEL_H
#define AXMC_KERNEL_H

// Dependencies
#include <Arduino.h>
#include <digitalWriteFast.h>
#include "axmc_shared_assets.h"
#include "communication.h"
#include "module.h"

/**
 * @brief Provides the central runtime loop and methods that control data reception, task scheduling, setup and shutdown
 * of the custom hardware module logic.
 *
 * After initialization, call the Setup() class method in the setup() function and RuntimeCycle() method in the loop()
 * function of the main.cpp / main.ino file. These two methods carry out the steps typically executed during the runtime
 * of these functions for all managed modules and the Kernel class itself.
 *
 * @attention This class functions as the backbone of the library by providing runtime flow control functionality. Any
 * modification to this class should be carried out with extreme caution and will likely require modifying other classes
 * from this library.
 *
 * @note During initialization, this class should be provided with an array of pre-initialized hardware module classes
 * that subclass the (base) Module class. After initialization, the Kernel instance will manage the runtime of these
 * modules via internal methods.
 *
 * Example instantiation (as would found in main.cpp file):
 * @code
 * shared_assets::DynamicRuntimeParameters DynamicRuntimeParameters; // Dynamic runtime parameters structure first
 * Communication axmc_communication(Serial);  // Communication class second
 * TestModule test_module(1, 1, axmc_communication, DynamicRuntimeParameters);  // Example custom module
 * Module* modules[] = {&test_module};  // Packages module(s) into an array to be provided to the Kernel class
 *
 * // Kernel class should always be instantiated last
 * const uint8_t controller_id = 123;  // Example controller ID
 * Kernel kernel_instance(controller_id, axmc_communication, DynamicRuntimeParameters, modules);
 * @endcode
 */
class Kernel
{
    public:
        /**
         * @enum kKernelStatusCodes
         * @brief Specifies the byte-codes for errors and states that can be encountered during Kernel class runtime.
         *
         * The class uses these byte-codes to communicate the class runtime state to the PC.
         */
        enum class kKernelStatusCodes : uint8_t
        {
            kStandBy                = 0,  ///< Currently not used. Statically reserves 0 to NOT be a valid code.
            kSetupComplete          = 1,  ///< Setup() method runtime succeeded.
            kModuleSetupError       = 2,  ///< Setup() method runtime failed due to a module setup error.
            kReceptionError         = 3,  ///< Encountered a communication error when receiving the data from PC.
            kTransmissionError      = 4,  ///< Encountered a communication error when sending the data to PC.
            kInvalidMessageProtocol = 5,  ///< Received message uses an unsupported (unknown) protocol.
            kKernelParametersSet    = 6,  ///< Received and applied the parameters addressed to the Kernel class.
            kModuleParametersSet    = 7,  ///< Received and applied the parameters addressed to a managed Module class.
            kModuleParametersError  = 8,  ///< Unable to apply the received Module parameters.
            kCommandNotRecognized   = 9,  ///< The Kernel has received an unknown command.
            kTargetModuleNotFound   = 10  ///< No module with the requested type and id combination is found.
        };

        /**
         * @enum kKernelCommands
         * @brief Specifies the byte-codes for commands that can be executed during runtime.
         *
         * The class uses these byte-codes during incoming and outgoing PC communication. Specifically, the PC can send
         * addressable command codes to the microcontroller to trigger their execution by the Kernel. Conversely, the
         * Kernel includes the active command code with outgoing messages to notify the PC about the command that
         * produced the communicated Kernel state.
         */
        enum class kKernelCommands : uint8_t
        {
            kStandby            = 0,  ///< Standby code used during class initialization.
            kReceiveData        = 1,  ///< Checks and, if possible, receives PC-sent data. Not externally addressable.
            kResetController    = 2,  ///< Resets the software and hardware state of all modules.
            kIdentifyController = 3,  ///< Transmits the ID of the controller back to caller.
            kIdentifyModules    = 4   ///< Sequentially transmits the unique Type+ID 16-bit code of each managed module.
        };

        /// Tracks the currently active Kernel command. This is used to send data and error messages to the PC.
        uint8_t kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);

        /**
         * @brief Creates a new Kernel class instance.
         *
         * @param controller_id The unique identifier code of the microcontroller that runs this Kernel instance. This
         * ID has to be unique for all microcontrollers that are used at the same time to allow the PC to reliably
         * identify each controller.
         * @param communication The reference to the Communication class instance used to bidirectionally communicate
         * with the PCs. A single Communication class instance should be shared by all classes derived from this
         * library
         * @param dynamic_parameters The reference to the DynamicRuntimeParameters structure that stores PC-addressable
         * global runtime parameters that broadly alter the behavior of all managed hardware modules. This structure is
         * addressable through the MicroControllerInterface instance of the companion PC library.
         * @param module_array The array of pointers to custom hardware module classes (the children inheriting from
         * the base Module class). The Kernel instance will manage the runtime of these modules.
         */
        template <const size_t kModuleNumber>
        Kernel(
            const uint8_t controller_id,
            Communication& communication,
            axmc_shared_assets::DynamicRuntimeParameters& dynamic_parameters,
            Module* (&module_array)[kModuleNumber]
        ) :
            _modules(module_array),
            _module_count(kModuleNumber),
            _controller_id(controller_id),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters)
        {
            // While compiling an empty array should not be possible, ensures there is always at least one module to
            // manage.
            static_assert(
                kModuleNumber > 0,
                "Module number must be greater than 0. At least one valid Module-derived class instance must be "
                "provided during Kernel class initialization."
            );
        }

        /**
         * @brief Configures the hardware and software of the Kernel and all managed modules.
         *
         * This method is an API wrapper around the setup sequence of the Kernel and each managed custom hardware
         * module. If this method is not called before calling RuntimeCycle() method, the microcontroller will
         * not function as expected and its built-in LED will be used to show a blinking error signal. Similarly, if
         * this method is called, but fails, the microcontroller will not execute any command and will display a
         * blinking LED signal to indicate setup error. This method is used both during the initial setup and when
         * resetting the microcontroller.
         *
         * @warning THis method has to be called from the setup() function of the main.cpp / main.ino file.
         *
         * @attention This is the only method that turns off the built-in LED of the controller board. Seeing
         * the LED constantly ON (HIGH) after this method's runtime means the controller experienced a communication
         * error when it tried sending data to the PC. Seeing the LED blinking with ~2-second periodicity indicates that
         * the Kernel failed the setup sequence. If no error message reached the PC and the LED is ON or blinking, debug
         * the communication interface to identify the problem continuing with the runtime.
         */
        void Setup()
        {
            kernel_command = static_cast<uint8_t>(kKernelCommands::kResetController);  // Sets active command code

            // Ensures that the setup tracker is inactivated before running the rest of the setup code. This is needed
            // to support correct cycling through Setup() calls on Teensy board that do not reset on USB connection
            // cycling and to properly handle PC-sent 'reset' commands. Basically, this bricks the controller if any
            // managed module reports a failure to setup, which is a safety feature.
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
                        axmc_communication_assets::kPrototypes::kTwoUint8s,
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
         * During each runtime cycle, the class first receives and processes all messages stored in the serial interface
         * reception buffer. All messages other than Module-targeted commands are processed and handled immediately.
         * For example, Kernel-addressed commands are executed as soon as they are received. Module-addressed commands
         * are queued for execution and are executed after all available data is received and parsed.
         *
         * Once all data is received, the method loops over managed modules and attempts to execute one command stage
         * for each module. This design pattern ensures that PC communication takes precedence over executing local
         * commands.
         *
         * @warning This method has to be repeatedly called from the loop() function of the main.cpp / main.ino file.
         * It aggregates all necessary steps to operate the microcontroller through the centralized PC interface.
         *
         * @attention This method has to be called after calling the Setup() method. If it is called before the setup,
         * the microcontroller will cycle LED HIGH/LOW signal to indicate setup error and will not respond to any
         * further commands until it is reset via power cycling or firmware reupload.
         */
        void RuntimeCycle()
        {
            // Tracks whether Kernel hardware setup was completed. This only needs to run once, since if the Kernel ever
            // fails setup, it bricks the controller until the firmware is reset.
            static bool once = true;

            // If the Setup method was not called, sets up the built-in LED control via Kernel-specific setup sequence
            // known to not be fail-prone. This is only done once
            if (!_setup_complete && once)
            {
                SetupKernel();
                once = false;  // Sets the flag to indicate that the setup was completed.
            }

            // If the method is called before the Setup() method, instead of normal runtime continuously blinks the
            // LED to visually communicate SetUp error to the user.
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
                switch (static_cast<axmc_communication_assets::kProtocols>(protocol))
                {
                    // Returned protocol 0 indicates that the data was not received. This may be due to no data to
                    // receive or due to a reception pipeline error. In either case, this ends the reception loop.
                    case axmc_communication_assets::kProtocols::kUndefined: break_loop = true; break;

                    case axmc_communication_assets::kProtocols::kKernelParameters:

                        // If the message contains a non-zero return code, sends it back to the PC to acknowledge
                        // message reception
                        return_code = _communication.kernel_parameters.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // Replaces the content of the _dynamic_parameters structure with the newly received data.
                        _dynamic_parameters = _communication.kernel_parameters.dynamic_parameters;
                        SendData(static_cast<uint8_t>(kKernelStatusCodes::kKernelParametersSet));
                        break;

                    case axmc_communication_assets::kProtocols::kModuleParameters:
                        return_code = _communication.module_parameter.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // For module-addressed commands, attempts to resolve (discover) the addressed module. If this
                        // method succeeds, it returns an index (>=0) of the target module class inside the _modules
                        // array
                        target_module = ResolveTargetModule(
                            _communication.module_parameter.module_type,
                            _communication.module_parameter.module_id
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
                                axmc_communication_assets::kPrototypes::kTwoUint8s,
                                error_object
                            );
                        }

                        // If the parameters were set correctly, notifies the PC.
                        SendData(static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersSet));
                        break;

                    case axmc_communication_assets::kProtocols::kKernelCommand:
                        return_code = _communication.kernel_command.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // This method resolves and executes the command logic. It automatically extracts the command
                        // code from the received message stored in the Communication class attribute.
                        RunKernelCommand();
                        break;

                    case axmc_communication_assets::kProtocols::kDequeueModuleCommand:
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

                    case axmc_communication_assets::kProtocols::kOneOffModuleCommand:
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

                    case axmc_communication_assets::kProtocols::KRepeatedModuleCommand:
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
                            axmc_communication_assets::kPrototypes::kOneUint8,
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
        }

    private:
        /// An array that stores managed custom hardware module classes. These are the classes derived from the base
        /// Module class that encapsulate and expose the API to interface with specific hardware systems.
        Module** _modules;

        /// Stores the size of the _modules array. Since each module class is likely to be different in terms of its
        /// memory size, the individual classes are accessed using a pointer-reference system. For this system to work
        /// as expected, the overall size of the array needs to be stored separately, which is achieved by this
        /// variable
        const size_t _module_count;

        /// Stores the unique, user-defined ID of the managed microcontroller. This ID will be sent to the PC when it
        /// sends the Identification request. In turn, this allows identifying specific controllers through the serial
        /// connection interface, which is especially helpful during the initial setup.
        const uint8_t _controller_id;

        /// A reference to the shared instance of the Communication class. This class is used to bidirectionally
        /// communicate with the PC interface library.
        Communication& _communication;

        /// A reference to the shared instance of the DynamicRuntimeParameters structure. This structure stores
        /// dynamically addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        axmc_shared_assets::DynamicRuntimeParameters& _dynamic_parameters;

        /// This flag tracks whether the Setup() method has been called before calling the RuntimeCycle() method. This
        /// is used to ensure the kernel is properly configured before running commands.
        bool _setup_complete = false;

        /**
         * @brief Attempts to receive (parse) a message from the data contained in the serial interface reception
         * buffer.
         *
         * The RuntimeCycle method repeatedly calls this method until it returns an undefined protocol code (0).
         *
         * @attention This method automatically attempts to send error messages to the PC if it runs into errors. It
         * also activates the built-in LED to visually communicate encountered runtime errors. Do not use the
         * LED-connected pin in your code when using this class.
         *
         * @returns The byte-code of the protocol used by the received message or 0 to indicate no valid message was
         * received.
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
                static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kNoBytesToReceive))
            {
                // For legitimately failed runtimes, sends an error message to the PC.
                _communication.SendCommunicationErrorMessage(
                    static_cast<uint8_t>(kKernelCommands::kReceiveData),
                    static_cast<uint8_t>(kKernelStatusCodes::kReceptionError)
                );
            }

            // Regardless of the source of communication failure, uses 'kUndefined' protocol (0) code return to indicate
            // that no valid data to process was received.
            return static_cast<uint8_t>(axmc_communication_assets::kProtocols::kUndefined);
        }

        /**
         * @brief Packages and sends the provided event_code and data object to the PC via the Communication class
         * instance.
         *
         * This method simplifies sending data through the Communication class by automatically resolving most of the
         * payload metadata. This method guarantees that the formed payload follows the correct format and contains
         * the necessary data.
         *
         * @note It is highly recommended to use this utility method for sending data to the connected system instead of
         * using the Communication class directly. If the data you are sending does not need a data object, use the
         * overloaded version of this method that only accepts the event_code to send.
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
        template <typename ObjectType>
        void SendData(
            const uint8_t event_code,
            const axmc_communication_assets::kPrototypes prototype,
            const ObjectType& object
        )
        {
            // Packages and sends the data message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendDataMessage(kernel_command, event_code, prototype, object)) return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
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
            // Packages and sends the state message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendStateMessage(kernel_command, event_code)) return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /// Sends the ID code of the managed controller to the PC. This is used to identify specific controllers
        /// connected to different PC USB ports. If data sending fails, attempts sending a communication error and
        /// activates the LED indicator.
        void SendControllerID() const
        {
            // Packages and sends the service message to the PC. If the message was sent, ends the runtime
            if (_communication.SendServiceMessage<axmc_communication_assets::kProtocols::kControllerIdentification>(
                    _controller_id
                ))
                return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /// Loops over all managed modules and sends the combined type+id code of each managed module to the PC. Since
        /// each type+id combination has to be unique for a correctly configured microcontroller, the PC uses this
        /// information to ensure the MicroControllerInterface and the Microcontroller are configured appropriately.
        /// If data sending fails, attempts sending a communication error and activates the LED indicator.
        void SendModuleTypeIDs() const
        {
            for (size_t i = 0; i < _module_count; ++i)
            {
                // Packages and sends the service message to the PC.
                if (!_communication.SendServiceMessage<axmc_communication_assets::kProtocols::kModuleIdentification>(
                        _modules[i]->GetModuleTypeID()
                    ))
                {
                    // If sending one of the messages fails, attempts sending a communication error message before
                    // ending the runtime.
                    _communication.SendCommunicationErrorMessage(
                        kernel_command,
                        static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
                    );
                }
            }
        }

        /// Sends the input reception_code to the PC. This is used to acknowledge the reception of PC-sent Command and
        /// Parameter messages. If data sending fails, attempts sending a communication error and activates the
        /// LED indicator.
        void SendReceptionCode(const uint8_t reception_code) const
        {
            // Packages and sends the service message to the PC. If the message was sent, ends the runtime
            if (_communication.SendServiceMessage<axmc_communication_assets::kProtocols::kReceptionCode>(reception_code
                ))
                return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /**
         * @brief Sets up the hardware and software managed by the Kernel class.
         *
         * This method configures and inactivates the built-in LED (via the board's LED-connected pin) and resets the
         * shared DynamicRuntimeParameters structure fields.
         */
        void SetupKernel() const
        {
            // Configures and deactivates the built-in LED. Currently, this is the only hardware system directly
            // managed by the Kernel.
            pinModeFast(LED_BUILTIN, OUTPUT);
            digitalWriteFast(LED_BUILTIN, LOW);

            // Resets the controller-wide runtime parameters to default values.
            _dynamic_parameters.action_lock = true;
            _dynamic_parameters.ttl_lock    = true;
        }

        /**
         * @brief Determines and executes the requested Kernel command received from the PC.
         */
        void RunKernelCommand()
        {
            // Resolves and executes the specific command code communicated by the message:
            kernel_command = _communication.kernel_command.command;
            switch (static_cast<kKernelCommands>(kernel_command))
            {
                case kKernelCommands::kResetController: Setup(); return;

                case kKernelCommands::kIdentifyController: SendControllerID(); return;

                case kKernelCommands::kIdentifyModules: SendModuleTypeIDs(); return;

                default:
                    // If the command code was not matched with any valid code, sends an error message.
                    SendData(static_cast<uint8_t>(kKernelStatusCodes::kCommandNotRecognized));
            }
        }

        /**
         * @brief Loops through the managed modules and attempts to find the module addressed by the input type and id
         * codes.
         *
         * @param target_type: The type (family) of the addressed module.
         * @param target_id The ID of the specific module within the target_type family.
         *
         * @return A non-negative integer representing the index of the module in the array of managed modules if the
         * addressed module is found. A '-1' value if the target module was not found. Additionally, sends an error
         * message to the PC if it fails to find the target module.
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
            SendData(
                static_cast<uint8_t>(kKernelStatusCodes::kTargetModuleNotFound),
                axmc_communication_assets::kPrototypes::kTwoUint8s,
                errors
            );
            return -1;
        }

        /**
         * @brief Loops over all managed modules and for each attempts to resolve and execute a command.
         *
         * This method first determines whether there is a command to run and, if there is, calls the API method that
         * instructs the module to run the logic that matches the active command code. If the module does not recognize
         * the active command, instructs it to send an error message to the PC.
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

                // If RunActiveCommand is implemented properly, it will return 'true' if it matches the active command
                // code to the method to execute and 'false' otherwise. If the method returns 'false', the Kernel calls
                // an API method to send a predetermined error message to the PC.
                if (!_modules[i]->RunActiveCommand()) _modules[i]->SendCommandActivationError();
            }
        }
};

#endif  //AXMC_KERNEL_H
