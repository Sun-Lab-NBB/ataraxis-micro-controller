/**
 * @file
 * @brief The header file for the Kernel class, which is used to manage the Microcontroller runtime.
 *
 * @subsection kern_description Description:
 *
 * For most other classes of this library to work as expected, the Microcontroller should instantiate and use a Kernel
 * instance to manage its runtime. This class manages communication, resolves success and error codes and schedules
 * commands to be executed. Due to the static API exposed by the (base) Module class, Kernel will natively integrate
 * with any Module-derived class logic. This allows seamlessly integrated custom modules with already existing
 * Ataraxis control structure, improving overall codebase flexibility.
 *
 * @note A single instance of this class should be created in the main.cpp file. It should be provided with an instance
 * of the Communication class and an array of pointers to Module-derived classes during instantiation.
 *
 * The Kernel class contains two major runtime-flow functions that have to be called as part of the setup() and
 * loop() methods of the main.cpp or main.ino file:
 * - Setup(): Initializes the hardware (e.g.: pin modes) for all managed modules. This command only needs to be
 * called once, during setup() method runtime.
 * - RuntimeCycle(): Aggregates all other steps necessary to receive and process PC data and execute Kernel and Module
 * commands. This command needs to be called repeatedly, during loop() method runtime.
 *
 * Additionally, the Kernel class supports PC-addressable command execution. Specifically, the Kernel can be used to
 * identify the controller and fully reset all hardware and software parameters of the controller. See the class
 * enumerations for specific command codes.
 *
 * @subsection kern_developer_notes Developer Notes:
 * This class functions similar to major OS kernels, although it is considerably limited in scope. Specifically, it
 * manages all compatible Modules and handles communication with other Ataraxis systems. This class is very important
 * for the correct functioning of the library and, therefore, it should not be modified, if possible. Any modifications
 * to this class may require modifications to some or all other base (Core) classes of this library, as well as any
 * custom classes derived from the Core classes.
 *
 * @subsection mod_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - elapsedMillis.h for millisecond and microsecond timers.
 * - communication.h for Communication class, which is used to bidirectionally communicate with other Ataraxis systems.
 * - module.h for the shared Module class API access (allows interfacing with custom modules).
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 */

#ifndef AXMC_KERNEL_H
#define AXMC_KERNEL_H

// Dependencies
#include <Arduino.h>
#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "communication.h"
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Provides the central runtime loop and methods that control data reception, runtime flow (task scheduling),
 * setup and shutdown of the microcontroller's logic.
 *
 * @attention This class functions as the backbone of the Ataraxis microcontroller codebase by providing runtime flow
 * control functionality. Any modification to this class should be carried out with extreme caution and, ideally,
 * completely avoided by anyone other than the kernel developers of the project.
 *
 * @note This class should be provided with an array of pre-initialized Module-derived classes upon instantiation.
 * The class will then manage the runtime of these modules via internal methods.
 *
 * Example instantiation (as would found in main.cpp file):
 * @code
 * shared_assets::DynamicRuntimeParameters DynamicRuntimeParameters; // Dynamic runtime parameters structure first
 * Communication axmc_communication(Serial);  // Communication class second
 * IOCommunication<1, 2> io_instance(1, 1, axmc_communication, DynamicRuntimeParameters);  // Example custom module
 * Module* modules[] = {&io_instance};  // Packages module(s) into an array to be provided to the Kernel class
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
         * @struct kKernelStatusCodes
         * @brief Specifies the byte-codes for errors and states that can be encountered during Kernel class method
         * execution.
         *
         * The Kernel class uses these byte-codes to communicate the exact status of various Kernel runtimes between
         * class methods and to send messages to the PC.
         *
         * @note Due to the overall message code hierarchy, this structure can use any byte-range values for status
         * codes. Kernel status codes always take precedence over all other status codes for messages sent by the
         * Kernel class (and, therefore, they will never directly clash with Communication or TransportLayer codes).
         */
        enum class kKernelStatusCodes : uint8_t
        {
            kStandBy                = 0,   ///< Currently not used. Statically reserves 0 to NOT be a valid code.
            kSetupComplete          = 1,   ///< Setup() method runtime succeeded.
            kModuleSetupError       = 2,   ///< Setup() method runtime failed due to a module setup error.
            kDataReceptionError     = 3,   ///< ReceiveData() method failed due to a data reception error.
            kDataSendingError       = 4,   ///< SendData() method failed due to a data sending error.
            kStateSendingError      = 5,   ///< SendData() method failed due to a data sending error.
            kServiceSendingError    = 6,   ///< Error sending a service message to the connected system.
            kInvalidMessageProtocol = 7,   ///< Received message uses an unsupported (unknown) protocol.
            kKernelParametersSet    = 8,   ///< Received and applied the parameters addressed to the Kernel class.
            kModuleParametersSet    = 9,   ///< Received and applied the parameters addressed to a managed Module class.
            kModuleParametersError  = 10,  ///< Unable to apply the received Module parameters.
            kCommandNotRecognized   = 11,  ///< The Kernel has received an unknown command.
            kTargetModuleNotFound   = 12   ///< No module with the requested type and id combination is found.
        };

        /**
         * @struct kKernelCommands
         * @brief Specifies the byte-codes for commands that can be executed during runtime.
         *
         * The Kernel class uses these byte-codes to communicate what command was executed when it sends data messages
         * to the connected system. Additionally, some of the listed commands are addressable by sending Kernel a
         * Command message that uses the appropriate command code.
         */
        enum class kKernelCommands : uint8_t
        {
            kStandby            = 0,  ///< Standby code used during class initialization.
            kReceiveData        = 1,  ///< Receive data command. Not externally addressable.
            kResetController    = 2,  ///< Resets the software and hardware state of all modules.
            kIdentifyController = 3,  ///< Transmits the ID of the controller back to caller.
        };

        /// Tracks the currently active Kernel command. This is used to send data and error messages to the connected
        /// system.
        uint8_t kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);

        /**
         * @brief Creates a new Kernel class instance.
         *
         * @param controller_id The unique identifier code of the controller managed by this Kernel class instance. This
         * ID has to be unique for all microcontrollers that are used at the same time to allow the managing PC to
         * identify different controllers.
         * @param communication A reference to the Communication class instance that will be used to bidirectionally
         * communicate with the PC that manages the microcontrollers. Usually, a single Communication class instance is
         * shared by all classes of the AXMC codebase during runtime.
         * @param dynamic_parameters A reference to the DynamicRuntimeParameters structure that stores
         * PC-addressable global runtime parameters that broadly alter the behavior of all modules used by the
         * controller. This structure is addressable through the Kernel class (by sending Parameters message addressed
         * to the Kernel).
         * @param module_array The array of pointers to custom Module classes (the children inheriting from the base
         * Module class). The Kernel instance will manage the runtime of these modules.
         */
        template <const size_t kModuleNumber>
        Kernel(
            const uint8_t controller_id,
            Communication& communication,
            shared_assets::DynamicRuntimeParameters& dynamic_parameters,
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
         * @brief Sets up the hardware and software structures for the Kernel class and all managed Modules.
         *
         * This method is an API wrapper around the setup sequence of each module managed by the Kernel class and the
         * Kernel class itself. If this method is not called before calling RuntimeCycle() method, the controller will
         * not function as expected and the controller LED will be used to show a blinking error signal. Similarly, if
         * this method is called, but fails, the controller will not execute any command and will display a blinking
         * LED signal to indicate SetUp error.
         *
         * @warning This method is used both for initial setup and to reset the controller. As such, it has to be
         * called from main.ccp setup() function and will also be called if the Kernel receives the Reset command.
         *
         * @note This method sends success and error messages to the PC. It assumes it is always called after the
         * Communication class has been initialized and Serial.begin() has been called.
         *
         * @attention This is the only Kernel method that inactivates the built-in LED of the controller board. Seeing
         * the LED ON (HIGH) after this method's runtime means the controller experienced an error during its runtime
         * when it tried sending data to the connected system. Seeing the LED blinking with ~2-second periodicity
         * indicates one of the managed modules failed its setup sequence. If no error message reached the PC and the
         * LED is ON or blinking, it is highly advised to debug the communication setup, as this indicates that the
         * communication channel is not stable.
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
                        communication_assets::kPrototypes::kTwoUnsignedBytes,
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
         * @brief Carries out one microcontroller runtime cycle.
         *
         * Specifically, first receives and processes all messages queued up in the serial reception buffer. All
         * messages other than Module-targeted commands are processed and handled immediately. For example, Kernel
         * commands are executed as soon as they are received. Module-addressed commands are queued for execution and
         * are executed after all available data is received and parsed.
         *
         * Once all data is received, the method sequentially loops over all modules and, for each, resolves and runs a
         * command, if possible. Depending on configuration (blocking vs. non-blocking), this can at most complete one
         * command per cycle and, at least, one command stage per cycle.
         *
         * @note This method has to be repeatedly called during the main.cpp loop() method runtime. It aggregates all
         * necessary steps to support the microcontroller runtime.
         *
         * @attention This method has to be called after calling the Setup() method. If it is called before the setup
         * was carried out, instead of running as expected, the microcontroller will cycle LED HIGH/LOW signal to
         * indicate setup error.
         */
        void RuntimeCycle()
        {
            // Tracks whether Kernel hardware setup was completed. This only needs to run once, since if the Kernel ever
            // fails setup, it bricks the controller until the firmware is reset.
            static bool once = true;

            // If the Kernel was not set up, ensures the LED hardware is set to deliver the visual error signal.
            if (!_setup_complete && once)
            {
                SetupKernel();
                once = false;  // Sets the flag to indicate that the setup was completed.
            }

            // If the Kernel was not set up, instead of the usual runtime cycle, this method blinks the LED to indicate
            // setup error.
            if (!_setup_complete)
            {
                digitalWriteFast(LED_BUILTIN, HIGH);  // Turns the LED on
                delay(2000);                          // Delays for 2 seconds
                digitalWriteFast(LED_BUILTIN, LOW);   // Turns the LED off
                delay(2000);                          // Delays for 2 seconds
                return;                               // Ends early. A firmware reset is needed to get out of this loop.
            }

            kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);  // Sets active command code

            // Continuously parses the data received from the PC until all data is processed.
            while (true)
            {
                const uint8_t protocol = ReceiveData();  // Attempts to receive the data
                bool break_loop = false;  // A flag used to break the while loop once all available data is received.
                int16_t target_module;    // Stores the index of the module targeted by Module-addressed command.
                uint8_t return_code;      // Stores the return code of the received message.

                // Uses the message protocol of the returned message to execute appropriate logic to handle the message.
                switch (static_cast<communication_assets::kProtocols>(protocol))
                {
                    // Returned protocol 0 indicates that the data was not received. This may be due to no data to
                    // receive or due to a reception pipeline error. In either case, this ends the reception loop.
                    case communication_assets::kProtocols::kUndefined: break_loop = true; break;

                    case communication_assets::kProtocols::kKernelParameters:

                        // If the message contains a non-zero return code, sends it back to the PC to acknowledge
                        // message reception
                        return_code = _communication.kernel_parameters.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // Since Kernel parameters are fully parsed during the message reception, replaces the content
                        // of the _dynamic_parameters structure with the newly received data.
                        _dynamic_parameters = _communication.kernel_parameters.dynamic_parameters;
                        SendData(static_cast<uint8_t>(kKernelStatusCodes::kKernelParametersSet));
                        break;

                    case communication_assets::kProtocols::kModuleParameters:
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
                                communication_assets::kPrototypes::kTwoUnsignedBytes,
                                error_object
                            );
                        }

                        // If the parameters were set correctly, notifies the PC.
                        SendData(static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersSet));
                        break;

                    case communication_assets::kProtocols::kKernelCommand:
                        return_code = _communication.kernel_command.return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // This method resolves and executes the command logic. It automatically extracts the command
                        // code from the received message stored in the Communication class attribute.
                        RunKernelCommand();
                        break;

                    case communication_assets::kProtocols::kDequeueModuleCommand:
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

                    case communication_assets::kProtocols::kOneOffModuleCommand:
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

                    case communication_assets::kProtocols::KRepeatedModuleCommand:
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
                            communication_assets::kPrototypes::kOneUnsignedByte,
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
        /// An array that stores managed module classes. These are the classes derived from the base Module class that
        /// encapsulate and expose the API to interface with specific hardware systems managed by the Microcontroller.
        Module** _modules;

        /// Stores the size of the _modules array. Since each module class is likely to be different in terms of its
        /// memory size, the individual classes are accessed using a pointer-reference system. For this system to work
        /// as expected, the overall size of the array needs to be stored separately, which is achieved by this
        /// variable
        const size_t _module_count;

        /// Stores the unique, user-defined ID of the managed microcontroller. This ID will be sent to the connected
        /// system when it sends the Identification request. In turn, this allows identifying specific controllers
        /// through the serial connection interface, which is especially helpful during the initial setup.
        const uint8_t _controller_id;

        /// A reference to the shared instance of the Communication class. This class is used to receive data from the
        /// connected Ataraxis systems and to send feedback data to these systems.
        Communication& _communication;

        /// A reference to the shared instance of the ControllerRuntimeParameters structure. This structure stores
        /// dynamically addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        shared_assets::DynamicRuntimeParameters& _dynamic_parameters;

        /// This flag tracks whether the Setup() method has been called before calling the RuntimeCycle() method. This
        /// is used to ensure the kernel is properly configured before running commands.
        bool _setup_complete = false;

        /**
         * @breif Attempts to receive (parse) a message from the data contained in the circular reception buffer of the
         * Communication class.
         *
         * The RuntimeCycle method repeatedly calls this method until it returns an undefined protocol code (0).
         *
         * @attention This method automatically attempts to send error messages if it runs into errors. As a side
         * effect of these attempts, it may turn the built-in LED to visually communicate encountered runtime errors.
         * Do not use the LED-connected pin in your code when using this class.
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
                static_cast<uint8_t>(shared_assets::kCommunicationCodes::kNoBytesToReceive))
            {
                // For legitimately failed runtimes, sends an error message to the PC.
                _communication.SendCommunicationErrorMessage(
                    static_cast<uint8_t>(kKernelCommands::kReceiveData),
                    static_cast<uint8_t>(kKernelStatusCodes::kDataReceptionError)
                );
            }

            // Regardless of the source of communication failure, uses 'kUndefined' protocol (0) code return to indicate
            // that no valid data to process was received.
            return static_cast<uint8_t>(communication_assets::kProtocols::kUndefined);
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
        void
        SendData(const uint8_t event_code, const communication_assets::kPrototypes prototype, const ObjectType& object)
        {
            // Packages and sends the data message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendDataMessage(kernel_command, event_code, prototype, object)) return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kDataSendingError)
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
                static_cast<uint8_t>(kKernelStatusCodes::kStateSendingError)
            );
        }

        /// Sends the ID code of the managed controller to the PC. This is used to identify specific controllers
        /// connected to different PC USB ports. If data sending fails, attempts sending a communication error and
        /// activates the LED indicator.
        void SendControllerID() const
        {
            // Packages and sends the service message to the PC. If the message was sent, ends the runtime
            if (_communication.SendServiceMessage(
                    static_cast<uint8_t>(communication_assets::kProtocols::kIdentification),
                    _controller_id
                ))
                return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kServiceSendingError)
            );
        }

        /// Sends the input reception_code to the PC. This is used to acknowledge the reception of PC-sent Command and
        /// Parameter messages. If data sending fails, attempts sending a communication error and activates the
        /// LED indicator.
        void SendReceptionCode(const uint8_t reception_code) const
        {
            // Packages and sends the service message to the PC. If the message was sent, ends the runtime
            if (_communication.SendServiceMessage(
                    static_cast<uint8_t>(communication_assets::kProtocols::kReceptionCode),
                    reception_code
                ))
                return;

            // Otherwise, attempts sending a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kServiceSendingError)
            );
        }

        /**
         * @brief Sets up the hardware and software parameters directly controlled by the Kernel class.
         *
         * Specifically, this configures and inactivates the built-in LED (by manipulating the board's LED-connected
         * pin) and resets the shared DynamicRuntimeParameters structure fields.
         */
        void SetupKernel() const
        {
            // Configures and deactivates the built-in LED. Currently, this is the only hardware system directly
            // managed by the Kernel.
            pinMode(LED_BUILTIN, OUTPUT);
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

                default:
                    // If the command code was not matched with any valid code, sends an error message.
                    SendData(static_cast<uint8_t>(kKernelStatusCodes::kCommandNotRecognized));
            }
        }

        /**
         * @brief Loops through the managed modules and attempts to find the module addressed by the input type and id.
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
                communication_assets::kPrototypes::kTwoUnsignedBytes,
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
