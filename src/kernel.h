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
 * Additionally, the Kernel class supports PC-addressable command execution. Specifically, the Kernel can be sued to
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
 * @brief Provides the central runtime loop and methods that control data reception, runtime flow, setup and shutdown of
 * the microcontroller's logic.
 *
 * @attention This class functions as the backbone of the Ataraxis microcontroller codebase by providing runtime flow
 * control functionality. Any modification to this class should be carried out with extreme caution and, ideally,
 * completely avoided by anyone other than the kernel developers of the project.
 *
 * @note This class requires to be provided with an array of pre-initialized Module-derived classes upon instantiation.
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
            kStandby               = 0,   ///< Standby code used during class initialization.
            kSetupComplete         = 1,   ///< Setup() method runtime succeeded.
            kModuleSetupError      = 2,   ///< Setup() method runtime failed due to a module setup error.
            kNoDataToReceive       = 3,   ///< ReceiveData() method succeeded without receiving any data.
            kDataReceptionError    = 4,   ///< ReceiveData() method failed due to a data reception error.
            kDataSendingError      = 5,   ///< SendData() method failed due to a data sending error.
            kInvalidDataProtocol   = 6,   ///< Received message uses an unsupported (unknown) protocol.
            kKernelParametersSet   = 7,   ///< Received and applied the parameters addressed to the Kernel class.
            kKernelParametersError = 8,   ///< Unable to apply the received Kernel parameters.
            kModuleParametersSet   = 9,   ///< Received and applied the parameters addressed to a managed Module class.
            kModuleParametersError = 10,  ///< Unable to apply the received Module parameters.
            kParametersTargetNotFound = 11,  ///< The addressee of the parameters' message could not be found.
            kControllerReset          = 12,  ///< The Kernel has reset the controller's software and hardware states.
            kKernelCommandUnknown     = 13,  ///< The Kernel has received an unknown command.
            kModuleCommandQueued      = 14,  ///< The received command was queued to be executed by the Module.
            kCommandTargetNotFound    = 15,  ///< The addressee of the command message could not be found.
            kServiceSendingError      = 16,  ///< Error sending a service message to the connected system.
            kControllerIDSent         = 17,  ///< The requested controller ID was sent ot to the connected system.
            kModuleCommandError       = 18,  ///< Error executing an active module command.
            kModuleCommandsCompleted  = 19,  ///< All active module commands have been executed.
            kModuleAssetResetError    = 20,  ///< Resetting custom assets of a module failed.
            kModuleCommandsReset      = 21,  ///< Module command structure has been reset. Queued commands cleared.
            kStateSendingError        = 22,  ///< SendState() method failed due to a data sending error.
            kResetModuleQueueTargetNotFound =
                23  ///< The addressee of the Module command queue reset command is not found
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
            kStandby         = 0,  ///< Standby code used during class initialization. Not externally addressable.
            kSetup           = 1,  ///< Module setup command. Not externally addressable.
            kReceiveData     = 2,  ///< Receive data command. Not externally addressable.
            kResetController = 3,  ///< Resets the software and hardware state of all modules. Externally addressable.
            kIdentifyController = 4,  ///< Transmits the ID of the controller back to caller. Externally addressable.
            kRunModuleCommands  = 5,  ///< Executes active module commands. Not externally addressable.
        };

        /// Communicates the most recent status of the Kernel. Primarily, this variable is used during class method
        /// testing, but it can also be co-opted by any class or method that requires knowing the latest status of the
        /// class runtime.
        uint8_t kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kStandby);

        /// Tracks the currently active Kernel command. This is used to send data and error messages to the connected
        /// system. Overall, this provides a unified message system shared by Kernel and Module-derived classes.
        uint8_t kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);

        /**
         * @brief Creates a new Kernel class instance.
         *
         * @param controller_id The unique identifier for the controller managed by this Kernel class instance. This
         * ID has to be unique for all microcontrollers that are used at the same time and managed by the same PC
         * system.
         * @param communication A reference to the Communication class instance that will be used to send module runtime
         * data to the connected system. Usually, a single Communication class instance is shared by all classes of the
         * AXMC codebase during runtime.
         * @param dynamic_parameters A reference to the ControllerRuntimeParameters structure that stores
         * dynamically addressable runtime parameters that broadly alter the behavior of all modules used by the
         * controller. This structure is addressable through the Kernel class (by sending Parameters message addressed
         * to the Kernel).
         * @param module_array The array of pointers to custom module classes (the children inheriting from the base
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
         * @brief Sets up the hardware and software structures for the Kernel class and all managed modules.
         *
         * This method should only be triggered once, during the runtime of the main.cpp setup() method. Overall,
         * this method is an API wrapper around the setup sequence of each module managed by the Kernel class and the
         * Kernel class itself. If this method is not called before calling RuntimeCycle() method, the controller will
         * not function as expected and the controller LED will be used to show a blinking error signal.
         *
         * @note This method sends a success or error message to the PC. Therefore, it assumes it is always called after
         * the Communication class has been initialized (its begin() method was called).
         *
         * @attention This is the only Kernel method that inactivates the built-in LED of the controller board. Seeing
         * the LED ON (HIGH) after this method's runtime means the controller experienced an error during its runtime
         * when it tried sending data to the connected system. If no error message reached the connected system, it is
         * highly advised to debug the communication setup, as this indicates that the communication channel is not
         * stable.
         */
        void Setup()
        {
            kernel_command = static_cast<uint8_t>(kKernelCommands::kSetup);  // Sets active command code

            // Sets up the runtime parameters of the Kernel class.
            ResetDynamicParameters();

            // Loops over each module and calls its SetupModule() virtual method. Note, expects that setup methods
            // generally cannot fail, but supports non-success return codes.
            for (size_t i = 0; i < _module_count; i++)
            {
                if (!_modules[i]->SetupModule())
                {
                    // If the setup fails, sends an error message that includes the ID information about the failed
                    // module alongside its runtime status.
                    const uint8_t error_object[3] = {
                        _modules[i]->GetModuleType(),
                        _modules[i]->GetModuleID(),
                        _modules[i]->module_status
                    };

                    // Sets the status and sends the error message to the PC
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupError);
                    SendData(kernel_status, communication_assets::kPrototypes::kThreeUnsignedBytes, error_object);
                    return;  // Returns without finishing the setup process.
                }

                // Also attempts to reset the custom assets of each module. Similar to the setup code above, this
                // generally should not lead to failures, but the system ios designed to handle errors if necessary.
                if (_modules[i]->ResetCustomAssets())
                {
                    const uint8_t error_object[3] = {
                        _modules[i]->GetModuleType(),
                        _modules[i]->GetModuleID(),
                        _modules[i]->module_status
                    };

                    // Sets the status and sends the error message to the PC
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleAssetResetError);
                    SendData(kernel_status, communication_assets::kPrototypes::kThreeUnsignedBytes, error_object);
                    return;  // Returns without finishing the setup process.
                }

                // Also resets the execution parameters of each module. This step cannot fail.
                _modules[i]->ResetExecutionParameters();
            }

            // Sets up the hardware managed by the Kernel. This is done last to, if necessary, override any
            // module-derived modifications of the hardware reserved by the Kernel.
            SetupKernel();

            // If the method reaches this point, all modules have been successfully set up. Sends the confirmation
            // message to the PC and sets the kernel_status to reflect this successful setup.
            kernel_status   = static_cast<uint8_t>(kKernelStatusCodes::kSetupComplete);
            _setup_complete = true;  // Sets the setup tracker to allow running commands.

            // Sends the state confirmation message (setup complete) to the PC.
            SendState(kernel_status);
        }

        /**
         * @brief Carries out one microcontroller runtime cycle.
         *
         * Specifically, first receives and processes all data queued up in the serial reception buffer. If the data
         * is a command addressed to the Kernel, that command is executed as part of the data reception runtime. If the
         * data is a command addressed to a module, the command is queued to be executed during module command runtime.
         *
         * Once all queued data is received, sequentially loops over all modules and executes the appropriate stage of
         * an already active or a newly activated command.
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
            static bool once = true;  // Tracks whether Kernel hardware setup was completed.

            // If the Kernel was not set up, ensures the LED hardware is set to deliver the visual error signal.
            if (!_setup_complete && once)
            {
                // Ensures the LED is set up to display signals.
                SetupKernel();
            }

            // If the Kernel was not set up, instead of the usual runtime cycle, this method blinks the LED to indicate
            // setup error.
            if (!_setup_complete)
            {
                digitalWriteFast(LED_BUILTIN, HIGH);  // Turns the LED on
                delay(2000);                          // Delays for 2 seconds
                digitalWriteFast(LED_BUILTIN, LOW);   // Turns the LED off
                delay(2000);                          // Delays for 2 seconds
            }

            kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);  // Sets active command code

            // Continuously parses the data received from the PC until all data is processed.
            while (true)
            {
                // Attempts to receive the data
                const uint8_t protocol = ReceiveData();

                bool break_loop = false;
                constexpr auto reception_protocol =
                    static_cast<uint8_t>(communication_assets::kProtocols::kReceptionCode);

                switch (static_cast<communication_assets::kProtocols>(protocol))
                {
                    // Returned protocol 0 indicates that the data was not received. This may be due to no data to
                    // receive or due to a reception pipeline error. In either case, this ends the reception loop.
                    case communication_assets::kProtocols::kUndefined: break_loop = true; break;

                    // Kernel-adressed parameters
                    case communication_assets::kProtocols::kKernelParameters:
                        // If the PC requested the acknowledgement code, notifies the PC that the message was received
                        // by sending back the included return code.
                        if (const uint8_t return_code = _communication.kernel_parameters.return_code)
                        {
                            SendServiceMessage(reception_protocol, return_code);
                        }

                        // Kernel-adressed parameters are written to the DynamicRuntimeParameters structure shared by
                        // all classes.
                        SetDynamicParameters();
                        break;

                    // Module-adressed parameters
                    case communication_assets::kProtocols::kModuleParameters:
                        if (const uint8_t return_code = _communication.module_parameter.return_code)
                        {
                            SendServiceMessage(reception_protocol, return_code);
                        }

                        // Triggers a method that loops over all modules and attempts to find the addressee and
                        // call its parameter extraction method.
                        SetModuleParameters();
                        break;

                    case communication_assets::kProtocols::kKernelCommand:
                        if (const uint8_t return_code = _communication.kernel_command.return_code)
                        {
                            SendServiceMessage(reception_protocol, return_code);
                        }

                        // Executes the requested Kernel command.
                        RunKernelCommand();
                        break;

                    case communication_assets::kProtocols::kDequeueModuleCommand:
                        if (const uint8_t return_code = _communication.module_dequeue.return_code)
                        {
                            SendServiceMessage(reception_protocol, return_code);
                        }

                        // Attempts to clear the requested modules' command queue
                        ClearModuleCommandQueue();
                        break;

                    case communication_assets::kProtocols::kOneOffModuleCommand:
                        if (const uint8_t return_code = _communication.one_off_module_command.return_code)
                        {
                            SendServiceMessage(reception_protocol, return_code);
                        }

                        QueueModuleCommand(
                            _communication.one_off_module_command.module_type,
                            _communication.one_off_module_command.module_id,
                            _communication.one_off_module_command.command,
                            _communication.one_off_module_command.noblock
                            // Disables command cycling (repetition)
                        );
                        break;

                    case communication_assets::kProtocols::KRepeatedModuleCommand:
                        if (const uint8_t return_code = _communication.repeated_module_command.return_code)
                        {
                            SendServiceMessage(reception_protocol, return_code);
                        }
                        QueueModuleCommand(
                            _communication.repeated_module_command.module_type,
                            _communication.repeated_module_command.module_id,
                            _communication.repeated_module_command.command,
                            _communication.repeated_module_command.noblock,
                            true,  // Enables command cycling (repetition).
                            _communication.repeated_module_command.cycle_delay
                        );
                        break;

                    default:
                        // If the protocol is not one of the expected protocols, sends an error message.
                        // Includes the invalid protocol value
                        kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kInvalidDataProtocol);
                        SendData(
                            kernel_status,
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
        /// An array that stores module classes. These are the classes derived from the base Module class that
        /// encapsulates and exposes the API to interface with specific hardware systems managed by the Microcontroller.
        Module** _modules;

        /// Stores the size of the _modules array. Since each module class is likely to be different in terms of its
        /// memory size, the individual classes are accessed using a pointer-reference system. For this system to work
        /// as expected, the overall size of the array needs to be stored separately, which is achieved by this
        /// variable
        const size_t _module_count;

        ///Stores the unique, user-defined ID of the managed microcontroller. This ID will be sent to the connected
        /// system when it sends the identification request. In turn, this allows identifying specific controllers
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
         * @brief Packages and sends the provided event_code and data object to the PC via the Communication class
         * instance.
         *
         * This method simplifies sending data through the Communication class by automatically resolving most of the
         * payload metadata. This method guarantees that the formed payload follows the correct format and contains
         * the necessary data.
         *
         * @note It is highly recommended to use this utility method for sending data to the connected system instead of
         * using the Communication class directly. If the data you are sending does not need a data object, use
         * SendState() for a more optimal message format.
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
         * @param object Additional data object to be sent along with the message. Currently, all data messages
         * have to contain a data object, but you can use a sensible placeholder for calls that do not have a valid
         * object to include. If your message does not require a data object, use SendState() method instead.
         */
        template <typename ObjectType>
        void
        SendData(const uint8_t event_code, const communication_assets::kPrototypes prototype, const ObjectType& object)
        {
            // Packages and sends the data to the connected system via the Communication class and if the message was
            // sent, ends the runtime
            if (_communication.SendDataMessage(0, 0, kernel_command, event_code, prototype, object)) return;

            // If the message was not sent, attempts sending an error message. For this, combines the latest status of
            // the Communication class (likely an error code) and the TransportLayer status code into a 2-byte array.
            // This will be the data object transmitted with the error message.
            const uint8_t errors[2] = {_communication.communication_status, _communication.GetTransportLayerStatus()};

            // Attempts sending the error message. Uses the unique DataSendingError event code and the object
            // constructed above. Does not evaluate the status of sending the error message to avoid recursions.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kDataSendingError);
            _communication.SendDataMessage(
                0,
                0,
                kernel_command,
                kernel_status,
                communication_assets::kPrototypes::kOneUnsignedByte,
                errors
            );

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Packages and sends the provided event_code to the PC via the Communication class instance.
         *
         * This method is very similar to SendData(), but is optimized to transmit messages that do not use custom
         * data objects.
         *
         * This method simplifies sending data through the Communication class by automatically resolving most of the
         * payload metadata. This method guarantees that the formed payload follows the correct format and contains
         * the necessary data.
         *
         * @note It is highly recommended to use this utility method for sending data to the connected system instead of
         * using the Communication class directly.
         *
         * @warning If sending the data fails for any reason, this method automatically emits an error message. Since
         * that error message may itself fail to be sent, the method also statically activates the built-in LED of the
         * board to visually communicate encountered runtime error. Do not use the LED-connected pin or LED when using
         * this method to avoid interference!
         *
         * @param event_code The byte-code specifying the event that triggered the data message.
         */
        void SendState(const uint8_t event_code)
        {
            // Packages and sends the state message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendStateMessage(0, 0, kernel_command, event_code)) return;

            // If the message was not sent, attempts sending an error message. For this, combines the latest status of
            // the Communication class (likely an error code) and the TransportLayer status code into a 2-byte array.
            // This will be the data object transmitted with the error message.
            const uint8_t errors[2] = {_communication.communication_status, _communication.GetTransportLayerStatus()};

            // Attempts sending the error message. Uses the unique kStateSendingError event code and the object
            // constructed above. Does not evaluate the status of sending the error message to avoid recursions.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kStateSendingError);
            _communication.SendDataMessage(
                0,
                0,
                kernel_command,
                kernel_status,
                communication_assets::kPrototypes::kTwoUnsignedBytes,
                errors
            );

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Packages and sends the provided service_code to the connected Ataraxis system via the Communication
         * class instance, using the requested service protocol.
         *
         * This method simplifies sending data through the Communication class by automatically resolving most of the
         * payload metadata. This method guarantees that the formed payload follows the correct format and contains
         * the necessary data.
         *
         * @note It is highly recommended to use this utility method for sending service messages to the connected
         * system instead of using the Communication class directly.
         *
         * @warning If sending the service message fails for any reason, this method automatically emits an error
         * message. Since that error message may itself fail to be sent, the method also statically activates the
         * built-in LED of the board to visually communicate encountered runtime error. Do not use the LED-connected
         * pin or LED when using this method to avoid interference!
         *
         * @param protocol The byte-code specifying the protocol to be used when sending the service message. This
         * determines the meaning of the included service_code.
         * @param service_code The byte-code to be included with the message. The exact meaning and purpose for the code
         * depends on the chosen service protocol.
         */
        void SendServiceMessage(const uint8_t protocol, const uint8_t service_code)
        {
            // Packages and sends the service message to the connected system via the Communication class. If the
            // message was sent, ends the runtime
            if (_communication.SendServiceMessage(protocol, service_code)) return;

            // If the message was not sent attempts sending an error message. For this, combines the latest status of
            // the Communication class (likely an error code) and the TransportLayer status code into a 2-byte array.
            // This will be the data object transmitted with the error message.
            const uint8_t errors[2] = {_communication.communication_status, _communication.GetTransportLayerStatus()};

            // Attempts sending the error message. Uses the unique kServiceSendingError event code and the object
            // constructed above. Does not evaluate the status of sending the error message to avoid recursions.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kServiceSendingError);
            _communication.SendDataMessage(
                0,
                0,
                kernel_command,
                kernel_status,
                communication_assets::kPrototypes::kTwoUnsignedBytes,
                errors
            );

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Sets up the hardware directly controlled by the Kernel class.
         *
         * Currently, this method only configures the built-in LED pin of the microcontroller board and sets it to
         * LOW.
         */
        static void SetupKernel()
        {
            // Configures and deactivates the built-in LED. Currently, this is the only hardware system directly
            // managed by the Kernel.
            pinMode(LED_BUILTIN, OUTPUT);
            digitalWriteFast(LED_BUILTIN, LOW);
        }

        /**
         * @brief Resets the shared DynamicRuntimeParameters structure fields to use the default values.
         *
         * Currently, these parameters are only addressable through the Kernel class, but they are used by all modules
         * managed by the Kernel.
         */
        void ResetDynamicParameters() const
        {
            _dynamic_parameters.action_lock = true;
            _dynamic_parameters.ttl_lock    = true;
        }

        /**
         * @breif Attempts to receive (parse) a message from the data contained in the circular reception buffer of the
         * Communication class.
         *
         * During the RuntimeCycle() method's runtime, this method will be called repeatedly until it returns
         * an undefined protocol code (0).
         *
         * @attention This method automatically attempts to send error messages if it runs into errors. As a side
         * effect of these attempts, it may turn the built-in LED to visually communicate encountered runtime errors.
         * Do not use the LED-connected pin in your code when using this class.
         *
         * @returns The byte-code of the protocol used by the received message or 0 to indicate no message was
         * received.
         */
        uint8_t ReceiveData()
        {
            kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);  // Sets active command code

            // Attempts to receive a message sent by the PC.
            if (!_communication.ReceiveMessage())
            {
                // Data reception can fail due to one of the involved systems encountering an error or because there
                // was no data to receive. Determines which of the two cases occurred here and resolves it
                // appropriately.
                if (_communication.communication_status ==
                    static_cast<uint8_t>(shared_assets::kCommunicationCodes::kNoBytesToReceive))
                {
                    // This is a non-error clause, sometimes there is no data to receive as none was sent.
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kNoDataToReceive);
                }
                else
                {
                    // In other cases, there is a reception or transmission error that makes the data unparsable. In
                    // this case, sends and error message to the PC.
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kDataReceptionError);
                    // One or both of these codes may be error codes describing the specific issue.
                    const uint8_t errors[2] = {
                        _communication.communication_status,
                        _communication.GetTransportLayerStatus()
                    };
                    SendData(kernel_status, communication_assets::kPrototypes::kTwoUnsignedBytes, errors);
                }

                // In either case, returns the 'undefined' protocol code to indicate no message was received.
                return static_cast<uint8_t>(communication_assets::kProtocols::kUndefined);
            }

            // Otherwise, if the message was parsed and received, returns the protocol code of the received message.
            return _communication.protocol_code;
        }

        /**
         * @brief Overwrites the global DynamicRuntimeParameters structure with the new values received from the PC.
         */
        void SetDynamicParameters()
        {
            // Extracts the received parameter data into the DynamicRuntimeParameters structure shared by all
            // modules and Kernel class. Updates the structure by reference.
            _dynamic_parameters = _communication.kernel_parameters.dynamic_parameters;
            kernel_status       = static_cast<uint8_t>(kKernelStatusCodes::kKernelParametersSet);
        }

        /**
         * @brief Attempts to resolve the module targeted by the most recent ModuleParameter message and call its
         * parameter setting method.
         *
         * This method allows overwriting the parameters of specific modules by sending the appropriate message from
         * the PC.
         */
        void SetModuleParameters()
        {
            // Loops over the managed modules and attempts to find the addressed module
            for (size_t i = 0; i < _module_count; i++)
            {
                // If a Module class is found with matching type and id, calls its SetCustomParameters() method to
                // handle the received parameters' payload.
                if (_modules[i]->GetModuleType() == _communication.module_parameter.module_type &&
                    _modules[i]->GetModuleID() == _communication.module_parameter.module_id)
                {
                    // Calls the Module API method that processes the parameter object included with the message
                    if (!_modules[i]->SetCustomParameters())
                    {
                        // If parameter setting fails, sends an error message to the PC and aborts the runtime.
                        // Includes ID information about the payload addressee and extraction error code from the
                        // communication class.
                        const uint8_t errors[3] = {
                            _communication.module_parameter.module_type,
                            _communication.module_parameter.module_id,
                            _communication.communication_status
                        };
                        kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersError);
                        SendData(
                            kernel_status,
                            communication_assets::kPrototypes::kThreeUnsignedBytes,
                            errors
                        );  // Sends the error data to the PC
                        return;
                    }

                    // Otherwise, the parameters were parsed successfully
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersSet);
                    return;
                }
            }

            // If this point is reached, the parameter payload addressee was not found. Sends an error message
            // to the PC and reruns the loop.
            const uint8_t errors[2] = {
                _communication.module_parameter.module_type,
                _communication.module_parameter.module_id,
            };  // payload ID information
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kParametersTargetNotFound);
            SendData(kernel_status, communication_assets::kPrototypes::kTwoUnsignedBytes, errors);
        }

        /**
         * @brief Determines and executes the requested Kernel command received from the PC.
         */
        void RunKernelCommand()
        {
            // Resolves and executes the specific command code communicated by the message:
            switch (auto command = static_cast<kKernelCommands>(_communication.kernel_command.command))
            {
                case kKernelCommands::kResetController:
                    kernel_command = static_cast<uint8_t>(command);  // Adjusts executed command status
                    ResetControllerCommand();
                    return;

                case kKernelCommands::kIdentifyController:
                    kernel_command = static_cast<uint8_t>(command);  // Adjusts executed command status
                    IdentifyControllerCommand();
                    return;

                default:
                    // If the command code was not matched with any valid code, sends an error message.
                    const uint8_t errors[1] = {_communication.kernel_command.command};
                    kernel_status           = static_cast<uint8_t>(kKernelStatusCodes::kKernelCommandUnknown);
                    SendData(kernel_status, communication_assets::kPrototypes::kOneUnsignedByte, errors);
            }
        }

        /// Sends an identification service message that contains the unique ID of the controller to the PC.
        void IdentifyControllerCommand()
        {
            SendServiceMessage(static_cast<uint8_t>(communication_assets::kProtocols::kIdentification), _controller_id);
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kControllerIDSent);
        }

        /// Resets all software and hardware parameters for the Kernel and all managed modules.
        void ResetControllerCommand()
        {
            Setup();  // This essentially resets the controller.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kControllerReset);
            SendState(kernel_status);  // Notifies the connected system that the controller has been reset
        }

        /**
         * @brief Resolves the module targeted by the most recent Command message and queues the included command for
         * execution.
         *
         * Unlike Kernel commands, Module commands are not immediately executed once they are received. Instead, they
         * are queued to be executed whenever the next module command cycle occurs and the target module is done
         * executing any already active command.
         *
         * @param target_type The type (family) of the module targeted by the command.
         * @param target_id The ID of the specific module within the target_type family that will execute the command.
         * @param command_code The code specifying the command to be executed by the module.
         * @param noblock Indicates whether the command should be executed immediately or queued.
         * @param cycle Indicates whether the command should be executed repeatedly (cyclically) at a regular interval.
         * @param cycle_delay The delay in microseconds between command cycles for cyclic (recurrent) commands.
         */
        void QueueModuleCommand(
            const uint8_t target_type,
            const uint8_t target_id,
            const uint8_t command_code,
            const bool noblock,
            const bool cycle           = false,
            const uint32_t cycle_delay = 0
        )
        {
            // Loops over the managed modules and attempts to find the addressed module.
            for (size_t i = 0; i < _module_count; i++)
            {
                // If a Module class is found with matching type and id, queues the command to be executed by the
                // target module.
                if (_modules[i]->GetModuleType() == target_type && _modules[i]->GetModuleID() == target_id)
                {
                    // Directly accesses the necessary Module API method to queue the command. Note, the last two
                    // arguments are only
                    _modules[i]->QueueCommand(command_code, noblock, cycle, cycle_delay);

                    // Target module found, command queued
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandQueued);
                    return;
                }
            }

            // If this point is reached, the command payload addressee was not found. Sends an error message
            // to the PC and reruns the loop.
            const uint8_t errors[3] = {target_type, target_id, command_code};
            kernel_status           = static_cast<uint8_t>(kKernelStatusCodes::kCommandTargetNotFound);
            SendData(kernel_status, communication_assets::kPrototypes::kThreeUnsignedBytes, errors);
        }

        /// Clears the command queue for the targeted module. This is especially relevant for resetting recurrent
        /// commands without overwriting them with a non-recurrent command.
        void ClearModuleCommandQueue()
        {
            // Otherwise, loops over the managed modules and attempts to find the addressed module.
            for (size_t i = 0; i < _module_count; i++)
            {
                // If a Module class is found with matching type and id, queues the command to be executed by the
                // target module.
                if (_modules[i]->GetModuleType() == _communication.module_dequeue.module_type &&
                    _modules[i]->GetModuleID() == _communication.module_dequeue.module_id)
                {
                    _modules[i]->ResetCommandQueue();
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandsReset);
                    return;
                }
            }

            // If this point is reached, the reset command payload addressee was not found. Sends an error message
            // to the PC and reruns the loop.
            const uint8_t errors[2] = {
                _communication.module_dequeue.module_type,
                _communication.module_dequeue.module_id
            };
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kResetModuleQueueTargetNotFound);
            SendData(kernel_status, communication_assets::kPrototypes::kTwoUnsignedBytes, errors);
        }

        /**
         * @brief Loops over the managed modules and for each attempts to resolve and execute a command.
         *
         * This method sequentially executes the active commands for all managed modules, controlling the
         * connected hardware systems. It resolves re-running (recursing) and activating the queued commands
         * (replacing completed commands with newly queued commands) and also enables concurrent (non-blocking)
         * command execution.
         *
         * @attention This method automatically attempts to send error messages if it runs into errors. As a side
         * effect of these attempts, it may turn the built-in LED to visually communicate encountered runtime errors.
         * Do not use the LED-connected pin in your code when using this class.
         */
        void RunModuleCommands()
        {
            kernel_command =
                static_cast<uint8_t>(kKernelCommands::kRunModuleCommands);  // Adjusts executed command status

            // Loops over all managed modules
            for (size_t i = 0; i < _module_count; i++)
            {
                // First, determines which command to run, if any. This relies on the following choice
                // hierarchy: finish already active commands > execute a newly queued command > repeat a cyclic command.
                // Active command setting does not fail due to errors, so this method simply needs to be called once for
                // each module.
                _modules[i]->ResolveActiveCommand();

                // Since RunActiveCommand is a virtual method intended to be implemented by the end-user
                // that subclasses the base Module class, it relies on the Kernel to handle runtime errors.
                if (!_modules[i]->RunActiveCommand())
                {
                    // Constructs and sends the appropriate error message to the connected system.
                    uint8_t errors[3] = {
                        _modules[i]->GetModuleType(),
                        _modules[i]->GetModuleID(),
                        _modules[i]->GetActiveCommand()
                    };
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandError);
                    SendData(kernel_status, communication_assets::kPrototypes::kThreeUnsignedBytes, errors);

                    // Does not terminate the runtime and continues with other modules. This is intentionally
                    // designed to allow recovering from non-fatal command execution errors without terminating the
                    // runtime.
                }
            }

            //Adjusts the status and terminates the method runtime.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandsCompleted);
        }
};

#endif  //AXMC_KERNEL_H
