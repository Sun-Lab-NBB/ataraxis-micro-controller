/**
 * @file
 * @brief The header file for the Kernel class, which is used to manage the Microcontroller runtime.
 *
 * @subsection kern_description Description:
 *
 * For most other classes of this library to work as expected, the Microcontroller should instantiate and use a Kernel
 * instance to manage its runtime. This class manages communication, resolves success and error codes and schedules
 * commands to be executed. Due to the static API exposed by the (base) Module class, Kernel will natively integrate
 * with any Module-derived class logic.
 *
 * @note A single instance of this class should be created in the main.cpp file. It should be provided with an instance
 * of the Communication class and an array of Module-derived classes during instantiation.
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
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 * - communication.h for Communication class, which is used to bidirectionally communicate with other Ataraxis systems.
 * - module.h for the shared Module API access.
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
 * @brief Provides the central runtime loop and all kernel-level methods that control data reception, execution runtime
 * flow, setup and shutdown.
 *
 * @attention This class functions as the backbone of the AMC codebase by providing runtime flow control functionality.
 * Without it, the AMC codebase would not function as expected. Any modification to this class should be carried out
 * with extreme caution and, ideally, completely avoided by anyone other than the kernel developers of the project.
 *
 * The class contains both major runtime flow methods and minor kernel-level commands that provide general functionality
 * not available through Module (the parent of every custom module class). Overall, this class is designed
 * to compliment Module methods, and together the two classes provide a robust set of features to realize
 * custom Controller-mediated behavior.
 *
 * @note This class requires to be provided with an array of pre-initialized Module-derived classes upon instantiation.
 * The class will then manage the runtime of these modules via internal methods.
 *
 * @tparam module_number The number of custom module classes derived from the main Module class) that will be
 * managed by the class instance.
 * @tparam controller_id The unique identifier for the controller managed by the Kernel class instance. This identifier
 * will be sent to the connected system when it requests the identification code of the controller.
 */
template <size_t module_number, uint8_t controller_id>
class Kernel
{
        static_assert(
            module_number > 0,
            "Module number must be greater than 0. At least one valid Module instance must be provided during Kernel "
            "class initialization."
        );

    public:
        /// Statically reserves '2' as type id of the class. No other Core or (base) Module-derived class should use
        /// this type id.
        static constexpr uint8_t kModuleType = 2;

        /// There should always be a single Kernel class shared by all other classes. Therefore, the ID for the
        /// class instance is always 0 (not used).
        static constexpr uint8_t kModuleId = 0;

        /**
         * @struct kKernelStatusCodes
         * @brief Specifies the byte-codes for errors and states that can be encountered during Kernel class method
         * execution.
         *
         * The Kernel class uses these byte-codes to communicate the exact status of various Kernel runtimes between
         * class methods and to send messages to the PC.
         *
         * @note Unlike most other status-code structures,
         */
        enum class kKernelStatusCodes : uint8_t
        {
            kStandby                  = 0,  ///< Standby code used during class initialization.
            kModuleSetupComplete      = 1,  ///< SetupModules() method runtime succeeded.
            kModuleSetupError         = 2,  ///< SetupModules() method runtime failed due to a module error.
            kNoDataToReceive          = 3,  ///< ReceiveData() method succeeded without receiving any data.
            kDataReceptionError       = 4,  ///< ReceiveData() method failed due to a data reception error.
            kDataSendingError         = 5,  ///< SendData() method failed due to a data sending error.
            kInvalidDataProtocol      = 6,  ///< Received message uses an unsupported (unknown) protocol.
            kKernelParametersSet      = 7,  ///< Received and applied the parameters adressed to the Kernel class.
            kKernelParametersError    = 8,  ///< Unable to apply the received Kernel parameters.
            kModuleParametersSet      = 9,  ///< Received and applied the parameters adressed to a managed Module class.
            kModuleParametersError    = 10,  ///< Unable to apply the received Module parameters.
            kParametersTargetNotFound = 11,  ///< The addressee of the parameters' message could not be found.
            kControllerReset          = 12,  ///< The Kernel has reset the controller's software and hardware states.
            kKernelCommandUnknown     = 13,  ///< The Kernel has received an unknown command.
            kModuleCommandQueued      = 14,  ///< The received command was queued to be executed by the Module.
            kCommandTargetNotFound    = 15,  ///< The addressee of the command message could not be found.
            kServiceSendingError      = 16,  ///< Error sending a service message to the connected system.
            kModuleResetError         = 17,  ///< Unable to reset the custom assets of a module.
            kControllerIDSent         = 18,  ///< The requested controller ID was sent ot to the connected system.
            kModuleCommandError       = 19,  ///< Error executing an active module command.
            kModuleCommandsCompleted  = 20,  ///< All active module commands have been executed.
        };

        /**
         * @struct kKernelCommands
         * @brief Specifies the byte-codes for commands that can be executed during runtime.
         *
         * The Kernel class uses these byte-codes to communicate what command was executed when it sends data messages
         * to the connected system. Usually, this is used for error messages.
         */
        enum class kKernelCommands : uint8_t
        {
            kStandby            = 0,  ///< Standby code used during class initialization. Not externally addressable.
            kSetupModules       = 1,  /// Module setup command. Not externally addressable.
            kReceiveData        = 2,  /// Receive data command. Not externally addressable.
            kResetController    = 3,  /// Resets the software and hardware state of all modules. Externally addressable.
            kIdentifyController = 4,  /// Transmits the ID of the controller back to caller. Externally addressable.
            kRunModuleCommands  = 5,  /// Executes active module commands. Not externally addressable.
        };

        /// Communicates the most recent status of the Kernel. Primarily, this variable is used during class method
        /// testing, but it can also be co-opted by any class or method that requires knowing the latest status of the
        /// class runtime.
        uint8_t kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kStandby);

        /// Communicates the active command.
        uint8_t kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);

        /**
         * @brief Creates a new Kernel class instance.
         *
         * @param communication A reference to the Communication class instance shared by all other runtime-active
         * class is the only class that can receive data from the PC.
         * @param dynamic_parameters A reference to the DynamicRuntimeParameters structure used to store addressable
         * controller runtime parameters.
         * @param module_array The array of custom module classes (the children inheriting from the base Module class).
         * The kernel will manage the runtime of these modules.
         */
        Kernel(
            Communication& communication,
            shared_assets::DynamicRuntimeParameters& dynamic_parameters,
            Module* (&module_array)[module_number]
        ) :
            _modules(module_array),
            _module_count(module_number),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters),
            kControllerID(controller_id)
        {}

        /**
         * @brief Packages and sends the provided event_code and data object to the connected Ataraxis system via
         * the Communication class instance.
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
         * @tparam ObjectType The type of the data object to be sent along with the message. This is inferred
         * automatically by the template constructor.
         * @param event_code The byte-code specifying the event that triggered the data message.
         * @param object Additional data object to be sent along with the message. Currently, all data messages
         * have to contain a data object, but you can use a sensible placeholder for calls that do not have a valid
         * object to include. By default, this is set to placeholder byte value 255, which is always parsed as
         * placeholder and will be ignored upon reception.
         * @param object_size The size of the transmitted object, in bytes. This is calculated automatically based on
         * the type of the object. Do not overwrite this argument.
         */
        template <typename ObjectType = uint8_t>
        void SendData(
            const uint8_t event_code,
            const ObjectType& object = communication_assets::kDataPlaceholder,
            const size_t object_size = sizeof(ObjectType)
        )
        {
            // Packages and sends the data to the connected system via the Communication class
            const bool success =
                _communication.SendDataMessage(kModuleType, kModuleId, kernel_command, event_code, object, object_size);

            // If the message was sent, ends the runtime
            if (success) return;

            // If the message was not sent attempts sending an error message. For this, combines the latest status of
            // the Communication class (likely an error code) and the TransportLayer status code into a 2-byte array.
            // This will be the data object transmitted with the error message.
            const uint8_t errors[2] = {_communication.communication_status, _communication.GetTransportLayerStatus()};

            // Attempts sending the error message. Uses the unique DataSendingError event code and the object
            // constructed above. Does not evaluate the status of sending the error message to avoid recursions.
            _communication.SendDataMessage(
                kModuleType,
                kModuleId,
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kDataSendingError),
                errors,
                sizeof(errors)
            );

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kDataSendingError);
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
            _communication.SendDataMessage(
                kModuleType,
                kModuleId,
                kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kServiceSendingError),
                errors,
                sizeof(errors)
            );

            // As a fallback in case the error message does not reach the connected system, sets the class status to
            // the error code and activates the built-in LED. The LED is used as a visual indicator for a potentially
            // unhandled runtime error. The Kernel class manages the indicator inactivation.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kServiceSendingError);
            digitalWriteFast(LED_BUILTIN, HIGH);
        }

        /**
         * @brief Triggers the SetupModule() method of each module inside the module array.
         *
         * This method should only be triggered once, during the runtime of the main script setup() loop. Consider this
         * an API wrapper around the setup sequence of each module managed by the Kernel class.
         *
         * @note Automatically sends a success or error message to the PC and, as such, assumes this method is always
         * called after the Communication class is properly initialized. This method also sets the kernel_status to
         * reflect the status of this method's runtime.
         *
         * @attention This is the only Kernel method that inactivates the built-in LED. Seeing the LED after this
         * method's runtime means the controller experienced an error during its runtime. If no error message reached
         * the connected system, it is highly advised to debug the communication setup.
         *
         * @return True if all modules have been successfully set up, false otherwise.
         */
        bool SetupModules()
        {
            kernel_command = static_cast<uint8_t>(kKernelCommands::kSetupModules);  // Sets active command code

            // Configures and deactivates the built-in LED.
            pinMode(LED_BUILTIN, OUTPUT);
            digitalWriteFast(LED_BUILTIN, LOW);

            // Loops over each module and calls its SetupModule() virtual method. Note, expects that setup methods
            // generally cannot fail, but supports non-success returns codes.
            for (size_t i = 0; i < _module_count; i++)
            {
                if (const bool result = _modules[i]->SetupModule(); !result)
                {
                    // If the setup fails, sends an error message the includes the ID information about the failed
                    // module alongside the status, presumably set to the specific error code.
                    const uint8_t error_object[3] = {
                        _modules[i]->GetModuleType(),
                        _modules[i]->GetModuleID(),
                        _modules[i]->module_status
                    };

                    // Sets the status and sends the error message to the PC
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupError);
                    SendData(static_cast<uint8_t>(kKernelCommands::kSetupModules), kernel_status, error_object);
                    kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                    return false;  // Returns without finishing the setup process.
                }
            }

            // If the method reaches this point, all modules have been successfully set up. Sends the confirmation
            // message to the PC and sets the kernel_status to reflect this successful setup.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupComplete);

            // This automatically uses placeholder value 255 for the object, so it will be discarded upon reception.
            SendData(static_cast<uint8_t>(kKernelCommands::kSetupModules), kernel_status);

            // Setup complete, returns with success value
            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
            return true;
        }

        /**
         * @breif Attempts to receive (parse) a message from the data contained in the circular buffer and process the
         * message payload.
         *
         * This is one of the two Kernel methods that have to be repeatedly called during loop() method runtime to
         * enable the proper functioning of the controller. This method contains all logic for receiving and processing
         * the incoming Kernel- and module-addressed commands and parameter updates.
         *
         * @attention This method automatically attempts to send error messages if it runs into errors. As a side
         * effect of these attempts, it may turn the built-in LED to visually communicate encountered runtime errors.
         * Do not use the LED-connected pin in your code when using this class.
         */
        void ReceiveData()
        {
            kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);  // Sets active command code

            // Attempts to receive a message sent by the PC. The messages received from the PC are stored in the
            // circular buffer of the controller. The call to this method attempts to parse the message from the
            // data available in the buffer. If only part of the message was received, the method will block for some
            // time and attempt to receive and parse the whole message.
            // If no data was received, breaks the loop. If the reception failed due to a runtime error, sends an error
            // message to the PC. If the reception failed due to having no data to receive, terminates the runtime with
            // the appropriate non-error status
            if (!_communication.ReceiveMessage())
            {
                // This is a non-error clause, sometimes there is no data to be received.
                if (_communication.communication_status ==
                    static_cast<uint8_t>(shared_assets::kCommunicationCodes::kCommunicationNoBytesToReceive))
                {
                    // Sets the status to indicate there is no data to receive and breaks the runtime.
                    kernel_status  = static_cast<uint8_t>(kKernelStatusCodes::kNoDataToReceive);
                    kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                    return;
                }

                // Any other status code of the Communication class is interpreted as an error code and results in
                // kernel sending the error message to the PC.
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kDataReceptionError);

                // Retrieves the communication error status
                const uint8_t errors[2] = {
                    _communication.communication_status,
                    _communication.GetTransportLayerStatus()
                };

                SendData(kernel_status, errors);  // Sends the error data to the PC
                kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                return;
            }

            // Processes the received payload, depending on the type (protocol) of the payload:
            // PARAMETERS
            if (const uint8_t protocol = _communication.protocol_code();
                protocol == static_cast<uint8_t>(communication_assets::kProtocols::kParameters))
            {
                const uint8_t target_type = _communication.parameter_header.module_type;
                const uint8_t target_id   = _communication.parameter_header.module_id;

                // If the sender requested the reception notification, sends the necessary service message to confirm
                // that the data was received as expected.
                if (const uint8_t return_code = _communication.parameter_header.return_code)
                {
                    SendServiceMessage(
                        static_cast<uint8_t>(communication_assets::kProtocols::kReceptionCode),
                        return_code
                    );
                }

                // If the parameters are adressed to the Kernel class, the parameters always have to be in the form of
                // the DynamicRuntimeParameters structure.
                if (target_type == kModuleType && target_id == kModuleId)
                {
                    // Extracts the received parameters into the DynamicRuntimeParameters structure shared by all
                    // modules and Kernel class.
                    if (!_communication.ExtractParameters(_dynamic_parameters))
                    {
                        // If the extraction fails, sends an error message to the PC and aborts the runtime
                        // Includes ID information about the payload addressee and error statuses from involved modules
                        const uint8_t errors[3] = {target_type, target_id, _communication.communication_status};
                        kernel_status           = static_cast<uint8_t>(kKernelStatusCodes::kKernelParametersError);
                        SendData(kernel_status, errors);  // Sends the error data to the PC
                        kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                        return;
                    }

                    // Sets the class runtime status and inactivates (completes) the command before aborting the
                    // runtime.
                    kernel_status  = static_cast<uint8_t>(kKernelStatusCodes::kKernelParametersSet);
                    kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                    return;
                }

                // Otherwise, if the parameters were not addressed to the Kernel class, loops over the managed modules
                // and attempts to find the adressed module
                for (size_t i = 0; i < _module_count; i++)
                {
                    // If a Module class is found with matching type and id, calls its SetCustomParameters() method to
                    // handle the received parameters' payload.
                    if (_modules[i]->GetModuleType() == target_type && _modules[i]->GetModuleID() == target_id)
                    {
                        // Calls the Module API method that processes the parameter object included with the message
                        if (!_modules[i]->SetCustomParameters())
                        {
                            // If parameter setting fails, sends an error message to the PC adn aborts the runtime.
                            // Includes ID information about the payload addressee and error statuses from involved
                            // modules
                            const uint8_t errors[4] = {
                                target_type,
                                target_id,
                                _communication.communication_status,
                                _modules[i]->module_status
                            };
                            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersError);
                            SendData(kernel_status, errors);  // Sends the error data to the PC
                            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                            return;
                        }

                        // Otherwise, if the parameters were parsed successfully, ends the reception runtime
                        kernel_status  = static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersSet);
                        kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                        return;
                    }
                }

                // If this point is reached, none of the conditionals above were able to resolve the payload
                // addressee. In this case, sends an error message and aborts the runtime.
                const uint8_t errors[2] = {
                    target_type,
                    target_id,
                };  // payload ID information
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kParametersTargetNotFound);
                SendData(kernel_status, errors);  // Sends the error data to the PC
                kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                return;
            }

            // COMMANDS
            else if (protocol == static_cast<uint8_t>(communication_assets::kProtocols::kCommand))
            {
                const uint8_t target_type = _communication.command_message.module_type;
                const uint8_t target_id   = _communication.command_message.module_id;

                // If the sender requested the reception notification, sends the necessary service message to confirm
                // that the data was received as expected.
                if (const uint8_t return_code = _communication.command_message.return_code)
                {
                    SendServiceMessage(
                        static_cast<uint8_t>(communication_assets::kProtocols::kReceptionCode),
                        return_code
                    );
                }

                // If the command is addressed to the Kernel class, the command is executed immediately.
                if (target_type == kModuleType && target_id == kModuleId)
                {
                    // Resolves and executes the specific command code communicated by the message:
                    switch (auto command = static_cast<kKernelCommands>(_communication.command_message.command))
                    {
                        case kKernelCommands::kResetController:
                            kernel_command = static_cast<uint8_t>(command);  // Adjusts executed command status

                            // Upon receiving the ResetModules command, first resets the parameters and command queues
                            // of all modules managed by the kernel and then reverts their hardware parameters to the
                            // default configuration. This effectively resets the controller to its initial state.
                            for (size_t i = 0; i < _module_count; i++)  // Loops over all modules
                            {
                                // This clears the command queue and cannot fail.
                                _modules[i]->ResetExecutionParameters();

                                // Resets custom assets (parameters, etc.). While it should not fail, it may.
                                if (!_modules[i]->ResetCustomAssets())
                                {
                                    // If resetting custom assets fails, sends the error message to the connected
                                    // system. Note, does not terminate the runtime and attempts to reset the
                                    // rest of the modules. This is to support the use of the reset command as a
                                    // safety measure (it has to reset as much of the controller as possible).

                                    // Includes the ID and status information from the Module.
                                    const uint8_t errors[3] = {target_type, target_id, _modules[i]->module_status};
                                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleResetError);
                                    SendData(kernel_status, errors);
                                }
                            }

                            // If setup method fails, it automatically sends the necessary error message(s) to the
                            // connected system.
                            SetupModules();

                            // Finally, resets the DynamicParameters structure to the default configuration
                            _dynamic_parameters.action_lock = true;
                            _dynamic_parameters.ttl_lock    = true;

                            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kControllerReset);
                            // Notifies the connected system that the controller has been reset
                            SendData(kernel_status);
                            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                            return;

                        case kKernelCommands::kIdentifyController:
                            kernel_command = static_cast<uint8_t>(command);  // Adjusts executed command status

                            // Upon receiving the IdentifyController command, the Kernel sends the unique identification
                            // code of the controller to the connected system.
                            SendServiceMessage(
                                static_cast<uint8_t>(communication_assets::kProtocols::kIdentification),
                                kControllerID
                            );  // Sends the identification code to the PC
                            kernel_status  = static_cast<uint8_t>(kKernelStatusCodes::kControllerIDSent);
                            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                            return;

                        default:
                            // If the command code was not matched with any valid code, sends an error message.
                            // Note, intentionally uses the old (ReceiveData) command status, as this clause means the
                            // Kernel was not able to resolve the command it was asked to carry out.
                            const uint8_t errors[3] = {target_type, target_id, _communication.command_message.command};
                            kernel_status           = static_cast<uint8_t>(kKernelStatusCodes::kKernelCommandUnknown);
                            SendData(kernel_status, errors);  // Sends the error data to the PC
                            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                            return;
                    }
                }

                // Otherwise, loops over the managed modules and attempts to find the adressed module.
                for (size_t i = 0; i < _module_count; i++)
                {
                    // If a Module class is found with matching type and id, queues the command to be executed by the
                    // target module.
                    if (_modules[i]->GetModuleType() == target_type && _modules[i]->GetModuleID() == target_id)
                    {
                        // Directly accesses the necessary Module API methods to queue the command. Unlike parameter
                        // extraction, this method cannot fail.
                        _modules[i]->QueueCommand(
                            _communication.command_message.command,
                            _communication.command_message.noblock,
                            _communication.command_message.cycle,
                            _communication.command_message.cycle_duration
                        );

                        // Ends the reception runtime
                        kernel_status  = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandQueued);
                        kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                        return;
                    }
                }

                // If this point is reached, none of the conditionals above were able to resolve the payload
                // addressee. In this case, sends an error message and aborts the runtime.
                const uint8_t errors[2] = {
                    target_type,
                    target_id,
                };  // payload ID information
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kCommandTargetNotFound);
                SendData(kernel_status, errors);  // Sends the error data to the PC
                kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
                return;
            }

            // INVALID PROTOCOL
            // If the protocol is neither command nor parameters data, this indicates an incompatible message
            // protocol and triggers an error message.
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kInvalidProtocol);
            // Includes the invalid protocol value
            SendData<uint8_t>(kernel_status, _communication.protocol_code);  // Sends the error data to the PC
            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
        }

        /**
         * @brief Loops over the managed modules and for each attempts to resolve and execute a command.
         *
         * This is the second major method of the Kernel class that has to be repeatedly called from the main loop()
         * method. This method concurrently executes the active commands for all managed modules, controlling the
         * connected hardware systems.
         *
         * This method also resolves re-running (recursing) and activating the queued commands (replacing completed
         * commands with newly queued commands).
         *
         * @attention This method automatically attempts to send error messages if it runs into errors. As a side
         * effect of these attempts, it may turn the built-in LED to visually communicate encountered runtime errors.
         * Do not use the LED-connected pin in your code when using this class.
         */
        void RunCommands()
        {
            kernel_command =
                staic_cast<uint8_t>(kKernelCommands::kRunModuleCommands);  // Adjusts executed command status

            // Loops over all managed modules
            for (size_t i = 0; i < _module_count; i++)
            {
                // First, determines whether the module ahs an active command. This relies on the following choice
                // hierarchy: finish already active commands > execute a newly queued command > repeat a cyclic command.
                // Active command setting does not fail due to errors. 'false' means there is no valid command to
                // execute. Running the command, on the other hand, may fail.
                if (_modules[i]->SetActiveCommand())
                {
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
                        SendData(kernel_status, errors);
                        // Does not terminate the runtime and continues with other modules. This is intentionally
                        // designed to allow recovering from non-fatal command execution errors without terminating the
                        // runtime.
                    }
                }
            }

            //Adjusts the status and tracked command and terminates the method runtime.
            kernel_status  = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandError);
            kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);
        }

    private:
        /// An array that stores module classes. These are the classes derived from the base Module class that
        /// encapsulate and expose the API to interface with specific hardware systems managed by the Microcontroller.
        Module** _modules;

        /// Stores the size of the _modules array. Since each module class is likely to be different in terms of its
        /// memory size, the individual classes are accessed using pointer-reference system. For this system to work as
        /// expected, the overall size of the array needs to be stored separately, which is achieved by this
        /// variable
        size_t _module_count;

        /// A reference to the shared instance of the Communication class. This class is used to receive data from the
        /// connected Ataraxis systems and to send feedback data to these systems.
        Communication& _communication;

        /// A reference to the shared instance of the ControllerRuntimeParameters structure. This structure stores
        /// dynamically addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        shared_assets::DynamicRuntimeParameters& _dynamic_parameters;

        /// Stores the unique user-assigned ID of the controller. These IDs help with identifying which controllers
        /// use which serial communication (USB and UART) ports by sending the ID request command.
        constexpr uint8_t kControllerID;
};

#endif  //AXMC_KERNEL_H
