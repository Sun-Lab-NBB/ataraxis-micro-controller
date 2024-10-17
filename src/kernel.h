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
 * - shared_assets.h for globally shared static message byte-codes and parameter structures.
 * - communication.h for Communication class, which is used to bidirectionally communicate with other Ataraxis systems.
 * - module.h for the shared Module API access.
 */

#ifndef AXMC_KERNEL_H
#define AXMC_KERNEL_H

// Dependencies
#include <Arduino.h>
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
 * @tparam module_number The number of custom module classes 9classes derived from the main Module class) that will be
 * managed by the class instance.
 */
template <size_t module_number>
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
            kStandby             = 0,  ///< Standby code used during class initialization.
            kModuleSetupComplete = 1,  ///< SetupModules() method runtime succeeded.
            kModuleSetupError    = 2,  ///< SetupModules() method runtime failed due to a module error.

            kNoDataToReceive           = 2,  ///< No data to receive either due to reception error or genuinely no data
            kPayloadIDDataReadingError = 3,  ///< Unable to extract the ID data of the received payload
            kPayloadParameterReadingError  = 4,   ///< Unable to extract a parameter structure from the received payload
            kInvalidKernelParameterIDError = 5,   ///< Unable to recognize Module-targeted parameter ID
            kInvalidModuleParameterIDError = 6,   ///< Unable to recognize Kernel-targeted parameter ID
            kKernelParameterSet            = 7,   ///< Kernel parameter has been successfully set to input value
            kModuleParameterSet            = 8,   ///< Module parameter has been successfully set to input value
            kNoCommandRequested            = 9,   ///< Received payload did not specify a command to execute
            kUnrecognizedKernelCommand     = 10,  ///< Unable to recognize the requested Kernel-targeted command code
            kEchoCommandComplete           = 11,  ///< Echo Kernel command executed successfully
            kModuleCommandQueued = 12  ///< Module-addressed command has been successfully queued for execution
        };

        enum class kKernelCommands : uint8_t
        {
            kSetupModules = 1,
            k
        };

        /// Communicates the most recent status of the Kernel. Primarily, this variable is used during class method
        /// testing, but it can also be co-opted by any class or method that requires knowing the latest status of the
        /// class runtime.
        uint8_t kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kStandby);

        /**
         * @brief Creates a new Kernel class instance.
         *
         * @param communication A reference to the Communication class instance shared by all other runtime-active
         * classesis the only class that can receive data from the PC.
         * @param dynamic_parameters A reference to the DynamicRuntimeParameters structure used to store addressable
         * controller runtime paramters.
         * @param module_array The array of custom module classes (the children inheriting from the base Module class).
         * The kernel will manage the runtime of these modules.
         */
        Kernel(
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters,
            Module* (&module_array)[module_number]
        ) :
            _modules(module_array),
            _module_count(module_number),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters)
        {}

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
         * @return True if all modules have been successfully set up, false otherwise.
         */
        bool SetupModules()
        {
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

                    // Sends the error message to the PC
                    _communication.SendDataMessage(
                        kModuleType,
                        kModuleId,
                        static_cast<uint8_t>(kKernelCommands::kSetupModules),
                        static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupError),
                        error_object
                    );

                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupError);
                    return false;  // REturns without finishing the setup process.
                }
            }

            // If the method reaches this point, all modules have been successfully set up. Sends the confirmation
            // message to the PC and sets the kernel_status to reflect this successful setup.
            kernel_status                 = static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupComplete);
            constexpr uint8_t placeholder = 0;  // Defines a placeholder as thee is no object to send here.
            _communication.SendDataMessage(
                kModuleType,
                kModuleId,
                static_cast<uint8_t>(kKernelCommands::kSetupModules),
                static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupComplete),
                placeholder
            );

            return true;
        }

        void ReceivePCData()
        {
            // Attempts to receive a message sent by the PC. The messages received from the PC are stored in the
            // circular buffer of the controller. The call to this method attempts to parse the message from the
            // data available in the buffer. If only part of the message was received, the method will block for some
            // time and attempt to receive and parse the whole message.
            bool status = _communication.ReceiveMessage();  // Note, status is reused below

            // If no data was received, breaks the loop. If this is due to reception runtime error, the method internally
            // generates and sends an error message to the PC. Otherwise, if this is due to no data being available (no error),
            // just skips receiving the data during this iteration of the runtime cycle.
            if (!status)
            {
                // Sets the status to indicate there is no data to receive and breaks the runtime. Does not send any messages to
                // the PC as this kind of cases is handled silently to conserve communication bandwidth
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kNoDataToReceive);
                return;
            }

            // Instantiates the ID data structure and reads the received header data into the instantiated structure, if the
            // data has been received
            SerialPCCommunication::IDInformation id_data;
            status = communication_port.ReadIDInformation(id_data);

            // If the read operation fails, breaks the method runtime (the error report is generated and sent by the read
            // method). Sets the status appropriately to indicate specific runtime failure
            if (!status)
            {
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kPayloadIDDataReadingError);
                return;
            }

            // Resolves the target of the received data payload
            AMCModule* target_module = modules[0];  // For modules only, stores the pointer to the target module class
            bool kernel_target =
                true;  // Tracks whether the target of the payload is Kernel itself or one of the Modules

            // If module_id and system_id of the received payload do not match Kernel IDs, searches through the modules until
            // the target module and system combination is found. Note, uses both module_id and system_id to identify
            // Kernel-targeted payloads although the module_id would have been sufficient. This is done as an extra data
            // integrity check.
            if (id_data.module != kCustomKernelParameters::module_id ||
                id_data.system != kCustomKernelParameters::system_id)
            {
                bool target_exists = false;  // Tracks whether the target module and system have been found
                for (size_t i = 0; i < module_count; i++)
                {
                    // If a Module class is found with matching module and system ids, sets up the necessary runtime variables
                    // to proceed with Module-addressed payload processing
                    if (modules[i]->module_id == id_data.module && modules[i]->system_id == id_data.system)
                    {
                        target_exists = true;        // This is needed to escape the error clause below
                        target_module = modules[i];  // Sets the pointer to the target module
                        kernel_target = false;       // Also records that the target is a Module and not Kernel
                        break;                       // Breaks the search loop
                    }
                }

                // If the module loop was unable to resolve the target, issues an error to the PC and aborts method runtime.
                // Note, transmits both the failed module_id and system_id as chained error values
                if (!target_exists)
                {
                    // Sets the kernel status appropriately. Note, uses a system-reserved code 2 which is unique to this
                    // specific error case.
                    kernel_status =
                        static_cast<uint8_t>(axmc_shared_assets::kSystemReservedCodes::kPayloadAddressError);

                    // Includes module and system IDs that led to a no-match error with the error message
                    const uint8_t error_values[2] = {id_data.module, id_data.system};

                    // Since a special (system-unique) code is used, sets the module, system and command to Unknown (0). The
                    // special error code has exactly one meaning across the entirety of the AMC codebase.
                    communication_port.SendErrorMessage(
                        static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kUnknownModule),
                        static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kUnknownSystem),
                        static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kUnknownCommand),
                        kernel_status,
                        error_values
                    );

                    return;  // Breaks method runtime
                }
            }

            // If the target is Kernel, sets internal command tracker to incoming command. This is needed to properly form and
            // send PC messages from-within AMCKernel sub-methods called by this method
            else
            {
                kernel_command = id_data.command;
            }

            // Instantiates parameter-storing structures to be used for parameter data extraction
            SerialPCCommunication::ByteParameter byte_parameter;    // Stores extracted byte-sized parameter data
            SerialPCCommunication::ShortParameter short_parameter;  // Stores extracted short-sized parameter data
            SerialPCCommunication::LongParameter long_parameter;    // Stores extracted long-sized parameter data
            uint16_t parameter_instance;  // Tracks how many parameters have been extracted from the received payload.

            // Determines the total number of parameter structures included in the received data payload
            const uint16_t total_parameter_count = static_cast<uint16_t>(communication_port.byte_data_count) +
                                                   static_cast<uint16_t>(communication_port.short_data_count) +
                                                   static_cast<uint16_t>(communication_port.long_data_count);

            // After resolving the payload target loops over all available parameter structures in the payload and uses them to
            // overwrite the values of target parameters in the appropriate Runtime, Kernel or Module structures. This has to
            // be done prior to resolving the command to handle cases where the transmitted parameters are designed to change
            // how the bundled command is executed.
            for (parameter_instance = 0; parameter_instance < total_parameter_count; parameter_instance++)
            {
                // Relies on the fact that parameter structure types are arranged sequentially along the length of the
                // payload to iteratively process all included parameters. Specifically, reads each parameter into the proper
                // pre-instantiated parameter structure with appropriate byte-size to store the extracted id_code and
                // new parameter value.
                if (communication_port.byte_data_count != 0)  // Byte parameters are found right after header
                {
                    // Reads the byte parameter into the storage structure
                    status = communication_port.ReadParameter(byte_parameter);

                    // Since ReadParameter method handles PC messaging internally, only sets the kernel_status appropriately if
                    // parameter reading sub-method fails and breaks main method runtime.
                    if (!status)
                    {
                        kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kPayloadParameterReadingError);
                        return;
                    }

                    // Calls the appropriate Kernel or Module parameter setting method that attempts to overwrite the specific
                    // parameter targeted by the received parameter structure with the new value
                    if (kernel_target)
                    {
                        // Attempts to overwrite the target parameter with the extracted value. The same method is used to
                        // address variables inside kControllerRuntimeParameters structure and kCustomKernelParameters
                        // structure.
                        status = SetParameter(byte_parameter.code, byte_parameter.value);
                    }
                    else
                    {
                        // Attempts to overwrite the target parameter with the extracted value. The same method is used to
                        // address variables inside kExecutionControlParameters structure and any Custom module parameter
                        // structure. Note, unlike the SetParameter method of the Kernel, the SetParameter method of each module
                        // uses two sub-methods, one for non-virtual Execution parameters structure and another for Custom
                        // structures (virtual method).
                        status = target_module->SetParameter(byte_parameter.code, byte_parameter.value);
                    }
                }
                else if (communication_port.short_data_count != 0)  // Short parameters are found right after Byte parameters
                {
                    // Reads the short parameter into the storage structure
                    status = communication_port.ReadParameter(short_parameter);

                    // Since ReadParameter method handles PC messaging internally, only sets the kernel_status appropriately if
                    // parameter reading sub-method fails and breaks main method runtime.
                    if (!status)
                    {
                        kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kPayloadParameterReadingError);
                        return;
                    }

                    // Calls the appropriate Kernel or Module parameter setting method that attempts to overwrite the specific
                    // parameter targeted by the received parameter structure with the new value
                    if (kernel_target)
                    {
                        // Attempts to overwrite the target parameter with the extracted value. The same method is used to
                        // address variables inside kControllerRuntimeParameters structure and kCustomKernelParameters
                        // structure.
                        status = SetParameter(short_parameter.code, short_parameter.value);
                    }
                    else
                    {
                        // Attempts to overwrite the target parameter with the extracted value. The same method is used to
                        // address variables inside kExecutionControlParameters structure and any Custom module parameter
                        // structure. Note, unlike the SetParameter method of the Kernel, the SetParameter method of each module
                        // uses two sub-methods, one for non-virtual Execution parameters structure and another for Custom
                        // structures (virtual method).
                        status = target_module->SetParameter(short_parameter.code, short_parameter.value);
                    }
                }
                else if (communication_port.long_data_count != 0)  // Long parameters are at the end of the received payload
                {
                    // Reads the long parameter into the storage structure
                    status = communication_port.ReadParameter(long_parameter);

                    // Since ReadParameter method handles PC messaging internally, only sets the kernel_status appropriately if
                    // parameter reading sub-method fails and breaks main method runtime.
                    if (!status)
                    {
                        kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kPayloadParameterReadingError);
                        return;
                    }

                    // Calls the appropriate Kernel or Module parameter setting method that attempts to overwrite the specific
                    // parameter targeted by the received parameter structure with the new value
                    if (kernel_target)
                    {
                        // Attempts to overwrite the target parameter with the extracted value. The same method is used to
                        // address variables inside kControllerRuntimeParameters structure and kCustomKernelParameters
                        // structure.
                        status = SetParameter(long_parameter.code, long_parameter.value);
                    }
                    else
                    {
                        // Attempts to overwrite the target parameter with the extracted value. The same method is used to
                        // address variables inside kExecutionControlParameters structure and any Custom module parameter
                        // structure. Note, unlike the SetParameter method of the Kernel, the SetParameter method of each module
                        // uses two sub-methods, one for non-virtual Execution parameters structure and another for Custom
                        // structures (virtual method).
                        status = target_module->SetParameter(long_parameter.code, long_parameter.value);
                    }
                }

                // If setting the parameter fails, the parameter-method itself sends the error message to the PC. Moreover, for
                // Kernel-targeted methods, the sub-method automatically sets the kernel_status appropriately. However, since
                // Module-specific parameter-setting methods do not have access to the AMCKernel, this clause is required to
                // properly set the kernel_status when Module-addressed parameter setting fails. Also breaks the runtime.
                if (!status && !kernel_target)
                {
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kInvalidModuleParameterIDError);
                    return;
                }

                // For the reasons described above, also has to properly set the kernel_status whenever Module-addressed
                // parameter structure is processed successfully
                if (status && kernel_target)
                {
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleParameterSet);
                    // Note, does not break the runtime, as there may be more parameters to set and / or a command to execute
                }
            }

            // After setting all parameters, handles the received command. kNoCommand code (0) is considered valid and indicates
            // no requested command (used by payload that only contain new parameter values). As such, if code kNoCommand is
            // encountered, immediately finishes processing, as it indicates there is no need to resolve the command.
            if (id_data.command == static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand))
            {
                // Sets the status to indicate that no command has been requested. From this status, it is indirectly inferrable
                // that the method runtime has finished successfully.
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kNoCommandRequested);
                return;
            }

            // If a command code is provided, executes the appropriate command handling method
            if (kernel_target)
            {
                // If the command is addressed to the kernel, runs it in-place. Usually, kernel commands are reserved for
                // testing purposes and other generally non-cyclic execution cases, so they do not come with inherent
                // noblock execution capacity and are not part of the general runtime flow per se. THe method automatically
                // handles setting kernel_status and sending messages to the PC.
                RunKernelCommand(id_data.command);
            }

            // For commands addressed to Modules, does not run them in-place, but instead queues the command for execution by
            // saving the code to the appropriate field fo the ExecutionControlParameters structure of the Module. As such, the
            // method does not actually check for command code validity, that is outsourced the method that actually activates
            // the command for execution (called via a separate AMCKernel method).
            else
            {
                target_module->QueueCommand(id_data.command);

                // Sets the status to communicate that the command has been queued
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleCommandQueued);
            }

            return;  // Ends method runtime
        }

        void RunCommands()
        {}

        /**
         * @brief Updates the variable inside either kControllerRuntimeParameters or kCustomKernelParameters structure
         * specified by the input id_code to use the input value.
         *
         * The value is converted to the same type as the target variable prior to being written to the variable.
         *
         * This method generates and sends the appropriate message to the PC to indicate runtime success or failure.
         *
         * @note This method is used to address both global runtime parameters and local custom parameters. This simplifies
         * the codebase and error handling at the expense of the requirement to have extra care when modifying either
         * structure, the address code enumerator or this method.
         *
         * @param id_code The unique byte-code for the specific field inside the kControllerRuntimeParameters or
         * kCustomKernelParameters structure to be modified. These codes are defined in the kAddressableParameterCodes
         * private enumeration of this class and the id-code of the target parameter transmitted inside each Parameter
         * structure used to set the parameter has to match one of these codes to properly target a parameter value.
         * @param value The specific value to set the target parameter variable to.
         *
         * @returns bool @b true if the value was successfully set, @b false otherwise. The only realistic reason for
         * returning false is if the input id_code does not match any of the id_codes available through the
         * kAddressableParameterCodes enumeration.
         */
        bool SetParameter(const uint8_t id_code, const uint32_t value)
        {
            bool parameter_set = true;  // Tracks whether the target parameter has been found and processed

            // Casts the input id_code to be the same type as the enumeration and then, depending on which unique id the
            // input code matches, sets the corresponding variable's value to the input value.
            switch (static_cast<kAddressableParameterCodes>(id_code))
            {
                // Runtime Action Lock
                case kAddressableParameterCodes::kActionLock:
                    dynamic_runtime_parameters.action_lock = static_cast<bool>(value);
                    break;

                // Runtime TTL Lock
                case kAddressableParameterCodes::kTTLLock:
                    dynamic_runtime_parameters.ttl_lock = static_cast<bool>(value);
                    break;

                // Runtime Simulate Responses
                case kAddressableParameterCodes::kSimulateResponses:
                    dynamic_runtime_parameters.simulate_responses = static_cast<bool>(value);
                    break;

                // Runtime Simulate Specific Responses
                case kAddressableParameterCodes::kSimulateSpecificResponses:
                    dynamic_runtime_parameters.simulate_specific_responses = static_cast<bool>(value);
                    break;

                // Runtime Simulated Value
                case kAddressableParameterCodes::kSimulatedValue:
                    dynamic_runtime_parameters.simulated_value = static_cast<uint32_t>(value);
                    break;

                // Runtime Minimum Duration Lock
                case kAddressableParameterCodes::kMinimumLockDuration:
                    dynamic_runtime_parameters.minimum_lock_duration = static_cast<uint32_t>(value);
                    break;

                // No match found in either structure, returns 'false' to indicate method failure
                default:
                    parameter_set = false;  // Indicates that the parameter assignment failed
                    break;
            }

            // Sends a message to the PC to communicate the runtime status and returns the status for the main method to resolve
            // its runtime.
            if (!parameter_set)
            {
                // Error message to indicate the method was not able to match the input id_code to any known code. Includes the
                // specific parameter id code that failed the matching procedure. Uses kNoCommand code to indicate the message
                // originates from a non-command method.
                const uint8_t error_values[1] = {id_code};
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kInvalidKernelParameterIDError);
                communication_port.SendErrorMessage(
                    kCustomKernelParameters::module_id,
                    kCustomKernelParameters::system_id,
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    kernel_status,
                    error_values
                );
                return false;
            }
            else
            {
                // Success message to indicate that the parameter targeted by the ID code has been set to a new value (also
                // includes the value). Uses kNoCommand code to indicate the message originates from a non-command method.
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kKernelParameterSet);
                communication_port.CreateEventHeader(
                    kCustomKernelParameters::module_id,
                    kCustomKernelParameters::system_id,
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    kernel_status
                );
                communication_port.PackValue(id_code, value);
                communication_port.SendData();
                return true;
            }
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
        const shared_assets::DynamicRuntimeParameters& _dynamic_parameters;
};

#endif  //AXMC_KERNEL_H
