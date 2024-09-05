// KERNEL

#ifndef AMC_KERNEL_H
#define AMC_KERNEL_H

// Dependencies
#include "Arduino.h"
#include "module.h"
#include "shared_assets.h"

/**
 * @brief The main Core level class that provides the central runtime loop and all kernel-level methods that control
 * data reception, runtime flow, setup and shutdown.
 *
 * @attention This class functions as the backbone of the AMC codebase by providing runtime flow control functionality.
 * Without it, the AMC codebase would not function as expected. Any modification to this class should be carried out
 * with extreme caution and, ideally, completely avoided by anyone other than the kernel developers of the project.
 *
 * The class contains both major runtime flow methods and minor kernel-level commands that provide general functionality
 * not available through AMCModule (the backbone of every Module-level class). Overall, this class is designed
 * to organically compliment AMCModule methods and together the two classes provide a robust set of features to realize
 * almost any conceivable custom Controller-mediated behavior.
 *
 * @note This class is not fully initialized until it's SetModules method is used to provide it with an array of
 * AMCModule-derived Module-level classes. The vast majority of methods of this class require a correctly set array of
 * modules to operate properly.
 */
class Kernel
{
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
         * The Kernel class uses these byte-codes to communicate the exact status of various Kernel runtimes between class
         * methods and to send messages to the PC.
         *
         * @Note Should not use system-reserved codes 0 through 10 and 250 through 255.
         */
        enum class kKernelStatusCodes : uint8_t
        {
            kStandby                   = 11,  ///< Standby code used during initial class initialization
            kModulesNotSetError        = 12,  ///< A successful SetModules() runtime is required to use other methods
            kModulesSet                = 13,  ///< SetModules() method runtime succeeded
            kModuleSetupComplete       = 14,  ///< SetupModules() method runtime succeeded
            kNoDataToReceive           = 15,  ///< No data to receive either due to reception error or genuinely no data
            kPayloadIDDataReadingError = 16,  ///< Unable to extract the ID data of the received payload
            kPayloadParameterReadingError  = 17,  ///< Unable to extract a parameter structure from the received payload
            kInvalidKernelParameterIDError = 18,  ///< Unable to recognize Module-targeted parameter ID
            kInvalidModuleParameterIDError = 19,  ///< Unable to recognize Kernel-targeted parameter ID
            kKernelParameterSet            = 20,  ///< Kernel parameter has been successfully set to input value
            kModuleParameterSet            = 21,  ///< Module parameter has been successfully set to input value
            kNoCommandRequested            = 22,  ///< Received payload did not specify a command to execute
            kUnrecognizedKernelCommand     = 23,  ///< Unable to recognize the requested Kernel-targeted command code
            kEchoCommandComplete           = 24,  ///< Echo Kernel command executed successfully
            kModuleCommandQueued = 25  ///< Module-addressed command has been successfully queued for execution
        };

        /**
         * @enum kKernelCommandCodes
         * @brief Specifies the id byte-codes for the available Kernel-level commands. These id-codes should be used by the
         * PC to trigger specific Kernel commands upon data reception.
         *
         * Each kernel command should have a matching code stored in this enumeration in addition to a properly configured
         * trigger subroutine available through the RunKernelCommand() method of the AMCKernel class.
         *
         * @attention Should not use codes 0 through 10 and 250 through 255.
         */
        enum class kKernelCommandCodes : uint8_t
        {
            kEcho = 11  ///< Re-packages the received data and returns it back to the PC.
        };

        /// Communicates the most recent status of the Kernel. Primarily, this variable is used during class method testing,
        /// but it can also be co-opted by any class or method that at any point requires knowing the latest status of the
        /// kernel class runtime. This tracker is initialized to kStandby from the kKernelStatusCodes.
        uint8_t kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kStandby);

        /// Tracks the currently active kernel command. This information is used alongside some other static
        /// kCustomKernelParameters to properly send error messages from commands and other encapsulated contexts
        uint8_t kernel_command;

        /**
         * @brief Creates a new AMCKernel class instance.
         *
         * @attention The class initialization has to be completed by calling SetModules() method prior to calling any
         * other method. If this is not done, the Kernel is likely to behave unexpectedly.
         *
         * @param serial_communication_port A reference to the SerialPCCommunication class instance shared by all AMC
         * codebase classes that receive (AMCKernel only!) or send (AMCKernel and all Modules) data over the serial port.
         * The AMCKernel class uses this instance to receive all data sent from the PC and to send some event data to the
         * PC.
         */
        Kernel(Communication& communication, const shared_assets::DynamicRuntimeParameters& dynamic_parameters) :
            modules(nullptr),
            module_count(0),
            initialization_finished(false),
            _communication(communication),
            _dynamic_parameters(dynamic_parameters)
        {}

        /**
         * @brief Sets the internal modules array to the provided array of AMCModule-derived Module-level classes and
         * module_count variable to the automatically inferred total modules count.
         *
         * This method finishes the AMCKernel initialization process started by the class Constructor by providing it with
         * the array of Modules to operate upon. The AMC runtime is structured around the Module classes providing specific
         * Controller behaviors (in a physical-system-dependent fashion) and the AMCKernel class synchronizing and
         * controlling Module-specific subroutine execution flow.
         *
         * @note Expects valid arrays with size of at least 1 configured module. Will break early without finishing the
         * Module Setting and send an error-message to the PC if provided with an invalid array. Technically this should not
         * be possible in C++, but this case is verified and guarded against regardless.
         *
         * @tparam module_number The number of elements inside the provided array. This number is derived automatically
         * from the input array size and, in so-doing, eliminates the potential for user-errors when inputting the modules
         * array.
         * @param module_array The array of type AMCModule filled with the children inheriting from the Core base AMCModule
         * class.
         */
        template <size_t module_number>
        void SetModules(Module* (&module_array)[module_number])
        {
            // Requires at least one module to be provided for the correct operation, so aborts the method if an empty
            // array is provided to the method.
            if (module_number < 1)
            {
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModulesNotSetError);  // Sets error status

                // Sends a message to the PC to communicate that the method runtime failed. Assumes the communication class
                // is initialized first and the operation is actually possible and meaningful.
                communication_port.CreateEventHeader(
                    static_cast<uint8_t>(kCustomKernelParameters::module_id),
                    static_cast<uint8_t>(kCustomKernelParameters::system_id),
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    kernel_status
                );
                communication_port.SendData();
                return;  // Breaks method runtime
            }

            // Initializes internal variables using input data
            modules      = module_array;
            module_count = module_number;

            // Sets the lock-in flag to 'true' to allow executing other methods of the class. The class is initialized with
            // this flag being false and is unable to call any method other than SetModules() until the flag is set to true.
            initialization_finished = true;

            // Sets the status to indicate that the method runtime was successful
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModulesSet);
            return;
        }

        /**
         * @brief Triggers the SetupModule() method of each module inside the modules array.
         *
         * This method is only triggered once, during the runtime fo the main script setup() loop. Consider this an API
         * wrapper around the setup sequences of each modules to be used by the Kernel class.
         *
         * @note  Automatically sends a success or error message to the PC and, as such, assumes this method is always
         * called after finished setting up communication class. The only way for this method to fail it's runtime is to
         * be called prior to SetModules() method of this class.
         *
         * @attention Sets kernel_status to kModulesNotSetError code if runtime fails, as currently the only way for this
         * method to fail is if it is called prior to the SetModules() method . Uses kModuleSetupComplete code to indicate
         * successful runtime. All codes can be found in kKernelStatusCodes enumeration.
         */
        void SetupModules()
        {
            // Prevents the method from being executed until the AMCKernel class has been properly initialized.
            if (!initialization_finished)
            {
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModulesNotSetError);  // Setup method failure

                // Sends the error message to the PC to indicate setup failure. Expects the communication class to be fully
                // operational at the time of this method's runtime. Uses kNoCommand code to indicate that the message
                // originates from a non-command method.
                communication_port.CreateEventHeader(
                    static_cast<uint8_t>(kCustomKernelParameters::module_id),
                    static_cast<uint8_t>(kCustomKernelParameters::system_id),
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    kernel_status
                );
                communication_port.SendData();

                return;  // Breaks the runtime
            }

            // Loops over each module and calls its Setup() virtual method. Note, expects that setup methods cannot fail, which
            // is the case for an absolute majority of all conceived runtimes.
            for (size_t i = 0; i < module_count; i++)
            {
                modules[i]->SetupModule();
            }

            // Communicates method's runtime success to the PC and via kernel status and returned value
            kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupComplete);

            // Sends success message to the PC. Uses kNoCommand code to indicate that the message originates from a non-command
            // method.
            communication_port.CreateEventHeader(
                static_cast<uint8_t>(kCustomKernelParameters::module_id),
                static_cast<uint8_t>(kCustomKernelParameters::system_id),
                static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                kernel_status
            );
            communication_port.SendData();

            // Ends method runtime
            return;
        }

        void ReceivePCData()
        {
            bool status;  // Tracks the status of each sub-method call to catch if any method failures

            // Prevents method runtime if AMCKernel class initialization has not been finished and sends an appropriate
            // error message to the PC. Uses kNoCommand code since no actual command code is yet available (dat reception method
            // is prevented from running).
            if (!initialization_finished)
            {
                kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kModulesNotSetError);  // Setup method failure
                communication_port.CreateEventHeader(
                    static_cast<uint8_t>(kCustomKernelParameters::module_id),
                    static_cast<uint8_t>(kCustomKernelParameters::system_id),
                    static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
                    kernel_status
                );
                communication_port.SendData();
                return;
            }

            // Resets the communication port to receive new data. Note, if there is any unprocessed data in the temporary
            // communication_port buffer, it will be cleared out by this action.
            communication_port.Reset();

            // If no data was received, breaks the loop. If this is due to reception runtime error, the method internally
            // generates and sends an error message to the PC. Otherwise, if this is due to no data being available (no error),
            // just skips receiving the data during this iteration of the runtime cycle.
            if (!communication_port.ReceiveData())
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

        // void TriggerModuleRuntimes();

        // void ResetController();

        // void RuntimeCycle();

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

        /**
         * @brief Determines whether the input command code is valid and, if so, runs the requested AMCKernel clas command.
         *
         * @param command The id byte-code for the command to run. Can be 0 when no command is requested, the method will
         * automatically hand;e this as a correct non-erroneous no-command case.
         *
         * @note Uses kernel_status to communicate the success or failure of the method alongside the specific success or
         * failure code. Has no return value as it is not really helpful given the context of how this command is called.
         */
        void RunKernelCommand(uint8_t command)
        {
            // Casts the input command code to kKernelCommandCodes enumeration to match it to an existing id code. Expects that
            // any failure originating from the command runtime is handled at the command level and only handles the error
            // message due to an unknown command ID.
            switch (static_cast<kKernelCommandCodes>(command))
            {
                // Echo
                case kKernelCommandCodes::kEcho: Echo(); return;

                // If the input command is not recognized, returns false to indicate command runtime failure, sets the status
                // appropriately and sends an error message to the PC to indicate that the input command was not recognized
                default:
                    kernel_status = static_cast<uint8_t>(kKernelStatusCodes::kUnrecognizedKernelCommand);
                    communication_port.CreateEventHeader(
                        kCustomKernelParameters::module_id,
                        kCustomKernelParameters::system_id,
                        command,
                        kernel_status
                    );
                    communication_port.SendData();
                    return;
            }
        }

        bool Echo()
        {
            return true;
        }

    private:
        /// An internal array of AMCModule-derived Module-level classes. This array is set via SetModules() class method
        /// and it is used to store all Module's available to the Kernel during runtime. The methods of the Kernel loop over
        /// this array and call common interface methods inherited by each module from the core AMCModule class to execute
        /// Controller runtime behavior.
        Module** modules;

        /// A complimentary variable used to store the size of the AMCModule array. This is required for the classic
        /// realization of the array-pointer + size looping strategy used in C-based languages instead of template-based
        /// strategy. Since SetModules() is an internal class method that sets this variable through a template approach,
        /// the use of typically user-error-prone pointer+count method is completely safe for all other methods of the
        /// Kernel class, assuming SetModules() has been called (this is enforced).
        size_t module_count;

        /// A tracker flag used to determine whether SetModules() method has been used to properly finish class instance
        /// initialization. This flag is used to ensure it is impossible to call other methods of the class unless
        /// SetModules() has been called. This is a safety mechanism that helps avoid unexpected behavior.
        bool initialization_finished;

        // A reference to the shared instance of the Communication class. This class is used to send runtime data to
        /// the connected Ataraxis system.
        Communication& _communication;

        /// A reference to the shared instance of the ControllerRuntimeParameters structure. This structure stores
        /// dynamically addressable runtime parameters used to broadly alter controller behavior. For example, this
        /// structure dynamically enables or disables output pin activity.
        const shared_assets::DynamicRuntimeParameters& _dynamic_parameters;
};

#endif  //AMC_KERNEL_H
