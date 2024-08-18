//Implementation of Kernel functions

#include "kernel.h"

// The constructor accepts and sets the internal communication_port reference to the input SerialPCCommunication class
// instance (by reference) and also initializes internal modules array and module_count counter to 0. These two
// variables are properly set via the SetModules() class template method.
AMCKernel::AMCKernel(SerialPCCommunication& serial_communication_port) :
    modules(nullptr), module_count(0), initialization_finished(false), communication_port(serial_communication_port)
{}

// Loops over all Modules and triggers the setup method for each module. Intended to be used whenever the general
// setup() loop would typically be used. Handles appropriate success and failure messaging cases internally.
void AMCKernel::SetupModules()
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

// If there is data to be received from the PC, receives and processes the data. This is a fairly complex and involved
// process with multiple subcomponents prone to failure. If an error is encountered anywhere during the process, the
// reception cycle is aborted and an error message is set to PC. Due to how this method is implemented, that may mean
// that some Module / Kernel parameters becomes overwritten with new parameters sent from the PC and others don't. As
// such, errors originating from this method can potentially lead to unwanted behavior if not properly handled.
void AMCKernel::ReceivePCData()
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
    bool kernel_target       = true;  // Tracks whether the target of the payload is Kernel itself or one of the Modules

    // If module_id and system_id of the received payload do not match Kernel IDs, searches through the modules until
    // the target module and system combination is found. Note, uses both module_id and system_id to identify
    // Kernel-targeted payloads although the module_id would have been sufficient. This is done as an extra data
    // integrity check.
    if (id_data.module != kCustomKernelParameters::module_id || id_data.system != kCustomKernelParameters::system_id)
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
            kernel_status = static_cast<uint8_t>(axmc_shared_assets::kSystemReservedCodes::kPayloadAddressError);

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

// Updates the variable specified by the id_code either inside kControllerRuntimeParameters or kCustomKernelParameters
// structure to use the input value. Since the two structures share the address code enumeration, the same switch
// statement can be used to address both structures. Sends the success or error message (depending on the result of the
// runtime) to the PC and returns the boolean success status to caller method
bool AMCKernel::SetParameter(const uint8_t id_code, const uint32_t value)
{
    bool parameter_set = true;  // Tracks whether the target parameter has been found and processed

    // Casts the input id_code to be the same type as the enumeration and then, depending on which unique id the
    // input code matches, sets the corresponding variable's value to the input value.
    switch (static_cast<kAddressableParameterCodes>(id_code))
    {
        // Runtime Action Lock
        case kAddressableParameterCodes::kActionLock: runtime_parameters.action_lock = static_cast<bool>(value); break;

        // Runtime TTL Lock
        case kAddressableParameterCodes::kTTLLock: runtime_parameters.ttl_lock = static_cast<bool>(value); break;

        // Runtime Simulate Responses
        case kAddressableParameterCodes::kSimulateResponses:
            runtime_parameters.simulate_responses = static_cast<bool>(value);
            break;

        // Runtime Simulate Specific Responses
        case kAddressableParameterCodes::kSimulateSpecificResponses:
            runtime_parameters.simulate_specific_responses = static_cast<bool>(value);
            break;

        // Runtime Simulated Value
        case kAddressableParameterCodes::kSimulatedValue:
            runtime_parameters.simulated_value = static_cast<uint32_t>(value);
            break;

        // Runtime Minimum Duration Lock
        case kAddressableParameterCodes::kMinimumLockDuration:
            runtime_parameters.minimum_lock_duration = static_cast<uint32_t>(value);
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
        kernel_status                 = static_cast<uint8_t>(kKernelStatusCodes::kInvalidKernelParameterIDError);
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

// Runs the requested kernel-level command, if the input command code points to a valid command. Sends the message to
// notify the PC about the success or failure of the method and returns the boolean runtime status. Expects NoCommand
// code to be handled outside of this method. Does not return any values, but does set kernel_status to communicate the
// rutnime status properly.
void AMCKernel::RunKernelCommand(const uint8_t command)
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

bool AMCKernel::Echo()
{
    return true;
}
