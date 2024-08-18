/**
 * @file
 * @brief The implementation file for the Core AMCModule Core class.
 *
 * @subsection description Description:
 * Contains the following elements:
 * - All non-virtual method implementations as well as the class constructor.
 *
 * @note All virtual methods are declared as purely virtual and, as such, have to be implemented separately for each
 * child class.
 *
 * @see amc_module.h for method documentation.
 */

#include "module.h"

// Class Constructor implementation. Takes in the reference to the ControllerRuntimeParameters structure instantiated
// by the AMCKernel class as a constant reference and a non-constant reference to the SerialPCCommunication class
// instance shared by all classes that require PC communication. Also takes direct assignments for module type and
// instance IDs.
AMCModule::AMCModule(
    uint8_t module_type_id,
    uint8_t module_instance_id,
    const axmc_shared_assets::ControllerRuntimeParameters& controller_runtime_parameters,
    SerialPCCommunication& serial_communication_port
) :
    module_id(module_type_id),
    system_id(module_instance_id),
    runtime_parameters(controller_runtime_parameters),
    communication_port(serial_communication_port)
{}

// Utility methods

// Simulates or casts preset simulated value as a boolean type
bool AMCModule::GenerateBool()
{
    if (runtime_parameters.simulate_specific_responses)
    {
        // If exact value use is enabled, converts the simulated value to a bool-compatible range (0 to 1) by
        // returning 0 if the input value is 0 and otherwise rounding it down to 1
        return runtime_parameters.simulated_value > 0;
    }
    else
    {
        // Otherwise (if random simulation is enabled), returns a random value between 0 and 1 (HIGH and LOW)
        return static_cast<bool>(random(2));
    }
}

// Simulates or casts preset simulated value as an uint8_t type
uint8_t AMCModule::GenerateByte()
{
    if (runtime_parameters.simulate_specific_responses)
    {
        // If exact value use is enabled, converts the simulated value to a byte-compatible range (0 to 255) by
        // capping it at 255 if the input is above 255 and passing the value as-is if it is at or below 255
        return (runtime_parameters.simulated_value <= 255) ? static_cast<uint8_t>(runtime_parameters.simulated_value)
                                                           : 255;
    }
    else
    {
        // Otherwise (if random simulation is enabled), returns a random value between 0 and 255
        return static_cast<uint8_t>(random(256));
    }
}

// Simulates or casts preset simulated value as an uint16_t type
uint16_t AMCModule::GenerateUnsignedShort()
{
    if (runtime_parameters.simulate_specific_responses)
    {
        // If exact value use is enabled, converts the simulated value to an unsigned short-compatible range (0 to
        // 65535) by rounding down to 65535 if the input is above 65535 and passing the value as-is if it is at or
        // below 65535
        return (runtime_parameters.simulated_value <= 65535) ? static_cast<uint16_t>(runtime_parameters.simulated_value)
                                                             : 65535;
    }
    else
    {
        // Otherwise (if random simulation is enabled), returns a random value between 0 and 65535
        return static_cast<uint16_t>(random(65536));
    }
}

// Simulates or casts preset simulated value as an uint32_t type
uint32_t AMCModule::GenerateUnsignedLong()
{
    if (runtime_parameters.simulate_specific_responses)
    {
        // If exact value use is enabled, returns the value as is, since simulated_value is already within unsigned
        // long range
        return runtime_parameters.simulated_value;
    }
    else
    {
        // Otherwise (if random simulation is enabled), generates a random value.
        // Note: random() in Arduino does not directly support the full uint32_t range. A workaround is implemented
        // below that generates uint32 by separately generating two 16-bit values and combining them to produce
        // the final 32-bit value.
        uint32_t high = random(0, 65536);  // Generates the high part
        uint32_t low  = random(0, 65536);  // Generates the low part
        return (high << 16) | low;         // Combines high and low to create a full 32-bit random number
    }
}

// Either simulates or polls an analog pin (with averaging, if required) and returns the analog-to-digital-converted
// pin value as uint16_t. Can average multiple raw values together to produce the final readout if required.
uint16_t AMCModule::GetRawAnalogReadout(
    const uint8_t pin,        // The analog pin to be polled
    const uint16_t pool_size  // The number of pin poll values to average to produce the final readout
)
{
    uint16_t average_readout;  // Pre-declares the final output readout as an unit16_t

    // If response simulation is enabled, calls a value generation function
    if (runtime_parameters.simulate_responses)
    {
        average_readout = GenerateUnsignedShort();
    }

    // Otherwise uses real pin values
    else
    {
        // Pool size 0 and 1 mean essentially the same: no averaging, so lumps the check into < 2
        if (pool_size < 2)
        {
            // If averaging is disabled, simply reads and outputs the acquired value. An optimization for teensies can
            // be disabling software averaging and relying on hardware averaging instead of using this execution path
            average_readout = analogRead(pin);
        }
        else
        {
            uint32_t accumulated_readouts = 0;  // Aggregates polled values by self-addition

            // If averaging is enabled, repeatedly polls the pin the requested number of times.
            // Note, while this can be further optimized on teensies by manipulating pin averaging, this is not
            // available for Arduino's, so a more cross-platform-friendly, but less optimal procedure is used here.
            // NOTE, i's type always has to be the same as pool size (currently uint16_t), hence why the auto and
            // decltype check below.
            for (auto i = decltype(pool_size) {0}; i < pool_size; i++)
            {
                accumulated_readouts += analogRead(pin);  // Aggregates readouts via self-addition
            }

            // Here the averaging and rounding is performed to obtain an integer value rather than dealing with floating
            // point arithmetic. This is another Arduino-favoring optimization as teensy boards come with a hardware FP
            // module that removes a lot of the speed loss associated with FP arithmetic on Arduino boards.
            // Note, adding pool_size/2 before dividing by pool_size forces half-up ('standard') rounding.
            // Also, explicitly ensures that the final value is cast to uint16_t.
            average_readout = static_cast<uint16_t>((accumulated_readouts + pool_size / 2) / pool_size);
        }
    }

    return average_readout;
}

// Either simulates or polls a digital pin and returns the readout value as a boolean. Attempts to use a fast readout
// method if possible (via digitalWriteFast library). Can average multiple raw values together to produce the final
// readout if required.
bool AMCModule::GetRawDigitalReadout(
    const uint8_t pin,  // The digital pin to read. Has to be constant and set to read for the fast reading to work
    const uint16_t pool_size  // The number of pin poll values to average to produce the final readout
)
{
    bool digital_readout;  // Pre-declares the final output readout as a boolean

    // If response simulation is enabled, calls a value generation function
    if (runtime_parameters.simulate_responses)
    {
        digital_readout = GenerateBool();
    }

    // Otherwise reads the physical sensor value. Uses pooling as an easy way to implement custom debouncing logic,
    // which often relies on averaging multiple digital readouts to confirm that the pin is stably active or inactive.
    // Generally, this is a built-in hardware feature for many boards, but the logic is maintained here for backward
    // compatibility reasons.
    else
    {
        // Pool size 0 and 1 mean essentially the same: no averaging, so lumps the check into < 2
        if (pool_size < 2)
        {
            // If averaging is disabled, simply reads and outputs the acquired value
            digital_readout = digitalReadFast(pin);
        }
        else
        {
            uint32_t accumulated_readouts = 0;  // Aggregates polled values by self-addition

            // If averaging is enabled, repeatedly polls the pin the requested number of times.
            // NOTE, i's type always has to be the same as pool size (currently uint16_t), hence why the auto and
            // decltype check below.
            for (auto i = decltype(pool_size) {0}; i < pool_size; i++)
            {
                accumulated_readouts += digitalReadFast(pin);  // Aggregates readouts via self-addition
            }

            // Here the averaging and rounding is performed to obtain an integer value rather than dealing with floating
            // point arithmetic. This is another Arduino-favoring optimization as teensy boards come with a hardware FP
            // module that removes a lot of the speed loss associated with FP arithmetic on Arduino boards.
            // Note, adding pool_size/2 before dividing by pool_size forces half-up ('standard') rounding.
            // Also, explicitly ensures that the final value is cast to boolean true or false.
            digital_readout = static_cast<bool>((accumulated_readouts + pool_size / 2) / pool_size);
        }
    }

    return digital_readout;
}

// Depending on the noblock configuration of the executed command, either blocks in-place for the user-defined number of
// microseconds or checks whether user-defined number of microseconds has passed since the last reset of the stage
// timer. Returns 'true' if the required number of microseconds has passed (blocking or not) and 'false' otherwise.
bool AMCModule::WaitForMicros(
    const uint32_t delay_duration  // The duration that this function should check for, in microseconds
)
{
    // If the caller command is executed in blocking mode, blocks in-place (inferred from the noblock flag)
    if (!execution_parameters.noblock)
    {
        // Blocks until delay_duration has passed
        while (execution_parameters.delay_timer <= delay_duration)
            ;

        return true;  // Returns true, the duration has been enforced
    }

    // Otherwise, if the caller function is executed in non-blocking mode, checks if the minimum duration has passed
    // and returns the check condition
    return (execution_parameters.delay_timer >= delay_duration);
}

// Depending on the noblock configuration of the executed command, either blocks in-place until the evaluated analog
// sensor value satisfies the threshold condition or checks whether the evaluated sensor value satisfies the threshold
// condition. Has a break-out mechanism that forcibly terminates the waiting loop if the sensor does not satisfy the
// condition for timeout number of microseconds. Returns a specific byte-code that indicates the result of method
// runtime, which is available through kReturnedUtilityCodes enumeration.
uint8_t AMCModule::WaitForAnalogThreshold(
    const uint8_t sensor_pin,     // The input pin (number) of the sensor to be evaluated
    const bool invert_condition,  // Allows inverting the default >= comparison to use <= operator instead
    const uint16_t threshold,     // The value of the sensor that is considered 'passing', depending on operator
    const uint32_t timeout,       // The number of microseconds after which method returns code 13 (times-out)
    const uint16_t pool_size      // The number of sensor readouts to average to give the final readout
)
{
    uint16_t value;  // Precreates the variable to store the sensor value
    bool passed;     // Tracks whether the threshold check is passed

    uint32_t min_delay = 0;  // Pre-initializes to 0, which disables the minimum delay if simulation is not used
    if (runtime_parameters.simulate_responses)
    {
        // If simulate_responses flag is true, uses minimum_lock_duration to enforce the preset minimum delay
        // duration
        min_delay = runtime_parameters.minimum_lock_duration;
    }

    // Obtains the real or simulated sensor value (the function comes with built-in simulation resolution)
    value = GetRawAnalogReadout(sensor_pin, pool_size);

    if (!invert_condition)
    {
        // If inversion is disabled, passing condition is value being equal to or greater than the threshold
        // Also factors in the minimum delay as otherwise it is not properly assessed when the method runs in non-blocking
        // mode
        passed = (value >= threshold) && (execution_parameters.delay_timer > min_delay);
    }
    else
    {
        // If inversion is enabled, passing condition is value being less than or equal to the threshold
        // Also factors in the minimum delay as otherwise it is not properly assessed when the method runs in non-blocking
        // mode
        passed = (value <= threshold) && (execution_parameters.delay_timer > min_delay);
    }

    // If the caller command is executed in blocking mode, blocks in-place until either a timeout duration of
    // microseconds passes or the sensor value exceeds the threshold
    if (!execution_parameters.noblock)
    {
        // This blocks until BOTH the min_delay has passed AND the sensor value exceeds the threshold
        while (execution_parameters.delay_timer <= min_delay || !passed)
        {
            // Obtains the sensor readout and checks it against the threshold on every iteration of the while loop
            value = GetRawAnalogReadout(sensor_pin, pool_size);
            if (!invert_condition)
            {
                passed = (value >= threshold);
            }
            else
            {
                passed = (value <= threshold);
            }

            // If the delay timer exceeds timeout value, interrupts and returns kWaitForAnalogThresholdTimeout (13),
            // which is a special timeout-quit code
            if (execution_parameters.delay_timer > timeout)
            {
                return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdTimeout);
            }
        }

        // Returns kWaitForAnalogThresholdPass (12) to indicate that the check has passed
        return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdPass);
    }

    // Otherwise, if the caller function is executed in non-blocking mode, determines what code to return
    else
    {
        // If the delay timer exceeds timeout, returns kWaitForAnalogThresholdTimeout (13) to indicate timeout condition
        if (execution_parameters.delay_timer > timeout)
            return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdTimeout);

        // Otherwise, if check passed, returns kWaitForAnalogThresholdPass (12)
        else if (passed) return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdPass);

        // Otherwise, returns kWaitForAnalogThresholdFailure (11)
        else return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForAnalogThresholdFailure);
    }
}

// Very similar to WaitForAnalogThreshold, but works for digital sensors.
// Depending on the noblock configuration of the executed command, either blocks in-place until the evaluated digital
// sensor value satisfies the threshold condition or checks whether the evaluated sensor value satisfies the threshold
// condition. Has a break-out mechanism that forcibly terminates the waiting loop if the sensor does not satisfy the
// condition for timeout number of microseconds. Returns a specific byte-code that indicates the result of method
// runtime, which is available through kReturnedUtilityCodes enumeration.
uint8_t AMCModule::WaitForDigitalThreshold(
    const uint8_t sensor_pin,  // The input pin (number) of the digital sensor to be evaluated
    const bool threshold,      // The value of the sensor that is considered 'passing'
    const uint32_t timeout,    // The number of microseconds after which function returns code 13 (times-out)
    const uint16_t pool_size   // The number of sensor readouts to average to give the final readout
)
{
    bool value;  // Precreates the variable to store the sensor value

    uint32_t min_delay = 0;  // Pre-initializes to 0, which disables the minimum delay if simulation is not used
    if (runtime_parameters.simulate_responses)
    {
        // If simulate_responses flag is true, uses minimum_lock_duration to enforce the preset minimum delay
        // duration
        min_delay = runtime_parameters.minimum_lock_duration;
    }

    // Obtains the real or simulated sensor value (the function comes with built-in simulation resolution)
    value = GetRawDigitalReadout(sensor_pin, pool_size);

    // If the caller command is executed in blocking mode, blocks in-place until either a timeout duration of
    // microseconds passes or the sensor value matches the threshold
    if (!execution_parameters.noblock)
    {
        // This blocks until BOTH the min_delay has passed AND the sensor value exceeds the threshold
        while (execution_parameters.delay_timer <= min_delay || value != threshold)
        {
            // Obtains the sensor readout and checks it against the threshold on every iteration of the while loop
            value = GetRawDigitalReadout(sensor_pin, pool_size);

            // If the delay timer exceeds timeout value, interrupts and returns kWaitForDigitalThresholdTimeout (16),
            // which is a special timeout-quit code
            if (execution_parameters.delay_timer > timeout)
            {
                return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdTimeout);
            }
        }

        // Returns kWaitForDigitalThresholdPass (15) to indicate that the check has passed
        return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdPass);
    }

    // Otherwise, if the caller function is executed in non-blocking mode, determines what code to return
    else
    {
        // If the delay timer exceeds timeout, returns kWaitForDigitalThresholdTimeout (16) to indicate timeout
        // condition
        if (execution_parameters.delay_timer > timeout)
            return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdTimeout);

        // Otherwise, if check passed, returns kWaitForDigitalThresholdPass (15)
        // Note, factors in the minimum delay to ensure it is properly assessed when the method runs in non-blocking
        // mode
        else if (value == threshold && execution_parameters.delay_timer >= min_delay)
            return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdPass);

        // Otherwise, returns kWaitForDigitalThresholdFailure (14)
        else return static_cast<uint8_t>(kReturnedUtilityCodes::kWaitForDigitalThresholdFailure);
    }
}

// Increments the stage field of the local ExecutionControlParameters structure by one.
void AMCModule::AdvanceActiveCommandStage()
{
    execution_parameters.stage++;
}

// Returns the stage field value of the local ExecutionControlParameters structure.
uint8_t AMCModule::GetActiveCommandStage()
{
    // If there is an actively executed command, returns its stage
    if (execution_parameters.command != 0)
    {
        return execution_parameters.stage;
    }
    // Otherwise returns 0 to indicate there is no actively running command
    else
    {
        return 0;
    }
}

// Resets the command and stage tracker in preparation for executing the next command. Has to be called at the end of
// every command to support proper command completion and runtime flow.
void AMCModule::CompleteCommand()
{
    execution_parameters.command = 0;
    execution_parameters.stage   = 0;
}

// Retrieves and returns the active command byte-code to caller.
uint8_t AMCModule::GetActiveCommand()
{
    return execution_parameters.command;
}

// If the input digital pin is not locked, uses digitalWrite to set it to the input value. Otherwise, sets it to LOW.
bool AMCModule::DigitalWrite(const uint8_t pin, const bool value, const bool ttl_pin)
{
    // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to LOW and returns
    // false to indicate that the pin is currently set to low
    if ((ttl_pin && runtime_parameters.ttl_lock) || (!ttl_pin && runtime_parameters.action_lock))
    {
        digitalWriteFast(pin, LOW);
        return false;
    }

    // If the pin is not locked, sets it to the appropriate value and returns the value to indicate that the pin has
    // been set to the target value
    digitalWriteFast(pin, value);
    return value;
}

// If the input analog pin is not locked, uses analogWrite to set it to the specified duty-cycle value. Otherwise, sets
// it to 0 (always off).
uint8_t AMCModule::AnalogWrite(const uint8_t pin, const uint8_t value, const bool ttl_pin)
{
    // If the appropriate lock parameter for the pin is enabled, ensures that the pin is set to 0 (permanently off) and
    // returns 0 to indicate that the pin is currently switched off (locked)
    if ((ttl_pin && runtime_parameters.ttl_lock) || (!ttl_pin && runtime_parameters.action_lock))
    {
        analogWrite(pin, 0);
        return 0;
    }

    // If the pin is not locked, sets it to the appropriate value and returns the value to indicate that the pin has
    // been set to the target value
    analogWrite(pin, value);
    return value;
}

// Integration methods

// Sets the ExecutionControlParameters structure field specified by the id_code (has to match a code inside the
// kExecutionControlParameterCodes enumerator) to the input value. The value is cast to the appropriate data type
// before overwriting the target variable. This method is used from the SetParameter general interface method, which is
// in turn, used by the Kernel.
bool AMCModule::SetAddressableExecutionParameter(
    const uint8_t id_code,  // The ID of the parameter to overwrite
    const uint32_t value    // The value to overwrite the parameter with
)
{
    // Selects and overwrites the appropriate execution parameter, based on the input id_code
    switch (static_cast<kExecutionControlParameterCodes>(id_code))
    {
        // Noblock mode toggle
        case kExecutionControlParameterCodes::kNextNoblock:
            execution_parameters.next_noblock = static_cast<bool>(value);
            return true;

        // Recurrent execution mode toggle
        case kExecutionControlParameterCodes::kRunRecurrently:
            execution_parameters.run_recurrently = static_cast<bool>(value);
            return true;

        // Recurrent delay time (microseconds)
        case kExecutionControlParameterCodes::kRecurrentDelay:
            execution_parameters.recurrent_delay = static_cast<uint32_t>(value);
            return true;

        // If the input code did not match any of the (addressable) execution parameter byte-codes, returns 'false'
        // to indicate that value assignment has failed
        default: return false;
    }
}

// A unification method that allows to handle overwriting Core execution parameter and Module custom parameters with
// a single function call. Depends on the custom implementation of the virtual method SetCustomParameter to work. Note,
// assumes that the target byte-codes for the execution parameters never overlap with the target byte-codes for the
// custom parameters
bool AMCModule::SetParameter(const uint8_t id_code, const uint32_t value)
{
    bool success;  // Tracks the assignment status

    if (id_code < 11)
    {
        // If the id code is below 11, this indicates that the targeted parameter is se Execution parameter (only these
        // parameters can be addressed using system-reserved codes).
        success = SetAddressableExecutionParameter(id_code, value);
    }
    else
    {
        // Otherwise, uses the developer-provided virtual interface to override the requested custom parameter with the
        // input value
        success = SetCustomParameter(id_code, value);
    }

    // Handles the parameter setting result status. If parameter writing fails, sends an error message to the PC and
    // returns 'false' so that the AMCKernel method that called this method can resolve its runtime too.
    if (!success)
    {
        // Sets the module_status appropriately
        module_status = static_cast<uint8_t>(kCoreModuleStatusCodes::kInvalidModuleParameterIDError);

        // Error message to indicate the method was not able to match the input id_code to any known code. Uses
        // NoCommand code to indicate that the message originates from a non-command method.
        const uint8_t error_values[1] = {id_code};
        communication_port.SendErrorMessage(
            module_id,
            system_id,
            static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
            module_status,
            error_values
        );
        return false;
    }
    // Otherwise, sends the success message to the PC to communicate the new value of the overwritten parameter.
    else
    {
        // Sets the module_status appropriately
        module_status = static_cast<uint8_t>(kCoreModuleStatusCodes::kModuleParameterSet);

        // Success message to indicate that the parameter targeted by the ID code has been set to a new value (also
        // includes the value). Uses NoCommand code to indicate that the message originates from a non-command method.
        communication_port.CreateEventHeader(
            module_id,
            system_id,
            static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kNoCommand),
            module_status
        );
        communication_port.PackValue(id_code, value);
        communication_port.SendData();
        return true;
    }
}

// An abstraction wrapper around an assignment to the next_command field of execution_parameter structure. Used by
// AMCKernel class to queue incoming Module commands for execution.
void AMCModule::QueueCommand(const uint8_t command)
{
    execution_parameters.next_command = command;

    // Sets the module status appropriately
    module_status = static_cast<uint8_t>(kCoreModuleStatusCodes::kModuleCommandQueued);
}

// If no active command is currently running, either sets a new command as active or re-runs an already executed
// command from the buffer if a sufficient number of microseconds has passed since the last command activation. The
// specific action of the method is determined by the value of next_command field, and the value of run_recurrently
// filed of the execution_parameters structure instance.
bool AMCModule::SetActiveCommand()
{
    // If the command field is not 0, this means there is already an active command being executed and no further action
    // is needed (returns false to indicate no new command was set). Placing this check at the top of the loop
    // minimizes delays when there is an active command in-progress.
    if (execution_parameters.command != 0) return false;

    // If the new_command flag is set to true and there is no active command (checked above), immediately activates the
    // next_command
    if (execution_parameters.new_command)
    {
        // Transfers the command and the noblock flag from buffer fields to active fields
        execution_parameters.command = execution_parameters.next_command;
        execution_parameters.noblock = execution_parameters.next_noblock;

        // Sets active command stage to 1, which is a secondary activation mechanism. All multi-stage functions start
        // with stage 1 and will be deadlocked if the stage is kept at 0 (default reset state)
        execution_parameters.stage = 1;

        // Removes the new_command flag to indicate that the new command has been consumed
        execution_parameters.new_command = false;

        // Resets recurrent timer to 0 whenever a command is activated
        execution_parameters.recurrent_timer = 0;

        // Returns 'true' to indicate that a new command was activated
        return true;
    }

    // There is no good reason for extracting this into a variable other than to make the 'if' check below not exceed
    // the column-width boundaries. Otherwise, the auto-formatter unwinds it to be beyond limits. So purely a cosmetic
    // step.
    bool recurrent = execution_parameters.run_recurrently;

    // If no new command is available, but recurrent activation is enabled, re-queues a command (if any is available)
    // from the buffer, if recurrent_delay of microseconds has passed since the last reset of the recurrent_timer
    if (recurrent && execution_parameters.recurrent_timer > execution_parameters.recurrent_delay)
    {
        // Repeats the activation steps from above, minus the new_command flag modification, if next_command is not 0
        if (execution_parameters.next_command != 0)
        {
            execution_parameters.command         = execution_parameters.next_command;
            execution_parameters.noblock         = execution_parameters.next_noblock;
            execution_parameters.stage           = 1;
            execution_parameters.recurrent_timer = 0;
            return true;
        }
        else return false;  // If next_command buffer is 0, returns false to indicate there is no command to set
    }

    // Returns 'false' if to indicate no command was set if the case is not handled by the conditionals above
    return false;
}

// Resets the state of the ExecutionControlParameters structures back to the original (compile-default) values.
// This is primarily used on teensies as they do not automatically reset on UART cycling, so a custom reset protocol had
// to be implemented.
void AMCModule::ResetExecutionParameters()
{
    // Sets execution_parameters to a default instance of ExecutionControlParameters structure
    execution_parameters = ExecutionControlParameters();
}
