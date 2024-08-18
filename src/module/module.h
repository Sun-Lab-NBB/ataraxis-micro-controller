/**
 * @file
 * @brief The header file for the Core AMCModule Core class, which is used as a parent for all custom Module classes to
 * automatically integrate them with the AMCKernel runtime control system.
 *
 * @subsection description Description:
 *
 * @note Every custom Module class should inherit from this class. This class is used to provide a static
 * interface that AMCKernel can use to integrate any class with the rest of the runtime code. Additionally, it shares
 * a number of utility functions that can be used to simplify custom Module development via the use of safe,
 * well-integrated methods that abstract away from low-level functionality like direct pin manipulation or time / sensor
 * threshold waiting.
 *
 * @attention It is highly advised to check one of the test_classes available through AMCKernel library (@em not
 * amc_kernel file!). These classes showcase the principles of constructing custom classes using the available base
 * class methods. Furthermore, it may be beneficial to check the test_AMC_code.cpp file to see the expected and
 * error-triggering class usage patters.
 *
 * This file contains the following items:
 * - The definition of the AMCModule base class with three broad kinds of methods. Utility methods which are designed to
 * be used by the custom commands of children classes to simplify command code writing. Integration methods that are
 * reserved for AMCKernel class usage, as they provide the static interface through which AMCKernel can use static
 * Core resources implemented inside the AMCModule class and inherited by all children classes. Virtual methods, which
 * are used for the same purpose as Integration methods, but provide an interface for the AMCKernel class to access
 * and manage custom commands and assets individually written by each Module-level class developer for that specific
 * Module.
 * - The definition of the kExecutionRuntimeParameters structure, which is a runtime-critical Core structure used to
 * queue and execute commands. It supports running commands in blocking and non-blocking modes and executing them
 * instantaneously (once) or recurrently. That structure should only be modified by AMCKernel developers, if ever and
 * any call to the structure has been abstracted using utility method, so module developers can use utility methods
 * without worrying about the functioning of the core structure.
 * - The definition of the kExecutionRuntimeParameterCodes enumeration, which stores the id_codes for the PC-addressable
 * fields of the kExecutionRuntimeParameters structure. This is a critical enumeration that is used to control the
 * proper modification of PC-addressable parameters in the kExecutionRuntimeParameters structure. Again, this structure
 * should only be modified by Kernel developers.
 * - kReturnedUtilityCodes, an enumeration that lists all byte-codes returned by Utility methods, so that custom
 * commands can interpret and handle each returned code appropriately. This provides an alternative interface to using
 * 'magic numbers', improving ht maintainability fo the codebase.
 * - Other internal trackers and variables used to integrate any class that inherits from the AMCModule base class with
 * the AMCKernel runtime control system.
 *
 * @subsection developer_notes Developer Notes:
 * This is one of the key Core level classes that is critically important for the functioning of the whole AMC codebase.
 * Generally, only AMC Kernel developers with a good grasp of the codebase should be modifying this class and any such
 * modifications should come with ample deprecation warnings and versioning. This is especially relevant for class
 * versions that modify existing functionality and, as such, are likely to be incompatible with existent custom Modules.
 *
 * There are two main ways of using this class to develop custom Modules. The preferred way is to rely on the available
 * utility methods provided by this class to abstract all interactions with the inherited Core methods and variables.
 * Specifically, any class inheriting from this base class should use the methods in the utility sections inside all
 * custom commands and overridden virtual methods where appropriate. The current set of utility methods should be
 * sufficient to support most standard microcontroller use cases, while providing automatic compatibility with the rest
 * of the AMC codebase. As a bonus, this method is likely to improve code maintainability, as utility methods are
 * guaranteed to be supported and maintained even if certain Core elements in the class are modified by Kernel library
 * developers.
 *
 * The second way, which may be useful if specific required functionality is not available through the utility methods,
 * is to directly use the included core structures and variables, such as the ExecutionControlParameters. This requires
 * a good understanding of how the base class and it's method function, as well as an understanding of how AMCKernel
 * works and interacts with AMCModule-derived classes. With sufficient care, this method cab provide more control over
 * the behavior of any custom Module, at the expense of being more verbose and less safe.
 *
 * Regardless of the use method, any custom Module inheriting from the AMCModule class has to implement the following
 * purely virtual methods to become fully compatible with AMCKernel-mediated runtime control system:
 * - SetCustomParameters: For the kernel to properly write any Parameter structures received from the PC to the target
 * custom (class-specific) parameter structures or variables.
 * - RunActiveCommand: For the module to actually execute the custom command using the unique command byte-code stored
 * in the 'command' field of the ExecutionControlParameters structure (active command).
 * - SetupModule: For the module to be able to properly set up its pins, default values and otherwise prepare for the
 * runtime cycle.
 * ResetCustomAssets: For the kernel to properly reset all custom (class-specific) assets, such as parameters, variables
 * and states, before starting a new runtime cycle.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - digitalWriteFast.h for fast digital pin manipulation methods.
 * - serial_pc_communication.h for SerialPCCommunication class to support sending data to the PC.
 * - shared_assets.h for globally shared static message byte-codes and the ControllerRuntimeParameter structure.
 * - stdbool.h for standard boolean types. Using stdbool.h instead of cstdbool for compatibility (with Arduino) reasons.
 * - stdint.h for fixed-width integer types. Using stdint.h instead of cstdint for compatibility (with Arduino) reasons.
 *
 * @see amc_module.cpp for method implementation.
 */

#ifndef AMC_MODULE_H
#define AMC_MODULE_H

#include "Arduino.h"
#include "axmc_shared_assets.h"
#include "communication.h"
#include "digitalWriteFast.h"
#include "elapsedMillis.h"
#include "stdbool.h"
#include "stdint.h"

/**
 * @brief A major Core-level class that serves as the parent for all Module- level classes and provides them with a set
 * of core utilities and methods necessary to automatically integrate with the rest of the AMC codebase.
 *
 * This class serves as the shared (via inheritance) repository of utility and runtime-control methods that allow to
 * automatically integrate any custom Module into the existing AMC codebase. Specifically, by inheriting from this
 * class, any Module automatically gains access to the methods that queue and execute Module commands based on the
 * input from the PC. Additionally, this allows the AMCKernel to interact with any custom module using the virtual and
 * non-virtual methods inherited from the AMCModule class, effectively embedding any correctly constructed module into
 * the AMC runtime flow control structure.
 *
 * @note This class offers a collection of utility methods that should preferentially be used when writing custom
 * functions. These method abstract the interactions with Core structures that are used to enable many of the distinct
 * AMC codebase functions such as concurrent (non-blocking) execution of many different commands at the same time and
 * error handling. Be sure to check the documentation for all Utility (in contrast to Integration or Virtual) and the
 * examples available through test classes of AMCKernel library prior to writing custom functions.
 *
 * @warning Every Module class @b has to inherit from this base class to be compatible with the rest of the AMC
 * codebase. Moreover, the base class itself uses pure virtual methods and, as such, cannot be instantiated. Only a
 * child class that properly overrides all pure virtual methods of the base class can be instantiated.
 */
class AMCModule
{
  public:
    /**
     * @struct ExecutionControlParameters
     * @brief Stores parameters that are used to dynamically queue, execute and control the execution flow of commands
     * for each Module-level class instance.
     *
     * An instance of this structure is used by the base AMCModule Core class instance from which all custom modules are
     * expected to inherit. All runtime control manipulations that involve changing the variables inside this structure
     * should be carried out via the methods available through the base AMCModule Core class via inheritance.
     *
     * @attention Generally, any modification to this structure or code that makes use of this structure should be
     * reserved for developers with a good grasp of the existing codebase and, if possible, avoided (especially
     * deletions or refactoring of existing variables). More specifically, this should be handed off to AMC project
     * kernel developers when possible.
     */
    struct ExecutionControlParameters
    {
        uint8_t command               = 0;      ///< Currently active (in-progress) command
        uint8_t stage                 = 0;      ///< Stage of the currently active command
        bool noblock                  = false;  ///< Specifies if the currently active command is blocking
        uint8_t next_command          = 0;      ///< A buffer that allows to queue the next command to be executed
        bool next_noblock             = false;  ///< A buffer to store the next noblock flag value
        bool new_command              = 0;      ///< Tracks whether next_command is a new or recurrent command
        bool run_recurrently          = false;  ///< Specifies whether the queued command runs once and clears or recurs
        uint32_t recurrent_delay      = 0;      ///< The minimum amount of microseconds between recurrent command calls
        elapsedMicros recurrent_timer = 0;      ///< A timer class instance to time recurrent command activation delays
        elapsedMicros delay_timer     = 0;      ///< A timer class instance to time delays between active command stages
    } execution_parameters;

    // Stores the most recent module status code. This variable is used to communicate the runtime status of the Module,
    // which is used by the Kernel to more effectively communicate any Module errors to the PC and to interface with
    // the Module.
    uint8_t module_status;

    /**
     * @brief Instantiates a new AMCModule Core class objects.
     *
     * @param module_type_id This ID is used to identify module types (classes) during data communication. Each class
     * has a custom set of parameters and commands, but is very likely to use the same parameter and command byte-codes
     * as other classes. Module type IDs solve the issue that would otherwise arise when the same byte-codes are used by
     * different classes with radically different implications, by ensuring any communicated byte-codes are interpreted
     * according to class-specific code enumerations. The IDs are assigned dynamically by the user when configuring the
     * Controller (and PC, they have to match!) runtime layout. This allows to solve the problem that would eventually
     * arise for custom modules, where different developers might have reused the same IDs for different modules.
     * @param module_instance_id This ID is used to identify the specific instance of the module_type_id-specified
     * class. This allows to address specific class instances, which are often mapped to particular physical system.
     * Eg: If module_type_id of 1 corresponds to the 'Door' class, module_instance_id of 1 would mean "Kitchen Door" and
     * 2 would mean "Bathroom Door".
     * @param controller_runtime_parameters A constant reference to the ControllerRuntimeParameters structure instance
     * provided by the AMCKernel class. There should only be one such instance and the only class allowed to modify that
     * instance is the AMCKernel class itself. All Module classes take this instance by reference as runtime parameters
     * control many aspects of Module runtime behavior.
     * @param serial_communication_port A reference to the SerialPCCommunication class instance shared by all AMC
     * codebase classes that receive (AMCKernel only!) or send (AMCKernel and all Modules) data over the serial port.
     * Modules use this class instance to send event (and error) data over to the PC.
     */
    AMCModule(
        uint8_t module_type_id,
        uint8_t module_instance_id,
        const axmc_shared_assets::ControllerRuntimeParameters& controller_runtime_parameters,
        SerialPCCommunication& serial_communication_port
    );

    // Declares non-virtual utility methods. These methods should not be overwritten by derived classes, and they are
    // primarily designed to help developers write custom module commands by providing standardized high-level access to
    // the underlying Core structures derived from the AMCModule class.

    /**
     * @brief Generates a boolean value via pseudorandom sampling or predefined value rescaling.
     *
     * The method uses the kSimulatedValue from referenced ControllerRuntimeParameters structure instance as the seed
     * rescaled to fit into the boolean range (via ceiling rounding) if exact value simulation is enabled. Otherwise,
     * it uses random() function to generate the value pseudorandomly. Primarily, this function is designed for
     * controller logic testing, but it can be used as a pseudorandom generator if required.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @returns bool @b true or @b false, depending on the outcome of value generation.
     */
    bool GenerateBool();

    /**
     * @brief Generates a uint8_t value via pseudorandom sampling or predefined value rescaling.
     *
     * The function uses the kSimulatedValue from referenced ControllerRuntimeParameters structure instance as the seed
     * rescaled to fit into the uint8_t range (via ceiling rounding) if exact value simulation is enabled. Otherwise,
     * it uses random() function to generate the value pseudorandomly. Primarily, this function is designed for
     * controller logic testing, but it can be used as a pseudorandom generator if required.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @returns uint8_t between 0 and 255, depending on the outcome of value generation.
     */
    uint8_t GenerateByte();

    /**
     * @brief Generates a uint16_t value via pseudorandom sampling or predefined value rescaling.
     *
     * The function uses the kSimulatedValue from referenced ControllerRuntimeParameters structure instance as the seed
     * rescaled to fit into the uint16_t range (via ceiling rounding) if exact value simulation is enabled. Otherwise,
     * it uses random() function to generate the value pseudorandomly. Primarily, this function is designed for
     * controller logic testing, but it can be used as a pseudorandom generator if required.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @returns uint16_t between 0 and 65535, depending on the outcome of value generation.
     */
    uint16_t GenerateUnsignedShort();

    /**
     * @brief Generates a uint32_t value via pseudorandom sampling or predefined value rescaling.
     *
     * The function uses the kSimulatedValue from referenced ControllerRuntimeParameters structure instance as the seed
     * rescaled to fit into the boolean range (via ceiling rounding) if exact value simulation is enabled. Otherwise,
     * it uses random() function to generate the value pseudorandomly. Primarily, this function is designed for
     * controller logic testing, but it can be used as a pseudorandom generator if required.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @returns uint32_t between 0 and 4294967295, depending on the outcome of value generation.
     */
    uint32_t GenerateUnsignedLong();

    /**
     * @brief Polls and (optionally) averages the value(s) of the requested analog sensor pin and returns the resultant
     * raw analog readout value to the caller.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @note This method should be used for all analog pin reading tasks as it automatically supports the use of
     * value simulation subroutines, which is considered a standard property for all AMC sensors.
     *
     * @param pin The number of the Analog pin to be polled (in simulation mode will be ignored internally).
     * @param pool_size The number of pin readout values to average to produce the final readout (also ignored in
     * simulation mode). Setting to 0 or 1 means no averaging is performed, 2+ means averaging is performed.
     *
     * @returns uint16_t value of the analog sensor. Currently uses 16-bit resolution as the maximum supported by analog
     * pin hardware.
     */
    uint16_t GetRawAnalogReadout(const uint8_t pin, const uint16_t pool_size);

    /**
     * @brief Polls and (optionally) averages the value(s) of the requested digital sensor pin and returns the resultant
     * pin state (HIGH or LOW) to caller.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @note This method should be used for all digital pin reading tasks as it automatically supports the use of
     * value simulation subroutines, which is considered a standard property for all AMC sensors.
     *
     * @param pin The number of the Digital pin to be polled (in simulation mode will be ignored internally).
     * @param pool_size The number of pin readout values to average to produce the final readout (also ignored in
     * simulation mode). Setting to 0 or 1 means no averaging is performed, 2+ means averaging is performed.
     *
     * @returns bool @b true (HIGH) or @b false (LOW), depending on the state of the input pin.
     */
    bool GetRawDigitalReadout(const uint8_t pin, const uint16_t pool_size);

    /**
     * @brief Checks if the delay_duration of microseconds has passed since the module's ExecutionControlParameters
     * structure's delay_timer field has been reset.
     *
     * Depending on execution configuration, the function can block in-place until the escape duration has passed or be
     * used as a simple check of whether the required duration of microseconds has passed.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @note It is encouraged to use this method for all delay-related tasks (and implement a noblock method design to
     * support concurrent task execution if required). This is because this method properly handles the use of value
     * simulation subroutines, which are considered a standard property for all AMC delay methods.
     *
     * @param delay_duration The duration, in @em microseconds the function should delay / check for.
     *
     * @returns bool @b true if the delay has been successfully enforced (either via blocking or checking) and @b false
     * otherwise. If the elapsed duration equals to the delay, this is considered a passing condition.
     */
    bool WaitForMicros(const uint32_t delay_duration);

    /**
     * @brief Checks if the evaluated @b Analog sensor pin detects a value that is equal to or exceeds the trigger
     * threshold.
     *
     * @note Trigger threshold is specified using a concrete threshold value and the comparison is carried out either
     * using the >= (default) or <= operator. As such, the value can exceed the threshold by being greater than or equal
     * to the threshold or less than or equal to the threshold, which is specified via the invert_condition flag.
     *
     * Depending on configuration, the function can block in-place until the escape condition is met or be used as a
     * simple check of whether the condition is met.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @note It is encouraged to use this method for all analog sensor-threshold related tasks (and implement a noblock
     * method design to support concurrent task execution if required). This is because this method properly handles the
     * use of value simulation subroutines, which are considered a standard property for all AMC sensor
     * threshold-related methods.
     *
     * @attention Currently only supports open-ended threshold. The rationale here is that even with the best analog
     * resolution analog sensors will fluctuate a little and having a concrete (==) threshold comparison would very
     * likely not work as intended. Having a one-side open-ended threshold offers the best of both worlds, where the
     * value still has to be within threshold boundaries, but also can be fluctuate above or below it and still
     * register.
     *
     * @param sensor_pin The number of the analog pin whose' readout value will be evaluated against the threshold.
     * @param invert_condition A boolean flag that determines whether the default comparison operator (>=, false) is
     * inverted to (<=, true).
     * @param threshold The concrete threshold value against which the parameter will be evaluated. Regardless of the
     * operator, if pin readout equals to threshold, the function will return @b true.
     * @param timeout The number of microseconds after the function breaks and returns kWaitForAnalogThresholdTimeout,
     * this is necessary to prevent soft-blocking.
     * @param pool_size The number of sensor readouts to average into the final value.
     *
     * @returns uint8_t kWaitForAnalogThresholdFailure byte-code if the condition was not met and
     * kWaitForAnalogThresholdPass if it was met. Returns kWaitForAnalogThresholdTimeout if timeout duration has
     * been exceeded (to avoid soft-blocking). All byte-codes should be interpreted using the kReturnedUtilityCodes
     * enumeration.
     */
    uint8_t WaitForAnalogThreshold(
        const uint8_t sensor_pin,
        const bool invert_condition,
        const uint16_t threshold,
        const uint32_t timeout,
        const uint16_t pool_size
    );

    /**
     * @brief Checks if the evaluated @b Digital sensor pin detects a value that matches the trigger threshold.
     *
     * This is an equivalent of the WaitForAnalogThreshold with considerably streamlined logic due to the binary nature
     * of digital pin threshold checking.
     *
     * @note Unlike WaitForAnalogThreshold() method, this method uses exact thresholds. This is because it works with
     * digital sensors that generally do not fluctuate (bouncing noise aside) and, therefore, are a lot more reliable
     * for using them with a concrete threshold.
     *
     * Depending on configuration, the function can block in-place until the escape condition is met or be used as a
     * simple check of whether the condition is met.
     *
     * This is a non-virtual utility method that is available to all derived classes.
     *
     * @note It is encouraged to use this method for all digital sensor-threshold related tasks (and implement a noblock
     * method design to support concurrent task execution if required). This is because this method properly handles the
     * use of value simulation subroutines, which are considered a standard property for all AMC sensor
     * threshold-related methods.
     *
     * @param sensor_pin The number of the pin whose' readout value will be evaluated against the threshold.
     * @param threshold A boolean @b true or @b false which is used as a threshold for the pin value. Passing condition
     * is evaluated using == operator, so the readout value has to match the threshold for the function to return @b
     * true.
     * @param timeout The number of microseconds after the function breaks and returns kWaitForDigitalThresholdTimeout,
     * this is necessary to prevent soft-blocking.
     * @param pool_size The number of sensor readouts to average into the final value. While not strictly necessary for
     * digital pins, the option is maintained to allow for custom debouncing logic (which is often conducted through
     * averaging).
     *
     * @returns uint8_t kWaitForDigitalThresholdFailure byte-code if the condition was not met and
     * kWaitForDigitalThresholdPass if it was met. Returns kWaitForDigitalThresholdTimeout if timeout duration has
     * been exceeded (to avoid soft-blocking). All byte-codes should be interpreted using the kReturnedUtilityCodes
     * enumeration.
     */
    uint8_t WaitForDigitalThreshold(
        const uint8_t sensor_pin,
        const bool threshold,
        const uint32_t timeout,
        const uint16_t pool_size
    );

    /**
     * @brief Advances the stage of the currently running Module command.
     *
     * This is a simple wrapper interface that helps writing noblock-compatible commands by modifying the stage tracker
     * inside the ExecutionControlParameters structure by 1 each time this method is called. Use it to properly advance
     * noblock execution command stages
     *
     * @note See one of the test_class_1 from AMCKernel library for examples.
     */
    void AdvanceActiveCommandStage();

    /**
     * @brief Extracts and returns the stage of the currently running Module command.
     *
     * This is a simple wrapper interface that helps writing noblock-compatible commands by providing convenient access
     * to the stage tracker field of the ExecutionControlParameters structure. Use it to extract and evaluate the stage
     * number to execute the proper portion of each noblock-compatible command.
     *
     * @returns uint8_t The stage number of the currently active command. The returned number is necessarily greater
     * than 0 for any active command and 0 if there is currently noa active command.
     */
    uint8_t GetActiveCommandStage();

    /**
     * @brief Sets the command and stage fields of the ExecutionControlParameters structure to 0, indicating that the
     * command has been completed.
     *
     * @warning It is essential that this method is called at the end of every command execution to allow the Module to
     * execute other commands. Failure to do so can completely deadlock the Module and, if command is blocking, the
     * whole Controller.
     *
     * @note The 'end' here means the algorithmic end: when the command completes everything it sets out to do. For
     * noblock commands, the Controller may need to loop through the command code multiple times, so this methods call
     * should be protected by an 'if' statement that ensures it is only called when the command has finished it's work.
     */
    void CompleteCommand();

    /**
     * @brief Extracts and returns the byte-code of the currently active command stored in ExecutionControlParameters
     * structure.
     *
     * This is a simple wrapper interface primarily designed to assist writing RunActiveCommand() virtual method
     * implementation by providing easy access to the command-code that needs to be verified and executed by that
     * method.
     *
     * @returns uint8_t The byte-code of the currently active command. The returned number is necessarily greater than
     * 0 if a command is active and 0 if there is currently no active command.
     */
    uint8_t GetActiveCommand();

    /**
     * @brief Sets the input digital pin to the specified value (High or Low).
     *
     * This utility method is used to control the value of a digital pin configured to output a constant High or Low
     * level signal. Internally, the method handles the correct use of global runtime parameters that control whether
     * action and ttl pin activity is allowed.
     *
     * @note It is recommend to use this method for all digital pin control takes.
     *
     * @param pin The value of the digital pin to be set to the input value.
     * @param value The boolean value to set the pin to.
     * @param ttl_pin The boolean flag that determines whether the pin is used to drive active systems or for TTL
     * communication. This is necessary to properly comply with action_lock and ttl_lock field settings of the global
     * ControllerRuntimeParameters structure.
     *
     * @returns bool Current value of the pin (the value the pin has actually been set to).
     */
    bool DigitalWrite(const uint8_t pin, const bool value, const bool ttl_pin = false);

    /**
     * @brief Sets the input analog pin to the specified duty-cycle value (from 0 to 255).
     *
     * This utility method is used to control the value of aan analog pin configured to output a PWM-pulse with the
     * defined duty cycle from 0 (off) to 255 (always on). Internally, the method handles the correct use of global
     * runtime parameters that control whether action and ttl pin activity is allowed.
     *
     * @note Currently, the method only supports standard 8-bit resolution to maintain backward-compatibility with AVR
     * boards. In the future, this may be adjusted.
     *
     * @param pin The value of the analog pin to be set to the input value.
     * @param value A uint8_t duty cycle value from 0 to 255 that controls the proportion of time during which the pin
     * is On. Note, the exact meaning of each duty-cycle step depends on the clock that is used to control the analog
     * pin cycling behavior.
     * @param ttl_pin The boolean flag that determines whether the pin is used to drive active systems or for TTL
     * communication. This is necessary to properly comply with action_lock and ttl_lock field settings of the global
     * ControllerRuntimeParameters structure.
     *
     * @returns uint8_t Current value (duty cycle) of the pin (the value the pin has actually been set to).
     */
    uint8_t AnalogWrite(const uint8_t pin, const uint8_t value, const bool ttl_pin = false);

    // Declares integration methods and variables to be used by AMCKernel class to manage every class instance that
    // inherits from the AMCModule class (In other words: every module class instance).

    /// The ID byte-code for the class (module type) to be shared by all class instances. Used as the 'module' field
    /// of the incoming and outgoing data payload header structures. Intended to be used by the AMCKernel class to
    /// properly address incoming commands and internally to properly address the outgoing event data. This ID is
    /// crucial as it changes the interpretation of the rest of the byte-codes used during communication.
    uint8_t module_id = 0;

    /// The ID byte-code for the specific instance of the class (module_type). Used as the 'system' field of the
    /// incoming and outgoing data payload header structures. Intended to be used by the AMCKernel class to
    /// properly address incoming commands and internally to properly address the outgoing event data. This ID is
    /// dependent on the module_id and has to be unique for every class instance (every instance that shares the same
    /// module_id).
    uint8_t system_id = 0;

    /**
     * @brief Saves the input command to the appropriate field inside the local ExecutionControlParameters structure, so
     * that it can be executed during as soon as the Module is done with the currently running command (if any).
     *
     * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
     * method to queue Module commands for execution when they are received from the PC. This method is primarily
     * designed to abstract away direct structure manipulation.
     *
     * @attention This method does not check whether the input command code is valid, it simply saves it to the
     * appropriate field. The validity check is carried out by the virtual RunActiveCommand() method.
     *
     * @param command The id byte-code for the command to execute.
     */
    void QueueCommand(const uint8_t command);

    /**
     * @brief Overwrites the value specified by the input id_code of the PC-addressable field inside the module-specific
     * ExecutionControlParameters structure instance with the input value.
     *
     * Casts the input value to the appropriate type before modifying the value. Can only target PC-addressable fields
     * of the structure, as not fields are by-design PC-addressable.
     *
     * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
     * method to set execution parameters when they are received from the PC. This method is intended to be used via the
     * SetParameter() method of this class to properly unify Custom and non-Custom parameter setting logic (and error /
     * success handling).
     *
     * @param id_code The unique byte-code for the specific field inside the ExecutionControlParameters structure to be
     * modified. These codes are defined in the kExecutionControlParameterCodes private enumeration of this class and
     * the id-code of the parameter structure has to match one of these codes to properly target an execution parameter
     * value.
     * @param value The specific value to set the target parameter variable to. Uses 32-bit format to support the entire
     * range of possible input values.
     *
     * @returns bool @b true if the value was successfully set, @b false otherwise. The only realistic reason for
     * returning false is if the input id_code does not match any of the id_codes available through the
     * kExecutionControlParameterCodes enumeration. SetParameter() class method uses the returned values to determine
     * whether to issue a success or failure message to the PC and handles PC messaging.
     */
    bool SetAddressableExecutionParameter(const uint8_t id_code, const uint32_t value);

    /**
     * @brief An integration method that allows the AMCKernel to handle parameter assignments regardless of whether they
     * are intended for the kExecutionControlParameters or a Custom parameters structure.
     *
     * @note This is a non-virtual method that is used by the AMCKernel to assign new parameter values received from
     * PC to appropriate parameter structure fields. This method handles generating the necessary success or error
     * messages that are sent to the PC to indicate method performance.
     *
     * This method is called both when setting kExecutionControlParameters and any custom parameters. The method uses
     * the same success and failure codes for 'System' execution parameter and custom parameters, derived from the
     * kCoreModuleStatusCodes structure. This allows to seamlessly unify Core and Custom parameter setting status and
     * handled them identically regardless of the source on the PC side.
     *
     * @attention The only method that every developer should write themselves for this entire aggregation of methods
     * (SetParameter, SetAddressableExecutionParameter and SetCustomParameter) to function as expected is the
     * SetCustomParameter(), which overwrites custom parameters. The rest of the structure would integrate
     * automatically, provided that the SetCustomParameter() is written correctly.
     *
     * @param id_code The byte-code that points to a specific parameter to be overwritten inside either the Core
     * kExecutionControlParameters or any Custom parameters structure of the Module.
     * @param value The new value to set the target parameter to.
     *
     * @returns bool @b true if the method succeeds and @b false otherwise. Handles any required PC messaging in-place and
     * only returns the status to assist the caller AMCKernel method in deciding how to proceed with its own runtime.
     */
    bool SetParameter(const uint8_t id_code, const uint32_t value);

    /**
     * @brief If the module is not already executing a command, sets a new command to be executed.
     *
     * Specifically, first checks whether the module is currently executing a command. If the module is not executing a
     * command and a new command is available, transfers command data from queue (buffer) into the active directory. If
     * no new command is available, but run_recursively ExecutionControlParameters flag is @b true, transfers (the same
     * command as presumably has just finished running) into the active directory to re-run it, provided that the
     * specified recurrent_delay of microseconds has passed since the last command activation.
     *
     * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
     * method to set execution parameters when they are received from the PC. It is expected that the AMCKernel handles
     * any error / success messaging for this method internally, using Kernel-specific event codes where necessary.
     *
     * @returns bool @b true if a new command was successfully set and @b false otherwise.
     */
    bool SetActiveCommand();

    /**
     * @brief Resets the local instance of the ExecutionControlParameters structure to default values.
     *
     * This method is designed primarily for Teensy boards that do not reset on UART / USB cycling. As such, this method
     * is designed to reset the Controller between runtimes to support proper runtime setup and execution.
     *
     * Has no returns as there is no way this method can fail if it compiles.
     *
     * @note This is a non-virtual method that is primarily designed to be used by the AMCKernel runtime scheduler
     * method to set execution parameters when they are received from the PC. It is expected that the AMCKernel handles
     * any error / success messaging for this method internally, using Kernel-specific event codes where necessary.
     */
    void ResetExecutionParameters();

    // Declares virtual methods to be overwritten by each child class. Virtual methods are critical for integrating
    // the custom logic of each child class with the centralized runtime flow control functionality realized through the
    // AMCKernel class. Specifically, each module developer should define a custom implementation for each of these
    // methods, which AMCKernel will call to properly handle custom class-specific commands and parameters.
    // Uses pure virtual approach to enforce that each child overrides these methods with custom logic implementation.

    /**
     * @brief Sets the custom parameter specified by the input id_code to the input value. This method provides an
     * interface for the AMCKernel class to work with custom parameters of any custom class.
     *
     * Specifically, use this method to provide an interface the AMCKernel class can use to translate incoming
     * Parameter data-structures (See SerialPCCommunication implementation) into overwriting the target parameter
     * values. The exact realization (codes, parameters, how many parameter structures and how to name them, etc.) is
     * entirely up to you, as long as the interface behaves in a way identical to the SetAddressableExecutionParameter()
     * method of the AMCModule base class.
     *
     * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
     * from base AMCModule class. This method is intended to be used via the inherited SetParameter() method of this
     * class to properly unify Custom and non-Custom parameter setting logic (and error / success handling).
     *
     * @param id_code The unique byte-code that specifies the custom parameter to be modified.
     * @param value The value to write to the target parameter variable. Should be converted to the appropriate type
     * for each custom parameter.
     *
     * @returns bool @b true if the new parameter value was successfully set, @b false otherwise. SetParameter() class
     * method uses the returned values to determine whether to issue a success or failure message to the PC and handles
     * PC messaging. The only reason for receiving the 'false' response should be if the input id_code does not match
     * any of the valid id_codes used to identify specific parameter values.
     */
    virtual bool SetCustomParameter(const uint8_t id_code, const uint32_t value) = 0;

    /**
     * @brief Triggers the currently active command. This method provides the interface for the AMCKernel class to
     * activate custom commands and verifies the module actually recognizes the command.
     *
     * Specifically, use this method to allow AMCKernel to activate (physically run) custom commands specified by the
     * unique byte-code written to the active_command field of the local ExecutionControlParameters structure by the
     * non-virtual SetActiveCommand() method.
     *
     * It is highly advised to write custom commands using the utility methods available through inheritance from the
     * AMCModule class, especially the CompleteCommand() method that has to be called at the end of each command. The
     * available collection of utility methods offer a seamless integration with the core features of the AMCModule
     * class and the broader Ataraxis framework (which includes the AMC system). See one of the test classes available
     * in the AMCKernel library for an example of how to write custom functions.
     *
     * @note Use GetActiveCommand() non-virtual method in your custom implementation to determine the currently active
     * command code. 0 is not a valid active command code! The suggested use for this method is a basic switch statement
     * that executes the requested command that matches the active_command field of the ExecutionControlParameters
     * structure and returns 'false' if the command does not match any of the predefined custom command IDs.
     *
     * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
     * from base AMCModule class.
     *
     * @returns bool @b true if the command was successfully triggered, @b false otherwise. AMCKernel uses the returned
     * value of this method to determine whether to issue an error message to the PC (if command fails verification),
     * so it is important that the method properly handles verification failure cases and returns false.
     */
    virtual bool RunActiveCommand() = 0;

    /**
     * @brief Carries out all setup operations for the current module. This method provides the interface for the
     * AMCKernel class to setup all custom elements of the Module class at the beginning of each runtime.
     *
     * Specifically, use this method as you would typically use a 'setup' loop of the Arduino default code layout. Use
     * it to set pin-modes, set various local assets to default values, adjust analog resolution, etc. This method is
     * called at the beginning of each runtime, following the reset of all assets, as a way to mimic the setup()
     * behavior for Teensy boards that do not reset on USB / UART cycling.
     *
     * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
     * from base AMCModule class.
     *
     * @attention Ideally, this should not contain any logic that can fail in any way as error handling for setup
     * methods is not implemented at this time.
     */
    virtual void SetupModule() = 0;

    /**
     * @brief Resets all local assets that needs to be reset back to provided defaults. This method provides the
     * interface for the AMCKernel class to reset all active assets to simulate USB / UART cycling on Arduino boards.
     *
     * Specifically, use this method to reset any class-specific custom assets (parameter structures, variables,
     * trackers, etc.) that need to be reset between runtime cycles. On Arduino boards, this happens naturally whenever
     * the USB / UART connection is re-established, but this is not true for Teensy boards. Since it is advantageous to
     * start each runtime from a well-known default values, this method is used to reset all custom assets. All Core
     * assets have compatible methods so that they can (and are!) reset before the beginning fo each new runtime.
     *
     * @warning This is a pure-virtual method that @b has to be implemented separately for each child class derived
     * from base AMCModule class.
     */
    virtual void ResetCustomAssets() = 0;

    /**
     * @brief A pure virtual destructor method to ensure proper cleanup.
     *
     * No extra cleanup steps other than class deletion itself (which also does not happen as everything in the
     * codebase so far is static)
     */
    virtual ~AMCModule() = default;

  protected:
    /// This is a reference to the global ControllerRuntimeParameters structure instantiated and controlled by the
    /// AMCKernel class. Many methods of the base AMCModule and (expectedly) derived classes require the information
    /// from this structure to function properly. However, the only class that is allowed to modify the instance is the
    /// AMCKernel class itself. To protect the base instance from modification, this reference is provided as a
    /// constant member.
    const axmc_shared_assets::ControllerRuntimeParameters& runtime_parameters;

    /// This is a reference to the shared instance of the SerialPCCommunication class that handles the bidirectional
    /// Controller-PC communication via the serial UART or USB interface. While any data reception methods of the
    /// class are intended to be used solely by the AMCKernel class, all data transmission methods should be used
    /// in-place by the modules that need to send data to the PC. As such, this reference is not constant and any Module
    /// that inherits from the base class is allowed to use the internal methods that modify the local variables of the
    /// referenced class (this is required fro the proper functioning of the communication protocols).
    SerialPCCommunication& communication_port;

    /**
     * @enum kExecutionControlParameterCodes
     * @brief Provides the id byte-codes for the variables inside the ExecutionControlParameters structure that are
     * addressable by the PC.
     *
     * This enumeration is used to address and set specific execution parameters when the PC sends values, such as a
     * new command to execute, for any of these variables. Fields that are by design @b not PC-addressable do not have
     * an id-code assigned to them in an effort to discourage accidental overwriting. If a particular field is not
     * PC-addressable, that does not mean it is constant. Many of the local methods use non-PC-addressable fields to
     * correctly control module command execution flow.
     *
     * @warning To support correct command handling behavior by the AMCKernel class, this enumeration should @b only
     * use codes 1 through 10. Since no other enumeration uses these codes to target specific parameter values, this
     * ensures that critical execution control parameters can be precisely addressed in all Modules derived from the
     * AMCModule base class.
     *
     * @attention All variables inside this enumeration should @b not use code 0.
     */
    enum class kExecutionControlParameterCodes : uint8_t
    {
        kNextNoblock    = 1,  ///< Allows to adjust execution mode between blocking and non-blocking
        kRunRecurrently = 2,  ///< Allows to select whether the queued command runs once and clears or recurs
        kRecurrentDelay = 3,  ///< Allows to specify the minimum amount of microseconds between recurrent command calls
    };

    /**
     * @enum kReturnedUtilityCodes
     * @brief Codifies byte-codes that are returned by certain utility methods available through the base AMCModule
     * class via inheritance.
     *
     * Use the enumerators inside this enumeration when writing result-handling code for the derived class methods that
     * make use of base AMCModule class utility methods that return byte-codes to correctly interpret the resultant
     * codes. Each variable inside this enumeration is prefixed the name of the method whose' return byte-code it
     * describes.
     *
     * @attention This enumeration only covers the base class utility function return codes. You need to provide a
     * custom enumeration when writing your own classes that are derived from the base class to function as the source
     * of the status byte-codes (if you intend to support that functionality).
     *
     * @note All variables inside this enumeration should NOT use codes 0 through 10 (inclusive) and 250 through 255.
     * These codes are reserved for system-use.
     */
    enum class kReturnedUtilityCodes : uint8_t
    {
        kWaitForAnalogThresholdFailure  = 11,  ///< Analog threshold check failed
        kWaitForAnalogThresholdPass     = 12,  ///< Analog threshold check passed
        kWaitForAnalogThresholdTimeout  = 13,  ///< Analog threshold check aborted due to timeout
        kWaitForDigitalThresholdFailure = 14,  ///< Digital threshold check failed
        kWaitForDigitalThresholdPass    = 15,  ///< Digital threshold check passed
        kWaitForDigitalThresholdTimeout = 16   ///< Digital threshold check aborted due to timeout
    };

    /**
     * @enum kCoreModuleStatusCodes
     * @brief Stores byte-codes used to communicate the status of the Module runtime which is communicated to other
     * classes (AMCKernel) and to the PC.
     *
     * @attention These codes only apply to Core methods inherited from the base AMCModule class and it is expected that
     * a separate enumeration is defined and used to communicate the status codes related to custom commands and
     * methods of each class inheriting from the AMCModule class. In this case, the custom code enumeration should use
     * complimentary codes with this enumeration to avoid code overlap. Having the two enumerations overlap will likely
     * result in unexpect4ed behavior and inability of the PC / other classes to correctly process status codes.
     *
     * @note This enumerator should use codes 0 through 10 where applicable to make critical system statuses stand out
     * from the more generic codes. Only developers with a good grasp of the codebase should be changing / introducing
     * these codes as they can lead to broad runtime failures if used incorrectly.
     */
    enum class kCoreModuleStatusCodes : uint8_t
    {
        kInvalidModuleParameterIDError = 11,  ///< Unable to recognize incoming Kernel parameter id
        kModuleParameterSet            = 12,  ///< Module parameter has been successfully set to input value
        kModuleCommandQueued           = 13   ///< The next command to execute has been successfully queued
    };
};

#endif  //AMC_MODULE_H
