// KERNEL

#ifndef AMC_KERNEL_H
#define AMC_KERNEL_H

// Dependencies
#include "Arduino.h"
#include "shared_assets.h"
#include "module.h"

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
class AMCKernel
{
  public:
    /// Instantiates the ControllerRuntimeParameters structure. This structure is the source of all global
    /// dynamically-addressable Controller parameters that broadly control runtime behavior. This codebase expects
    /// that this is the only instance of this structure present in the entire codebase for safety reasons.
    axmc_shared_assets::ControllerRuntimeParameters runtime_parameters;

    /**
     * @struct kCustomKernelParameters
     * @brief Stores all parameters that are exclusively used by Kernel class commands and should not be shared with
     * Module classes via shared kControllerRuntimeParameters structure instance.
     *
     * @note This structure contains both addressable dynamic and non-addressable static parameters.
     *
     * Named similarly to the anticipated naming convention used for the custom parameter structures of Module classes
     * to maintain visual similarity when developing official modules. The structure is globally exposed to assist
     * testing, but may be hidden within private class section in the future to prevent unauthorized access.
     *
     * @attention This structure shares the address byte-code enumeration with the global kControllerRuntimeParameters
     * structure to optimize codebase maintainability.
     */
    struct kCustomKernelParameters
    {
        static constexpr uint8_t module_id = 0;  // While Kernel is not a Module, it does use an ID of 0
        // Kernel has no systems and always exists as a single entity, so uses kALLSystems code as system ID
        static constexpr uint8_t system_id = static_cast<uint8_t>(axmc_shared_assets::kGeneralByteCodes::kAllSystems);
    };

    /**
     * @struct kKernelErrorCodes
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
        kStandby                       = 11,  ///< Standby code used during initial class initialization
        kModulesNotSetError            = 12,  ///< A successful SetModules() runtime is required to use other methods
        kModulesSet                    = 13,  ///< SetModules() method runtime succeeded
        kModuleSetupComplete           = 14,  ///< SetupModules() method runtime succeeded
        kNoDataToReceive               = 15,  ///< No data to receive either due to reception error or genuinely no data
        kPayloadIDDataReadingError     = 16,  ///< Unable to extract the ID data of the received payload
        kPayloadParameterReadingError  = 17,  ///< Unable to extract a parameter structure from the received payload
        kInvalidKernelParameterIDError = 18,  ///< Unable to recognize Module-targeted parameter ID
        kInvalidModuleParameterIDError = 19,  ///< Unable to recognize Kernel-targeted parameter ID
        kKernelParameterSet            = 20,  ///< Kernel parameter has been successfully set to input value
        kModuleParameterSet            = 21,  ///< Module parameter has been successfully set to input value
        kNoCommandRequested            = 22,  ///< Received payload did not specify a command to execute
        kUnrecognizedKernelCommand     = 23,  ///< Unable to recognize the requested Kernel-targeted command code
        kEchoCommandComplete           = 24,  ///< Echo Kernel command executed successfully
        kModuleCommandQueued           = 25   ///< Module-addressed command has been successfully queued for execution
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
    AMCKernel(SerialPCCommunication& serial_communication_port);

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
    void SetModules(AMCModule* (&module_array)[module_number])
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
    void SetupModules();

    void ReceivePCData();

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
    bool SetParameter(const uint8_t id_code, const uint32_t value);

    /**
     * @brief Determines whether the input command code is valid and, if so, runs the requested AMCKernel clas command.
     *
     * @param command The id byte-code for the command to run. Can be 0 when no command is requested, the method will
     * automatically hand;e this as a correct non-erroneous no-command case.
     *
     * @note Uses kernel_status to communicate the success or failure of the method alongside the specific success or
     * failure code. Has no return value as it is not really helpful given the context of how this command is called.
     */
    void RunKernelCommand(uint8_t command);

    bool Echo();

  private:
    /// An internal array of AMCModule-derived Module-level classes. This array is set via SetModules() class method
    /// and it is used to store all Module's available to the Kernel during runtime. The methods of the Kernel loop over
    /// this array and call common interface methods inherited by each module from the core AMCModule class to execute
    /// Controller runtime behavior.
    AMCModule** modules;

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

    /// A reference to the shared instance of the SerialPCCommunication class that handles the bidirectional
    /// Controller-PC communication via the serial UART or USB interface. AMCKernel uses this class instance to both
    /// receive data from the PC and transmit data to the PC.
    SerialPCCommunication& communication_port;

    /**
     * @enum kControllerRuntimeParameterCodes
     * @brief Specifies id-codes for each field of the ControllerRuntimeParameters global structure and local
     * kCustomKernelParameters structure so that they can be uniquely addressed and modified using PC communication
     * interface.
     *
     * Whenever the PC sends a command to change a particular Kernel-exclusive or shared controller-wide parameter, it
     * should use the id-code that matches the specific ID code used in this enumeration to target a particular
     * parameter.
     *
     * Each enumerator in this enumeration is named after the specific parameter it represents.
     *
     * @note This enumeration is only available as a private variable of the AMCKernel class as a way to ensure only
     * this class is capable of addressing shared and local Kernel parameters.
     *
     * @attention This enumeration should avoid codes 0 through 10 and 250 through 255, as they are reserved for
     * specific system uses.
     */
    enum class kAddressableParameterCodes : uint8_t
    {
        kActionLock                = 11,  ///< Enables or disables triggering (writing to) action pins
        kTTLLock                   = 12,  ///< Enables or disables triggering (writing to) TTL pins
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
};

#endif  //AMC_KERNEL_H
