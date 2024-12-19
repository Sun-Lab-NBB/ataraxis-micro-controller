/**
 * @file
 * @brief The header-only file that stores all assets intended to be shared between library classes.
 *
 * @section axmc_sa_description Description:
 *
 * This file contains:
 * - axmc_shared_assets namespace that stores general-purpose assets shared between library classes.
 * - communication_assets namespace that stores structures and enumerations used by the Communication, Kernel and
 * (base) Module classes to support bidirectional communication with the PC.
 *
 * @section axmc_sa_developer_notes Developer Notes:
 *
 * The primary reason for having this file is to store shared byte-code enumerations and structures in the same place.
 * Many of the codes available through this file have to be unique across the library and / or Ataraxis project as
 * a whole.
 *
 * @section axmc_sa_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 */

#ifndef AXMC_SHARED_ASSETS_H
#define AXMC_SHARED_ASSETS_H

// Dependencies
#include <Arduino.h>

/**
 * @brief A structure with packed memory layout.
 *
 * Uses packed attribute to ensure the structure can be properly serialized during data transmission. This specification
 * should be used by all structures whose data is sent to the PC or received from the PC.
 */
#if defined(__GNUC__) || defined(__clang__)
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif

/**
 * @namespace axmc_shared_assets
 * @brief Provides all assets (structures, enumerations, functions) that are intended to be shared between the classes
 * of the library.
 *
 * The shared assets are primarily used to simplify library development by storing co-dependent assets in the same
 * place. Additionally, it simplifies using these assets with template classes from the library.
 */
namespace axmc_shared_assets
{
    /**
     * @enum kCommunicationCodes
     * @brief Assigns meaningful names to all status codes used by the Communication class.
     *
     * @note Due to the unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 151 through 200. Codes below 150 are reserved for the ataraxis-transport-layer-mc
     * library.
     */
    enum class kCommunicationCodes : uint8_t
    {
        kStandby             = 151,  ///< The default value used to initialize the communication_status variable.
        kReceptionError      = 152,  ///< Communication class ran into an error when receiving a message.
        kParsingError        = 153,  ///< Communication class ran into an error when parsing (reading) a message.
        kPackingError        = 154,  ///< Communication class ran into an error when writing a message to payload.
        kTransmissionError   = 155,  ///< Communication class ran into an error when transmitting a message.
        kMessageSent         = 156,  ///< Communication class successfully transmitted a message.
        kMessageReceived     = 157,  ///< Communication class successfully received a message.
        kInvalidProtocol     = 158,  ///< The message protocol code is not valid for the type of operation (Rx or Tx).
        kNoBytesToReceive    = 159,  ///< Communication class did not receive enough bytes to process the message.
        kParameterMismatch   = 160,  ///< The size of the received parameters structure does not match expectation.
        kParametersExtracted = 161,  ///< Parameter data has been successfully extracted.
        kExtractionForbidden = 162,  ///< Attempted to extract parameters from message other than ModuleParameters.
    };

    /**
     * @struct DynamicRuntimeParameters
     * @brief Stores global runtime parameters shared by all library classes and addressable through the Kernel class.
     *
     * These parameters broadly affect the runtime of all classes derived from the base Module class. They are
     * addressable through the Kernel class using the Communication interface.
     *
     * @warning The @b only class allowed to modify this structure is the Kernel class. Each (base) Module-derived
     * class requires an instance of this structure to properly support runtime behavior, but they should not modify the
     * instance.
     */
    struct DynamicRuntimeParameters
    {
            /// Enables running the controller logic without physically issuing commands. This is helpful for testing
            /// and debugging Microcontroller code. Specifically, blocks @b action pins from writing. Sensor and TTL
            /// pins are unaffected by this variable's state. Note, this only works for digital and analog writing
            /// methods inherited from the base Module class.
            bool action_lock = true;

            /// Same as action_lock, but specifically locks or unlocks output TTL pin activity. Same as action_lock,
            /// this only works for digital and analog writing methods inherited from the base Module class.
            bool ttl_lock = true;
    } PACKED_STRUCT;
}  // namespace axmc_shared_assets

/**
 * @namespace axmc_communication_assets
 * @brief Provides all assets (structures, enumerations, variables) necessary to interface with and support the
 * runtime of the Communication class.
 *
 * @attention These assets are designed to be used exclusively by the core classes: Communication, Kernel and (base)
 * Module. Do not modify or access these assets from any class derived from the base Module class or any other custom
 * class.
 */
namespace axmc_communication_assets
{
    /**
     * @enum kProtocols
     * @brief Stores protocol codes used by the Communication class to specify incoming and outgoing message structures.
     *
     * The Protocol byte-code instructs message parsers on how to process the incoming message. Each message has a
     * unique payload structure which cannot be parsed unless the underlying protocol is known.
     *
     * @attention The protocol code, derived from this enumeration, should be the first 'payload' byte of each message.
     * This enumeration should only be used by the Communication and Kernel classes. Protocol codes are designed to
     * be unique across the entire Ataraxis project.
     */
    enum class kProtocols : uint8_t
    {
        /// Not a valid protocol code. This is used to initialize the Communication class.
        kUndefined = 0,

        /// Protocol for receiving Module-addressed commands that should be repeated (executed recurrently).
        KRepeatedModuleCommand = 1,

        /// Protocol for receiving Module-addressed commands that should not be repeated (executed only once).
        kOneOffModuleCommand = 2,

        /// Protocol for receiving Module-addressed commands that remove all queued commands
        /// (including recurrent commands). This command is actually fulfilled by the Kernel that directly manipulates
        /// the local queue of the targeted Module.
        kDequeueModuleCommand = 3,

        /// Protocol for receiving Kernel-addressed commands. All Kernel commands are always non-repeatable (one-shot).
        kKernelCommand = 4,

        /// Protocol for receiving Module-addressed parameters. This relies on transmitting arbitrary sized parameter
        /// objects and requires each module to define a function for handling these parameters.
        kModuleParameters = 5,

        /// Protocol for receiving Kernel-addressed parameters. The parameters transmitted via these messages will be
        /// used to overwrite the fields of the global DynamicRuntimeParameters structure shared by all Modules.
        kKernelParameters = 6,

        /// Protocol for sending Module data or error messages that include an arbitrary data object in addition to
        /// event state-code.
        kModuleData = 7,

        /// Protocol for sending Kernel data or error messages that include an arbitrary data object in addition to
        /// event state-code.
        kKernelData = 8,

        /// Protocol for sending Module data or error messages that do not include additional data objects. These
        /// messages are optimized for delivering single event state-codes.
        kModuleState = 9,

        /// Protocol for sending Kernel data or error messages that do not include additional data objects. These
        /// messages are optimized for delivering single event state-codes.
        kKernelState = 10,

        /// Protocol used to acknowledge the reception of command and parameter messages. This is a minimalistic
        /// service protocol used to notify the PC that the controller received the message.
        kReceptionCode = 11,

        /// Protocol used to identify the controller to the PC. This is a service message protocol that transmits the
        /// unique ID of the controller to the PC. This is primarily used to determine the USB ports used by specific
        /// controllers.
        kIdentification = 12,
    };

    /**
     * @enum kPrototypes
     * @brief Stores prototype codes used by the Communication class to specify the data structure object that can be
     * used to parse DataMessage objects.
     *
     * Since most transmitted data objects use a small set of data structures, it is possible to map each used data
     * object structure to a unique prototype code. In turn, this allows optimizing data reception and logging on the PC
     * side as objet types can be obtained during a single reception cycle (the message instructs parser on
     * how to read the object).
     *
     * @note While this approach essentially limits the number of valid prototype codes to 255 (256 if 0 is made a valid
     * code), realistically, this is more than enough to cover a very wide range of runtime cases.
     */
    enum class kPrototypes : uint8_t
    {
        // Boolean
        /// A single bool value.
        kOneBool = 1,
        /// An array made up of two bool values.
        kTwoBools = 2,
        /// An array made up of three bool values.
        kThreeBools = 3,
        /// An array made up of four bool values.
        kFourBools = 4,
        /// An array made up of five bool values.
        kFiveBools = 5,

        // Unsigned integers
        /// A single unsigned byte (uint8_t) value.
        kOneUint8 = 10,
        /// An array made up of two unsigned byte (uint8_t) values.
        kTwoUint8s = 11,
        /// An array made up of three unsigned byte (uint8_t) values.
        kThreeUint8s = 12,
        /// An array made up of four unsigned byte (uint8_t) values.
        kFourUint8s = 13,
        /// An array made up of five unsigned byte (uint8_t) values.
        kFiveUint8s = 14,

        /// A single unsigned short (uint16_t) value.
        kOneUint16 = 20,
        /// An array made up of two unsigned short (uint16_t) values.
        kTwoUint16s = 21,
        /// An array made up of three unsigned short (uint16_t) values.
        kThreeUint16s = 22,
        /// An array made up of four unsigned short (uint16_t) values.
        kFourUint16s = 23,
        /// An array made up of five unsigned short (uint16_t) values.
        kFiveUint16s = 24,

        /// A single unsigned long (uint32_t) value.
        kOneUint32 = 30,
        /// An array made up of two unsigned long (uint32_t) values.
        kTwoUint32s = 31,
        /// An array made up of three unsigned long (uint32_t) values.
        kThreeUint32s = 32,
        /// An array made up of four unsigned long (uint32_t) values.
        kFourUint32s = 33,
        /// An array made up of five unsigned long (uint32_t) values.
        kFiveUint32s = 34,

        /// A single unsigned long long (uint64_t) value.
        kOneUint64 = 40,
        /// An array made up of two unsigned long long (uint64_t) values.
        kTwoUint64s = 41,
        /// An array made up of three unsigned long long (uint64_t) values.
        kThreeUint64s = 42,
        /// An array made up of four unsigned long long (uint64_t) values.
        kFourUint64s = 43,
        /// An array made up of five unsigned long long (uint64_t) values.
        kFiveUint64s = 44,

        // Signed integers
        /// A single signed byte (int8_t) value.
        kOneInt8 = 50,
        /// An array made up of two signed byte (int8_t) values.
        kTwoInt8s = 51,
        /// An array made up of three signed byte (int8_t) values.
        kThreeInt8s = 52,
        /// An array made up of four signed byte (int8_t) values.
        kFourInt8s = 53,
        /// An array made up of five signed byte (int8_t) values.
        kFiveInt8s = 54,

        /// A single signed short (int16_t) value.
        kOneInt16 = 60,
        /// An array made up of two signed short (int16_t) values.
        kTwoInt16s = 61,
        /// An array made up of three signed short (int16_t) values.
        kThreeInt16s = 62,
        /// An array made up of four signed short (int16_t) values.
        kFourInt16s = 63,
        /// An array made up of five signed short (int16_t) values.
        kFiveInt16s = 64,

        /// A single signed long (int32_t) value.
        kOneInt32 = 70,
        /// An array made up of two signed long (int32_t) values.
        kTwoInt32s = 71,
        /// An array made up of three signed long (int32_t) values.
        kThreeInt32s = 72,
        /// An array made up of four signed long (int32_t) values.
        kFourInt32s = 73,
        /// An array made up of five signed long (int32_t) values.
        kFiveInt32s = 74,

        /// A single signed long long (int64_t) value.
        kOneInt64 = 80,
        /// An array made up of two signed long long (int64_t) values.
        kTwoInt64s = 81,
        /// An array made up of three signed long long (int64_t) values.
        kThreeInt64s = 82,
        /// An array made up of four signed long long (int64_t) values.
        kFourInt64s = 83,
        /// An array made up of five signed long long (int64_t) values.
        kFiveInt64s = 84,

        // Floating point
        /// A single 32-bit float value.
        kOneFloat32 = 90,
        /// An array made up of two 32-bit float values.
        kTwoFloat32s = 91,
        /// An array made up of three 32-bit float values.
        kThreeFloat32s = 92,
        /// An array made up of four 32-bit float values.
        kFourFloat32s = 93,
        /// An array made up of five 32-bit float values.
        kFiveFloat32s = 94,

        /// A single 64-bit double value.
        kOneFloat64 = 100,
        /// An array made up of two 64-bit double values.
        kTwoFloat64s = 101,
        /// An array made up of three 64-bit double values.
        kThreeFloat64s = 102,
        /// An array made up of four 64-bit double values.
        kFourFloat64s = 103,
        /// An array made up of five 64-bit double values.
        kFiveFloat64s = 104,
    };

    /**
     * @struct RepeatedModuleCommand
     * @brief Instructs the addressed Module to run the specified command repeatedly (recurrently).
     */
    struct RepeatedModuleCommand
    {
            /// The type (family) code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact. Setting this field to 0 disables delivery receipts.
            uint8_t return_code = 0;

            /// The module-type-specific code of the command to execute.
            uint8_t command = 0;

            /// Determines whether the command runs in blocking or non-blocking mode. If set to false, the controller
            /// runtime will block in-place for any time-waiting loops during command execution. Otherwise,
            /// the controller will run other commands while waiting for the block to complete.
            bool noblock = false;

            /// The period of time, in microseconds, to delay before repeating (cycling) the command.
            uint32_t cycle_delay = 0;
    } PACKED_STRUCT;

    /**
     * @struct OneOffModuleCommand
     * @brief Instructs the addressed Module to run the specified command exactly once (non-recurrently).
     */
    struct OneOffModuleCommand
    {
            /// The type (family) code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact. Setting this field to 0 disables delivery receipts.
            uint8_t return_code = 0;

            /// The module-type-specific code of the command to execute.
            uint8_t command = 0;

            /// Determines whether the command runs in blocking or non-blocking mode. If set to false, the controller
            /// runtime will block in-place for any sensor- or time-waiting loops during command execution. Otherwise,
            /// the controller will run other commands while waiting for the block to complete.
            bool noblock = false;
    } PACKED_STRUCT;

    /**
     * @struct DequeueModuleCommand
     * @brief Instructs the addressed Module to clear (empty) its command queue.
     */
    struct DequeueModuleCommand
    {
            /// The type (family) code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact. Setting this field to 0 disables delivery receipts.
            uint8_t return_code = 0;
    } PACKED_STRUCT;

    /**
     * @struct KernelCommand
     * @brief Instructs the Kernel to run the specified command exactly once.
     *
     * Currently, the Kernel only supports blocking non-repeated (one-off) commands.
     */
    struct KernelCommand
    {
            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact. Setting this field to 0 disables delivery receipts.
            uint8_t return_code = 0;

            /// The Kernel-specific code of the command to execute.
            uint8_t command = 0;
    } PACKED_STRUCT;

    /**
     * @struct ModuleParameters
     * @brief Instructs the addressed Module to overwrite its custom parameters object with the included object data.
     *
     * @warning Module parameters are stored in a module-family-specific structure, whose layout will not be known by
     * the parser at message reception. The Kernel class will use the included module type and id information from
     * the message to call the addressed Module's public API method that will extract the parameters directly from the
     * reception buffer.
     */
    struct ModuleParameters
    {
            /// The type (family) code of the module to which the parameter configuration is addressed.
            uint8_t module_type = 0;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id = 0;

            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact. Setting this field to 0 disables delivery receipts.
            uint8_t return_code = 0;
    } PACKED_STRUCT;

    /**
     * @struct KernelParameters
     * @brief Instructs the Kernel to update the shared DynamicRuntimeParameters object with included data.
     *
     * @note Since the target data structure for this method is static and known at compile-time, the parser
     * will be able to resolve the target structure in-place.
     */
    struct KernelParameters
    {
            /// When this field is set to a value other than 0, the Communication class will send this code back to the
            /// sender upon successfully processing the received command. This is to notify the sender that the command
            /// was received intact. Setting this field to 0 disables delivery receipts.
            uint8_t return_code = 0;

            /// Since Kernel parameter structure is known at compile-time, this message structure automatically formats
            /// the parameter data to match the format used by the Kernel.
            axmc_shared_assets::DynamicRuntimeParameters dynamic_parameters;
    } PACKED_STRUCT;

    /**
     * @struct ModuleData
     * @brief Communicates the event state-code of the sender Module and includes an additional data object.
     *
     * @note For messages that only need to transmit an event state-code, use ModuleStateMessage structure for better
     * efficiency.
     *
     * @warning Remember to add the data object matching the included prototype code to the message payload before
     * sending the message.
     */
    struct ModuleData
    {
            /// The code of the message protocol used by this structure.
            uint8_t protocol;

            /// The type (family) code of the module which sent the data message.
            uint8_t module_type;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id;

            /// The module-type-specific code of the command the module was executing when it sent the data message.
            uint8_t command;

            /// The module-type-specific code of the event within the command runtime that prompted the data
            /// transmission.
            uint8_t event;

            /// The byte-code of the prototype object that can be used to decode the object data included with this
            /// message. The PC will read the object data into the prototype object specified by this code.
            uint8_t prototype;

    } PACKED_STRUCT;

    /**
     * @struct KernelData
     * @brief Communicates the event state-code of the Kernel and includes an additional data object.
     *
     * @attention For messages that only need to transmit an event state-code, use KernelState structure for better
     * efficiency.
     *
     * @warning Remember to add the data object matching the included prototype code to the message payload before
     * sending the message.
     * */
    struct KernelData
    {
            /// The code of the message protocol used by this structure.
            uint8_t protocol;

            /// The Kernel-specific code of the command the kernel was executing when it sent the data message.
            uint8_t command;

            /// The Kernel-specific code of the event within the command runtime that prompted the data transmission.
            uint8_t event;

            /// The byte-code of the prototype object that can be used to decode the object data included with this
            /// message. The PC will read the object data into the prototype object specified by this code.
            uint8_t prototype;

    } PACKED_STRUCT;

    /**
     * @struct ModuleState
     * @brief Communicates the event state-code of the sender Module.
     *
     * @attention For messages that need to transmit a data object in addition to the state event-code, use
     * ModuleData structure.
     */
    struct ModuleState
    {
            /// The code of the message protocol used by this structure.
            uint8_t protocol;

            /// The type (family) code of the module which sent the data message.
            uint8_t module_type;

            /// The specific module ID within the broader module family specified by module_type.
            uint8_t module_id;

            /// The module-type-specific code of the command the module was executing when it sent the data message.
            uint8_t command;

            /// The module-type-specific code of the event within the command runtime that prompted the data
            /// transmission.
            uint8_t event;
    } PACKED_STRUCT;

    /**
     * @struct KernelState
     * @brief Communicates the event state-code of the Kernel.
     *
     * @attention For messages that need to transmit a data object in addition to the state event-code, use
     * KernelData structure.
     */
    struct KernelState
    {
            /// The code of the message protocol used by this structure.
            uint8_t protocol;

            /// The Kernel-specific code of the command the Kernel was executing when it sent the data message.
            uint8_t command;

            /// The Kernel-specific code of the event within the command runtime that prompted the data transmission.
            uint8_t event;
    } PACKED_STRUCT;

}  // namespace axmc_communication_assets

#endif  //AXMC_SHARED_ASSETS_H
