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
     * @brief Stores protocol codes used by the Communication class to specify incoming and outgoing message layouts.
     *
     * The Protocol byte-code instructs message parsers on how to process the incoming message. Each message has a
     * unique payload structure which cannot be parsed unless the underlying protocol is known.
     *
     * @attention The protocol code, derived from this enumeration, should be the first 'payload' byte of each message.
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
     * @brief Stores prototype codes used by the Communication class to specify the layout of additional data objects
     * transmitted with DataMessages (Kernel and Module).
     *
     * Since most transmitted data can be packaged into a small predefined set of objects, this enumeration is used
     * to map all currently supported data objects to a unique prototype code. In turn, this allows optimizing data
     * reception and logging on the PC side, as it can use the prototype code to precisely decode the data object of
     * every DataMessage. This in contrast to how ModuleParameter messages are processed, where the arbitrary data
     * portion of the message cannot be resolved with a single parsing cycle.
     *
     * @note While this approach essentially limits the number of valid prototype codes to 255 (256 if 0 is made a valid
     * code), realistically, this is more than enough to cover a very wide range of runtime cases. Currently, we provide
     * 165 unique prototype codes that can package up to 15 consecutive scalars, each being up to 64-bit (8-byte) in
     * size.
     *
     * @attention The prototypes in this enumeration are arranged in the ascending order of their memory footprint.
     */
    enum class kPrototypes : uint8_t
    {
        // 1 byte total
        kOneBool  = 1,  /// 1 8-bit boolean
        kOneUint8 = 2,  /// 1 unsigned 8-bit integer
        kOneInt8  = 3,  /// 1 signed 8-bit integer

        // 2 bytes total
        kTwoBools  = 4,  /// An array of 2 8-bit booleans
        kTwoUint8s = 5,  /// An array of 2 unsigned 8-bit integers
        kTwoInt8s  = 6,  /// An array of 2 signed 8-bit integers
        kOneUint16 = 7,  /// 1 unsigned 16-bit integer
        kOneInt16  = 8,  /// 1 signed 16-bit integer

        // 3 bytes total
        kThreeBools  = 9,   /// An array of 3 8-bit booleans
        kThreeUint8s = 10,  /// An array of 3 unsigned 8-bit integers
        kThreeInt8s  = 11,  /// An array of 3 signed 8-bit integers

        // 4 bytes total
        kFourBools  = 12,  /// An array of 4 8-bit booleans
        kFourUint8s = 13,  /// An array of 4 unsigned 8-bit integers
        kFourInt8s  = 14,  /// An array of 4 signed 8-bit integers
        kTwoUint16s = 15,  /// An array of 2 unsigned 16-bit integers
        kTwoInt16s  = 16,  /// An array of 2 signed 16-bit integers
        kOneUint32  = 17,  /// 1 unsigned 32-bit integer
        kOneInt32   = 18,  /// 1 signed 32-bit integer
        kOneFloat32 = 19,  /// 1 single-precision 32-bit floating-point number

        // 5 bytes total
        kFiveBools  = 20,  /// An array of 5 8-bit booleans
        kFiveUint8s = 21,  /// An array of 5 unsigned 8-bit integers
        kFiveInt8s  = 22,  /// An array of 5 signed 8-bit integers

        // 6 bytes total
        kSixBools     = 23,  /// An array of 6 8-bit booleans
        kSixUint8s    = 24,  /// An array of 6 unsigned 8-bit integers
        kSixInt8s     = 25,  /// An array of 6 signed 8-bit integers
        kThreeUint16s = 26,  /// An array of 3 unsigned 16-bit integers
        kThreeInt16s  = 27,  /// An array of 3 signed 16-bit integers

        // 7 bytes total
        kSevenBools  = 28,  /// An array of 7 8-bit booleans
        kSevenUint8s = 29,  /// An array of 7 unsigned 8-bit integers
        kSevenInt8s  = 30,  /// An array of 7 signed 8-bit integers

        // 8 bytes total
        kEightBools  = 31,  /// An array of 8 8-bit booleans
        kEightUint8s = 32,  /// An array of 8 unsigned 8-bit integers
        kEightInt8s  = 33,  /// An array of 8 signed 8-bit integers
        kFourUint16s = 34,  /// An array of 4 unsigned 16-bit integers
        kFourInt16s  = 35,  /// An array of 4 signed 16-bit integers
        kTwoUint32s  = 36,  /// An array of 2 unsigned 32-bit integers
        kTwoInt32s   = 37,  /// An array of 2 signed 32-bit integers
        kTwoFloat32s = 38,  /// An array of 2 single-precision 32-bit floating-point numbers
        kOneUint64   = 39,  /// 1 unsigned 64-bit integer
        kOneInt64    = 40,  /// 1 signed 64-bit integer
        kOneFloat64  = 41,  /// 1 double-precision 64-bit floating-point number

        // 9 bytes total
        kNineBools  = 42,  /// An array of 9 8-bit booleans
        kNineUint8s = 43,  /// An array of 9 unsigned 8-bit integers
        kNineInt8s  = 44,  /// An array of 9 signed 8-bit integers

        // 10 bytes total
        kTenBools    = 45,  /// An array of 10 8-bit booleans
        kTenUint8s   = 46,  /// An array of 10 unsigned 8-bit integers
        kTenInt8s    = 47,  /// An array of 10 signed 8-bit integers
        kFiveUint16s = 48,  /// An array of 5 unsigned 16-bit integers
        kFiveInt16s  = 49,  /// An array of 5 signed 16-bit integers

        // 11 bytes total
        kElevenBools  = 50,  /// An array of 11 8-bit booleans
        kElevenUint8s = 51,  /// An array of 11 unsigned 8-bit integers
        kElevenInt8s  = 52,  /// An array of 11 signed 8-bit integers

        // 12 bytes total
        kTwelveBools   = 53,  /// An array of 12 8-bit booleans
        kTwelveUint8s  = 54,  /// An array of 12 unsigned 8-bit integers
        kTwelveInt8s   = 55,  /// An array of 12 signed 8-bit integers
        kSixUint16s    = 56,  /// An array of 6 unsigned 16-bit integers
        kSixInt16s     = 57,  /// An array of 6 signed 16-bit integers
        kThreeUint32s  = 58,  /// An array of 3 unsigned 32-bit integers
        kThreeInt32s   = 59,  /// An array of 3 signed 32-bit integers
        kThreeFloat32s = 60,  /// An array of 3 single-precision 32-bit floating-point numbers

        // 13 bytes total
        kThirteenBools  = 61,  /// An array of 13 8-bit booleans
        kThirteenUint8s = 62,  /// An array of 13 unsigned 8-bit integers
        kThirteenInt8s  = 63,  /// An array of 13 signed 8-bit integers

        // 14 bytes total
        kFourteenBools  = 64,  /// An array of 14 8-bit booleans
        kFourteenUint8s = 65,  /// An array of 14 unsigned 8-bit integers
        kFourteenInt8s  = 66,  /// An array of 14 signed 8-bit integers
        kSevenUint16s   = 67,  /// An array of 7 unsigned 16-bit integers
        kSevenInt16s    = 68,  /// An array of 7 signed 16-bit integers

        // 15 bytes total
        kFifteenBools  = 69,  /// An array of 15 8-bit booleans
        kFifteenUint8s = 70,  /// An array of 15 unsigned 8-bit integers
        kFifteenInt8s  = 71,  /// An array of 15 signed 8-bit integers

        // 16 bytes total
        kEightUint16s = 72,  /// An array of 8 unsigned 16-bit integers
        kEightInt16s  = 73,  /// An array of 8 signed 16-bit integers
        kFourUint32s  = 74,  /// An array of 4 unsigned 32-bit integers
        kFourInt32s   = 75,  /// An array of 4 signed 32-bit integers
        kFourFloat32s = 76,  /// An array of 4 single-precision 32-bit floating-point numbers
        kTwoUint64s   = 77,  /// An array of 2 unsigned 64-bit integers
        kTwoInt64s    = 78,  /// An array of 2 signed 64-bit integers
        kTwoFloat64s  = 79,  /// An array of 2 double-precision 64-bit floating-point numbers

        // 18 bytes total
        kNineUint16s = 80,  /// An array of 9 unsigned 16-bit integers
        kNineInt16s  = 81,  /// An array of 9 signed 16-bit integers

        // 20 bytes total
        kTenUint16s   = 82,  /// An array of 10 unsigned 16-bit integers
        kTenInt16s    = 83,  /// An array of 10 signed 16-bit integers
        kFiveUint32s  = 84,  /// An array of 5 unsigned 32-bit integers
        kFiveInt32s   = 85,  /// An array of 5 signed 32-bit integers
        kFiveFloat32s = 86,  /// An array of 5 single-precision 32-bit floating-point numbers

        // 22 bytes total
        kElevenUint16s = 87,  /// An array of 11 unsigned 16-bit integers
        kElevenInt16s  = 88,  /// An array of 11 signed 16-bit integers

        // 24 bytes total
        kTwelveUint16s = 89,  /// An array of 12 unsigned 16-bit integers
        kTwelveInt16s  = 90,  /// An array of 12 signed 16-bit integers
        kSixUint32s    = 91,  /// An array of 6 unsigned 32-bit integers
        kSixInt32s     = 92,  /// An array of 6 signed 32-bit integers
        kSixFloat32s   = 93,  /// An array of 6 single-precision 32-bit floating-point numbers
        kThreeUint64s  = 94,  /// An array of 3 unsigned 64-bit integers
        kThreeInt64s   = 95,  /// An array of 3 signed 64-bit integers
        kThreeFloat64s = 96,  /// An array of 3 double-precision 64-bit floating-point numbers

        // 26 bytes total
        kThirteenUint16s = 97,  /// An array of 13 unsigned 16-bit integers
        kThirteenInt16s  = 98,  /// An array of 13 signed 16-bit integers

        // 28 bytes total
        kFourteenUint16s = 99,   /// An array of 14 unsigned 16-bit integers
        kFourteenInt16s  = 100,  /// An array of 14 signed 16-bit integers
        kSevenUint32s    = 101,  /// An array of 7 unsigned 32-bit integers
        kSevenInt32s     = 102,  /// An array of 7 signed 32-bit integers
        kSevenFloat32s   = 103,  /// An array of 7 single-precision 32-bit floating-point numbers

        // 30 bytes total
        kFifteenUint16s = 104,  /// An array of 15 unsigned 16-bit integers
        kFifteenInt16s  = 105,  /// An array of 15 signed 16-bit integers

        // 32 bytes total
        kEightUint32s  = 106,  /// An array of 8 unsigned 32-bit integers
        kEightInt32s   = 107,  /// An array of 8 signed 32-bit integers
        kEightFloat32s = 108,  /// An array of 8 single-precision 32-bit floating-point numbers
        kFourUint64s   = 109,  /// An array of 4 unsigned 64-bit integers
        kFourInt64s    = 110,  /// An array of 4 signed 64-bit integers
        kFourFloat64s  = 111,  /// An array of 4 double-precision 64-bit floating-point numbers

        // 36 bytes total
        kNineUint32s  = 112,  /// An array of 9 unsigned 32-bit integers
        kNineInt32s   = 113,  /// An array of 9 signed 32-bit integers
        kNineFloat32s = 114,  /// An array of 9 single-precision 32-bit floating-point numbers

        // 40 bytes total
        kTenUint32s   = 115,  /// An array of 10 unsigned 32-bit integers
        kTenInt32s    = 116,  /// An array of 10 signed 32-bit integers
        kTenFloat32s  = 117,  /// An array of 10 single-precision 32-bit floating-point numbers
        kFiveUint64s  = 118,  /// An array of 5 unsigned 64-bit integers
        kFiveInt64s   = 119,  /// An array of 5 signed 64-bit integers
        kFiveFloat64s = 120,  /// An array of 5 double-precision 64-bit floating-point numbers

        // 44 bytes total
        kElevenUint32s  = 121,  /// An array of 11 unsigned 32-bit integers
        kElevenInt32s   = 122,  /// An array of 11 signed 32-bit integers
        kElevenFloat32s = 123,  /// An array of 11 single-precision 32-bit floating-point numbers

        // 48 bytes total
        kTwelveUint32s  = 124,  /// An array of 12 unsigned 32-bit integers
        kTwelveInt32s   = 125,  /// An array of 12 signed 32-bit integers
        kTwelveFloat32s = 126,  /// An array of 12 single-precision 32-bit floating-point numbers
        kSixUint64s     = 127,  /// An array of 6 unsigned 64-bit integers
        kSixInt64s      = 128,  /// An array of 6 signed 64-bit integers
        kSixFloat64s    = 129,  /// An array of 6 double-precision 64-bit floating-point numbers

        // 52 bytes total
        kThirteenUint32s  = 130,  /// An array of 13 unsigned 32-bit integers
        kThirteenInt32s   = 131,  /// An array of 13 signed 32-bit integers
        kThirteenFloat32s = 132,  /// An array of 13 single-precision 32-bit floating-point numbers

        // 56 bytes total
        kFourteenUint32s  = 133,  /// An array of 14 unsigned 32-bit integers
        kFourteenInt32s   = 134,  /// An array of 14 signed 32-bit integers
        kFourteenFloat32s = 135,  /// An array of 14 single-precision 32-bit floating-point numbers
        kSevenUint64s     = 136,  /// An array of 7 unsigned 64-bit integers
        kSevenInt64s      = 137,  /// An array of 7 signed 64-bit integers
        kSevenFloat64s    = 138,  /// An array of 7 double-precision 64-bit floating-point numbers

        // 60 bytes total
        kFifteenUint32s  = 139,  /// An array of 15 unsigned 32-bit integers
        kFifteenInt32s   = 140,  /// An array of 15 signed 32-bit integers
        kFifteenFloat32s = 141,  /// An array of 15 single-precision 32-bit floating-point numbers

        // 64 bytes total
        kEightUint64s  = 142,  /// An array of 8 unsigned 64-bit integers
        kEightInt64s   = 143,  /// An array of 8 signed 64-bit integers
        kEightFloat64s = 144,  /// An array of 8 double-precision 64-bit floating-point numbers

        // 72 bytes total
        kNineUint64s  = 145,  /// An array of 9 unsigned 64-bit integers
        kNineInt64s   = 146,  /// An array of 9 signed 64-bit integers
        kNineFloat64s = 147,  /// An array of 9 double-precision 64-bit floating-point numbers

        // 80 bytes total
        kTenUint64s  = 148,  /// An array of 10 unsigned 64-bit integers
        kTenInt64s   = 149,  /// An array of 10 signed 64-bit integers
        kTenFloat64s = 150,  /// An array of 10 double-precision 64-bit floating-point numbers

        // 88 bytes total
        kElevenUint64s  = 151,  /// An array of 11 unsigned 64-bit integers
        kElevenInt64s   = 152,  /// An array of 11 signed 64-bit integers
        kElevenFloat64s = 153,  /// An array of 11 double-precision 64-bit floating-point numbers

        // 96 bytes total
        kTwelveUint64s  = 154,  /// An array of 12 unsigned 64-bit integers
        kTwelveInt64s   = 155,  /// An array of 12 signed 64-bit integers
        kTwelveFloat64s = 156,  /// An array of 12 double-precision 64-bit floating-point numbers

        // 104 bytes total
        kThirteenUint64s  = 157,  /// An array of 13 unsigned 64-bit integers
        kThirteenInt64s   = 158,  /// An array of 13 signed 64-bit integers
        kThirteenFloat64s = 159,  /// An array of 13 double-precision 64-bit floating-point numbers

        // 112 bytes total
        kFourteenUint64s  = 160,  /// An array of 14 unsigned 64-bit integers
        kFourteenInt64s   = 161,  /// An array of 14 signed 64-bit integers
        kFourteenFloat64s = 162,  /// An array of 14 double-precision 64-bit floating-point numbers

        // 120 bytes total
        kFifteenUint64s  = 163,  /// An array of 15 unsigned 64-bit integers
        kFifteenInt64s   = 164,  /// An array of 15 signed 64-bit integers
        kFifteenFloat64s = 165,  /// An array of 15 double-precision 64-bit floating-point numbers
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
