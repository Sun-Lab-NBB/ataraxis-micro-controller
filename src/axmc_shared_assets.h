/**
 * @file
 * @brief This file provides the assets shared between all library components.
 */

#ifndef AXMC_SHARED_ASSETS_H
#define AXMC_SHARED_ASSETS_H

// Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

/**
 * @namespace axmc_shared_assets
 * @brief Provides all assets (structures, enumerations, functions) that are intended to be shared between library
 * components.
 */
namespace axmc_shared_assets
{
    /**
     * @enum kCommunicationStatusCodes
     * @brief Defines the codes used by the Communication class to indicate the status of all supported data
     * manipulations.
     */
    enum class kCommunicationStatusCodes : uint8_t
    {
        kStandby             = 51,  ///< The default value used to initialize the communication_status variable.
        kReceptionError      = 52,  ///< Communication class ran into an error when receiving a message.
        kParsingError        = 53,  ///< Communication class ran into an error when parsing (reading) a message.
        kPackingError        = 54,  ///< Communication class ran into an error when writing a message to payload.
        kMessageSent         = 55,  ///< Communication class successfully sent a message.
        kMessageReceived     = 56,  ///< Communication class successfully received a message.
        kInvalidProtocol     = 57,  ///< The message protocol code is not valid for the type of operation (Rx or Tx).
        kNoBytesToReceive    = 58,  ///< Communication class did not receive enough bytes to process the message.
        kParameterMismatch   = 59,  ///< The size of the received parameters structure does not match expectation.
        kParametersExtracted = 60,  ///< Parameter data has been successfully extracted.
        kExtractionForbidden = 61,  ///< Attempted to extract parameters from the message other than ModuleParameters.
    };

    /**
     * @struct DynamicRuntimeParameters
     * @brief Stores global runtime parameters shared by all library assets and addressable through the Kernel instance.
     *
     * These parameters broadly affect the runtime behavior of all Module-derived instances. These parameters are
     * dynamically configured using the data transmitted from the PC.
     *
     * @warning The @b only class allowed to modify this structure is the Kernel. End users should never modify the
     * elements of this structure from any custom classes.
     */
    struct DynamicRuntimeParameters
    {
            /// Determines whether the library is allowed to change the output state of the hardware pins that control
            /// the 'actor' hardware modules.
            bool action_lock = true;

            /// Determines whether the library is allowed to change the output state of the hardware pins that control
            /// the 'ttl' (communication) hardware modules.
            bool ttl_lock = true;
    } PACKED_STRUCT;
}  // namespace axmc_shared_assets

/**
 * @namespace axmc_communication_assets
 * @brief Provides all assets (structures, enumerations, variables) used to support bidirectional communication with the
 * host-computer (PC).
 *
 * @warning These assets are designed to be used internally by the core library classes. End users should not modify
 * any assets from this namespace.
 */
namespace axmc_communication_assets
{
    /**
     * @enum kProtocols
     * @brief Defines the protocol codes used by the Communication class to specify incoming and outgoing message
     * layouts.
     */
    enum class kProtocols : uint8_t
    {
        /// Not a valid protocol code. Used to initialize the Communication class.
        kUndefined = 0,

        /// Used by Module-addressed commands that should be repeated (executed recurrently).
        KRepeatedModuleCommand = 1,

        /// Used by Module-addressed commands that should not be repeated (executed only once).
        kOneOffModuleCommand = 2,

        /// Used by Module-addressed commands that remove all queued commands, including recurrent commands.
        kDequeueModuleCommand = 3,

        /// Used by Kernel-addressed commands. All Kernel commands are always non-repeatable (one-shot).
        kKernelCommand = 4,

        /// Used by Module-addressed parameter messages.
        kModuleParameters = 5,

        /// Used by Kernel-addressed parameter messages.
        kKernelParameters = 6,

        /// Used by Module data or error messages that include an arbitrary data object in addition to the event
        /// state-code.
        kModuleData = 7,

        /// Used by Kernel data or error messages that include an arbitrary data object in addition to event state-code.
        kKernelData = 8,

        /// Used by Module data or error messages that only include the state-code.
        kModuleState = 9,

        /// Used by Kernel data or error messages that only include the state-code.
        kKernelState = 10,

        /// Used to acknowledge the reception of command and parameter messages.
        kReceptionCode = 11,

        /// Used to identify the host-microcontroller to the PC.
        kControllerIdentification = 12,

        /// Used to identify the hardware module instances managed by a Kernel instance to the PC.
        kModuleIdentification = 13,
    };

    /**
     * @enum kPrototypes
     * @brief Defines the prototype codes used by the Communication class to specify the layout of additional data
     * objects transmitted by KernelData and ModuleData messages.
     *
     * @warning The data messages can only transmit the objects whose prototypes are defined in this enumeration.
     */
    enum class kPrototypes : uint8_t
    {
        // 1 byte total
        kOneBool  = 1,  ///< 1 8-bit boolean
        kOneUint8 = 2,  ///< 1 unsigned 8-bit integer
        kOneInt8  = 3,  ///< 1 signed 8-bit integer

        // 2 bytes total
        kTwoBools  = 4,  ///< An array of 2 8-bit booleans
        kTwoUint8s = 5,  ///< An array of 2 unsigned 8-bit integers
        kTwoInt8s  = 6,  ///< An array of 2 signed 8-bit integers
        kOneUint16 = 7,  ///< 1 unsigned 16-bit integer
        kOneInt16  = 8,  ///< 1 signed 16-bit integer

        // 3 bytes total
        kThreeBools  = 9,   ///< An array of 3 8-bit booleans
        kThreeUint8s = 10,  ///< An array of 3 unsigned 8-bit integers
        kThreeInt8s  = 11,  ///< An array of 3 signed 8-bit integers

        // 4 bytes total
        kFourBools  = 12,  ///< An array of 4 8-bit booleans
        kFourUint8s = 13,  ///< An array of 4 unsigned 8-bit integers
        kFourInt8s  = 14,  ///< An array of 4 signed 8-bit integers
        kTwoUint16s = 15,  ///< An array of 2 unsigned 16-bit integers
        kTwoInt16s  = 16,  ///< An array of 2 signed 16-bit integers
        kOneUint32  = 17,  ///< 1 unsigned 32-bit integer
        kOneInt32   = 18,  ///< 1 signed 32-bit integer
        kOneFloat32 = 19,  ///< 1 single-precision 32-bit floating-point number

        // 5 bytes total
        kFiveBools  = 20,  ///< An array of 5 8-bit booleans
        kFiveUint8s = 21,  ///< An array of 5 unsigned 8-bit integers
        kFiveInt8s  = 22,  ///< An array of 5 signed 8-bit integers

        // 6 bytes total
        kSixBools     = 23,  ///< An array of 6 8-bit booleans
        kSixUint8s    = 24,  ///< An array of 6 unsigned 8-bit integers
        kSixInt8s     = 25,  ///< An array of 6 signed 8-bit integers
        kThreeUint16s = 26,  ///< An array of 3 unsigned 16-bit integers
        kThreeInt16s  = 27,  ///< An array of 3 signed 16-bit integers

        // 7 bytes total
        kSevenBools  = 28,  ///< An array of 7 8-bit booleans
        kSevenUint8s = 29,  ///< An array of 7 unsigned 8-bit integers
        kSevenInt8s  = 30,  ///< An array of 7 signed 8-bit integers

        // 8 bytes total
        kEightBools  = 31,  ///< An array of 8 8-bit booleans
        kEightUint8s = 32,  ///< An array of 8 unsigned 8-bit integers
        kEightInt8s  = 33,  ///< An array of 8 signed 8-bit integers
        kFourUint16s = 34,  ///< An array of 4 unsigned 16-bit integers
        kFourInt16s  = 35,  ///< An array of 4 signed 16-bit integers
        kTwoUint32s  = 36,  ///< An array of 2 unsigned 32-bit integers
        kTwoInt32s   = 37,  ///< An array of 2 signed 32-bit integers
        kTwoFloat32s = 38,  ///< An array of 2 single-precision 32-bit floating-point numbers
        kOneUint64   = 39,  ///< 1 unsigned 64-bit integer
        kOneInt64    = 40,  ///< 1 signed 64-bit integer
        kOneFloat64  = 41,  ///< 1 double-precision 64-bit floating-point number

        // 9 bytes total
        kNineBools  = 42,  ///< An array of 9 8-bit booleans
        kNineUint8s = 43,  ///< An array of 9 unsigned 8-bit integers
        kNineInt8s  = 44,  ///< An array of 9 signed 8-bit integers

        // 10 bytes total
        kTenBools    = 45,  ///< An array of 10 8-bit booleans
        kTenUint8s   = 46,  ///< An array of 10 unsigned 8-bit integers
        kTenInt8s    = 47,  ///< An array of 10 signed 8-bit integers
        kFiveUint16s = 48,  ///< An array of 5 unsigned 16-bit integers
        kFiveInt16s  = 49,  ///< An array of 5 signed 16-bit integers

        // 11 bytes total
        kElevenBools  = 50,  ///< An array of 11 8-bit booleans
        kElevenUint8s = 51,  ///< An array of 11 unsigned 8-bit integers
        kElevenInt8s  = 52,  ///< An array of 11 signed 8-bit integers

        // 12 bytes total
        kTwelveBools   = 53,  ///< An array of 12 8-bit booleans
        kTwelveUint8s  = 54,  ///< An array of 12 unsigned 8-bit integers
        kTwelveInt8s   = 55,  ///< An array of 12 signed 8-bit integers
        kSixUint16s    = 56,  ///< An array of 6 unsigned 16-bit integers
        kSixInt16s     = 57,  ///< An array of 6 signed 16-bit integers
        kThreeUint32s  = 58,  ///< An array of 3 unsigned 32-bit integers
        kThreeInt32s   = 59,  ///< An array of 3 signed 32-bit integers
        kThreeFloat32s = 60,  ///< An array of 3 single-precision 32-bit floating-point numbers

        // 13 bytes total
        kThirteenBools  = 61,  ///< An array of 13 8-bit booleans
        kThirteenUint8s = 62,  ///< An array of 13 unsigned 8-bit integers
        kThirteenInt8s  = 63,  ///< An array of 13 signed 8-bit integers

        // 14 bytes total
        kFourteenBools  = 64,  ///< An array of 14 8-bit booleans
        kFourteenUint8s = 65,  ///< An array of 14 unsigned 8-bit integers
        kFourteenInt8s  = 66,  ///< An array of 14 signed 8-bit integers
        kSevenUint16s   = 67,  ///< An array of 7 unsigned 16-bit integers
        kSevenInt16s    = 68,  ///< An array of 7 signed 16-bit integers

        // 15 bytes total
        kFifteenBools  = 69,  ///< An array of 15 8-bit booleans
        kFifteenUint8s = 70,  ///< An array of 15 unsigned 8-bit integers
        kFifteenInt8s  = 71,  ///< An array of 15 signed 8-bit integers

        // 16 bytes total
        kEightUint16s = 72,  ///< An array of 8 unsigned 16-bit integers
        kEightInt16s  = 73,  ///< An array of 8 signed 16-bit integers
        kFourUint32s  = 74,  ///< An array of 4 unsigned 32-bit integers
        kFourInt32s   = 75,  ///< An array of 4 signed 32-bit integers
        kFourFloat32s = 76,  ///< An array of 4 single-precision 32-bit floating-point numbers
        kTwoUint64s   = 77,  ///< An array of 2 unsigned 64-bit integers
        kTwoInt64s    = 78,  ///< An array of 2 signed 64-bit integers
        kTwoFloat64s  = 79,  ///< An array of 2 double-precision 64-bit floating-point numbers

        // 18 bytes total
        kNineUint16s = 80,  ///< An array of 9 unsigned 16-bit integers
        kNineInt16s  = 81,  ///< An array of 9 signed 16-bit integers

        // 20 bytes total
        kTenUint16s   = 82,  ///< An array of 10 unsigned 16-bit integers
        kTenInt16s    = 83,  ///< An array of 10 signed 16-bit integers
        kFiveUint32s  = 84,  ///< An array of 5 unsigned 32-bit integers
        kFiveInt32s   = 85,  ///< An array of 5 signed 32-bit integers
        kFiveFloat32s = 86,  ///< An array of 5 single-precision 32-bit floating-point numbers

        // 22 bytes total
        kElevenUint16s = 87,  ///< An array of 11 unsigned 16-bit integers
        kElevenInt16s  = 88,  ///< An array of 11 signed 16-bit integers

        // 24 bytes total
        kTwelveUint16s = 89,  ///< An array of 12 unsigned 16-bit integers
        kTwelveInt16s  = 90,  ///< An array of 12 signed 16-bit integers
        kSixUint32s    = 91,  ///< An array of 6 unsigned 32-bit integers
        kSixInt32s     = 92,  ///< An array of 6 signed 32-bit integers
        kSixFloat32s   = 93,  ///< An array of 6 single-precision 32-bit floating-point numbers
        kThreeUint64s  = 94,  ///< An array of 3 unsigned 64-bit integers
        kThreeInt64s   = 95,  ///< An array of 3 signed 64-bit integers
        kThreeFloat64s = 96,  ///< An array of 3 double-precision 64-bit floating-point numbers

        // 26 bytes total
        kThirteenUint16s = 97,  ///< An array of 13 unsigned 16-bit integers
        kThirteenInt16s  = 98,  ///< An array of 13 signed 16-bit integers

        // 28 bytes total
        kFourteenUint16s = 99,   ///< An array of 14 unsigned 16-bit integers
        kFourteenInt16s  = 100,  ///< An array of 14 signed 16-bit integers
        kSevenUint32s    = 101,  ///< An array of 7 unsigned 32-bit integers
        kSevenInt32s     = 102,  ///< An array of 7 signed 32-bit integers
        kSevenFloat32s   = 103,  ///< An array of 7 single-precision 32-bit floating-point numbers

        // 30 bytes total
        kFifteenUint16s = 104,  ///< An array of 15 unsigned 16-bit integers
        kFifteenInt16s  = 105,  ///< An array of 15 signed 16-bit integers

        // 32 bytes total
        kEightUint32s  = 106,  ///< An array of 8 unsigned 32-bit integers
        kEightInt32s   = 107,  ///< An array of 8 signed 32-bit integers
        kEightFloat32s = 108,  ///< An array of 8 single-precision 32-bit floating-point numbers
        kFourUint64s   = 109,  ///< An array of 4 unsigned 64-bit integers
        kFourInt64s    = 110,  ///< An array of 4 signed 64-bit integers
        kFourFloat64s  = 111,  ///< An array of 4 double-precision 64-bit floating-point numbers

        // 36 bytes total
        kNineUint32s  = 112,  ///< An array of 9 unsigned 32-bit integers
        kNineInt32s   = 113,  ///< An array of 9 signed 32-bit integers
        kNineFloat32s = 114,  ///< An array of 9 single-precision 32-bit floating-point numbers

        // 40 bytes total
        kTenUint32s   = 115,  ///< An array of 10 unsigned 32-bit integers
        kTenInt32s    = 116,  ///< An array of 10 signed 32-bit integers
        kTenFloat32s  = 117,  ///< An array of 10 single-precision 32-bit floating-point numbers
        kFiveUint64s  = 118,  ///< An array of 5 unsigned 64-bit integers
        kFiveInt64s   = 119,  ///< An array of 5 signed 64-bit integers
        kFiveFloat64s = 120,  ///< An array of 5 double-precision 64-bit floating-point numbers

        // 44 bytes total
        kElevenUint32s  = 121,  ///< An array of 11 unsigned 32-bit integers
        kElevenInt32s   = 122,  ///< An array of 11 signed 32-bit integers
        kElevenFloat32s = 123,  ///< An array of 11 single-precision 32-bit floating-point numbers

        // 48 bytes total
        kTwelveUint32s  = 124,  ///< An array of 12 unsigned 32-bit integers
        kTwelveInt32s   = 125,  ///< An array of 12 signed 32-bit integers
        kTwelveFloat32s = 126,  ///< An array of 12 single-precision 32-bit floating-point numbers
        kSixUint64s     = 127,  ///< An array of 6 unsigned 64-bit integers
        kSixInt64s      = 128,  ///< An array of 6 signed 64-bit integers
        kSixFloat64s    = 129,  ///< An array of 6 double-precision 64-bit floating-point numbers

        // 52 bytes total
        kThirteenUint32s  = 130,  ///< An array of 13 unsigned 32-bit integers
        kThirteenInt32s   = 131,  ///< An array of 13 signed 32-bit integers
        kThirteenFloat32s = 132,  ///< An array of 13 single-precision 32-bit floating-point numbers

        // 56 bytes total
        kFourteenUint32s  = 133,  ///< An array of 14 unsigned 32-bit integers
        kFourteenInt32s   = 134,  ///< An array of 14 signed 32-bit integers
        kFourteenFloat32s = 135,  ///< An array of 14 single-precision 32-bit floating-point numbers
        kSevenUint64s     = 136,  ///< An array of 7 unsigned 64-bit integers
        kSevenInt64s      = 137,  ///< An array of 7 signed 64-bit integers
        kSevenFloat64s    = 138,  ///< An array of 7 double-precision 64-bit floating-point numbers

        // 60 bytes total
        kFifteenUint32s  = 139,  ///< An array of 15 unsigned 32-bit integers
        kFifteenInt32s   = 140,  ///< An array of 15 signed 32-bit integers
        kFifteenFloat32s = 141,  ///< An array of 15 single-precision 32-bit floating-point numbers

        // 64 bytes total
        kEightUint64s  = 142,  ///< An array of 8 unsigned 64-bit integers
        kEightInt64s   = 143,  ///< An array of 8 signed 64-bit integers
        kEightFloat64s = 144,  ///< An array of 8 double-precision 64-bit floating-point numbers

        // 72 bytes total
        kNineUint64s  = 145,  ///< An array of 9 unsigned 64-bit integers
        kNineInt64s   = 146,  ///< An array of 9 signed 64-bit integers
        kNineFloat64s = 147,  ///< An array of 9 double-precision 64-bit floating-point numbers

        // 80 bytes total
        kTenUint64s  = 148,  ///< An array of 10 unsigned 64-bit integers
        kTenInt64s   = 149,  ///< An array of 10 signed 64-bit integers
        kTenFloat64s = 150,  ///< An array of 10 double-precision 64-bit floating-point numbers

        // 88 bytes total
        kElevenUint64s  = 151,  ///< An array of 11 unsigned 64-bit integers
        kElevenInt64s   = 152,  ///< An array of 11 signed 64-bit integers
        kElevenFloat64s = 153,  ///< An array of 11 double-precision 64-bit floating-point numbers

        // 96 bytes total
        kTwelveUint64s  = 154,  ///< An array of 12 unsigned 64-bit integers
        kTwelveInt64s   = 155,  ///< An array of 12 signed 64-bit integers
        kTwelveFloat64s = 156,  ///< An array of 12 double-precision 64-bit floating-point numbers

        // 104 bytes total
        kThirteenUint64s  = 157,  ///< An array of 13 unsigned 64-bit integers
        kThirteenInt64s   = 158,  ///< An array of 13 signed 64-bit integers
        kThirteenFloat64s = 159,  ///< An array of 13 double-precision 64-bit floating-point numbers

        // 112 bytes total
        kFourteenUint64s  = 160,  ///< An array of 14 unsigned 64-bit integers
        kFourteenInt64s   = 161,  ///< An array of 14 signed 64-bit integers
        kFourteenFloat64s = 162,  ///< An array of 14 double-precision 64-bit floating-point numbers

        // 120 bytes total
        kFifteenUint64s  = 163,  ///< An array of 15 unsigned 64-bit integers
        kFifteenInt64s   = 164,  ///< An array of 15 signed 64-bit integers
        kFifteenFloat64s = 165,  ///< An array of 15 double-precision 64-bit floating-point numbers
    };

    /**
     * @struct RepeatedModuleCommand
     * @brief Instructs the addressed Module instance to run the specified command repeatedly (recurrently).
     */
    struct RepeatedModuleCommand
    {
            /// The type (family) code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The ID of the specific module instance within the broader module family.
            uint8_t module_id = 0;

            /// The code to use for acknowledging the reception of the message, if set to a non-zero value.
            uint8_t return_code = 0;

            /// The code of the command to execute.
            uint8_t command = 0;

            /// Determines whether to allow concurrent execution of other commands while waiting for the requested
            /// command to complete.
            bool noblock = false;

            /// The delay, in microseconds, before repeating (cycling) the command.
            uint32_t cycle_delay = 0;
    } PACKED_STRUCT;

    /**
     * @struct OneOffModuleCommand
     * @brief Instructs the addressed Module instance to run the specified command exactly once (non-recurrently).
     */
    struct OneOffModuleCommand
    {
            /// The type (family) code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The ID of the specific module instance within the broader module family.
            uint8_t module_id = 0;

            /// The code to use for acknowledging the reception of the message, if set to a non-zero value.
            uint8_t return_code = 0;

            /// The code of the command to execute.
            uint8_t command = 0;

            /// Determines whether to allow concurrent execution of other commands while waiting for the requested
            /// command to complete.
            bool noblock = false;
    } PACKED_STRUCT;

    /**
     * @struct DequeueModuleCommand
     * @brief Instructs the addressed Module instance to clear (empty) its command queue.
     */
    struct DequeueModuleCommand
    {
            /// The type (family) code of the module to which the command is addressed.
            uint8_t module_type = 0;

            /// The ID of the specific module instance within the broader module family.
            uint8_t module_id = 0;

            /// The code to use for acknowledging the reception of the message, if set to a non-zero value.
            uint8_t return_code = 0;
    } PACKED_STRUCT;

    /**
     * @struct KernelCommand
     * @brief Instructs the Kernel to run the specified command exactly once.
     */
    struct KernelCommand
    {
            /// The code to use for acknowledging the reception of the message, if set to a non-zero value.
            uint8_t return_code = 0;

            /// The code of the command to execute.
            uint8_t command = 0;
    } PACKED_STRUCT;

    /**
     * @struct ModuleParameters
     * @brief Instructs the addressed Module instance to update its parameters with the included data.
     */
    struct ModuleParameters
    {
            /// The type (family) code of the module to which the parameter configuration is addressed.
            uint8_t module_type = 0;

            /// The ID of the specific module instance within the broader module family.
            uint8_t module_id = 0;

            /// The code to use for acknowledging the reception of the message, if set to a non-zero value.
            uint8_t return_code = 0;
    } PACKED_STRUCT;

    /**
     * @struct KernelParameters
     * @brief Instructs the Kernel to update the shared DynamicRuntimeParameters object with the included data.
     */
    struct KernelParameters
    {
            /// The code to use for acknowledging the reception of the message, if set to a non-zero value.
            uint8_t return_code = 0;

            /// The DynamicRuntimeParameters structure that stored the updated parameters.
            axmc_shared_assets::DynamicRuntimeParameters dynamic_parameters;
    } PACKED_STRUCT;

    /**
     * @struct ModuleData
     * @brief Communicates that the Module has encountered a notable event and includes an additional data object.
     *
     * @note Use the ModuleState structure for messages that only need to transmit an event state-code.
     */
    struct ModuleData
    {
            /// The message protocol used by this structure.
            uint8_t protocol;

            /// The type (family) code of the module that sent the data message.
            uint8_t module_type;

            /// The ID of the specific module instance within the broader module family.
            uint8_t module_id;

            /// The command the Module was executing when it sent the data message.
            uint8_t command;

            /// The event that prompted the data transmission.
            uint8_t event;

            /// The code that specifies the type of the data object transmitted with the message.
            uint8_t prototype;

    } PACKED_STRUCT;

    /**
     * @struct KernelData
     * @brief Communicates that the Kernel has encountered a notable event and includes an additional data object.
     *
     * @note Use the KernelState structure for messages that only need to transmit an event state-code.
     * */
    struct KernelData
    {
            /// The message protocol used by this structure.
            uint8_t protocol;

            /// The command the Kernel was executing when it sent the data message.
            uint8_t command;

            /// The event that prompted the data transmission.
            uint8_t event;

            /// The code that specifies the type of the data object transmitted with the message.
            uint8_t prototype;

    } PACKED_STRUCT;

    /**
     * @struct ModuleState
     * @brief Communicates that the Module has encountered a notable event.
     *
     * @note Use the ModuleData structure for messages that need to transmit a data object in addition to the state
     * event-code.
     */
    struct ModuleState
    {
            /// The message protocol used by this structure.
            uint8_t protocol;

            /// The type (family) code of the module that sent the data message.
            uint8_t module_type;

            /// The ID of the specific module instance within the broader module family.
            uint8_t module_id;

            /// The command the Module was executing when it sent the data message.
            uint8_t command;

            /// The event that prompted the data transmission.
            uint8_t event;
    } PACKED_STRUCT;

    /**
     * @struct KernelState
     * @brief Communicates that the Kernel has encountered a notable event.
     *
     * @note Use the KernelData structure for messages that need to transmit a data object in addition to the state
     * event-code.
     */
    struct KernelState
    {
            /// The message protocol used by this structure.
            uint8_t protocol;

            /// The command the Kernel was executing when it sent the data message.
            uint8_t command;

            /// The event that prompted the data transmission.
            uint8_t event;
    } PACKED_STRUCT;

}  // namespace axmc_communication_assets

#endif  //AXMC_SHARED_ASSETS_H
