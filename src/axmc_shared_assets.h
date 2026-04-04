/**
 * @file
 * @brief Provides the assets shared between all library components.
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
        kUndefined                = 0,   ///< Not a valid protocol code. Used to initialize the Communication class.
        kRepeatedModuleCommand    = 1,   ///< Module-addressed commands that should be repeated (recurrently).
        kOneOffModuleCommand      = 2,   ///< Module-addressed commands that should not be repeated (one-off).
        kDequeueModuleCommand     = 3,   ///< Module commands that clear all queued commands, including recurrent ones.
        kKernelCommand            = 4,   ///< Kernel-addressed commands. All Kernel commands are non-repeatable.
        kModuleParameters         = 5,   ///< Module-addressed parameter messages.
        kModuleData               = 6,   ///< Module messages with an event state-code and an additional data object.
        kKernelData               = 7,   ///< Kernel messages with an event state-code and an additional data object.
        kModuleState              = 8,   ///< Module data or error messages that only include the state-code.
        kKernelState              = 9,   ///< Kernel data or error messages that only include the state-code.
        kReceptionCode            = 10,  ///< Acknowledges the reception of command and parameter messages.
        kControllerIdentification = 11,  ///< Identifies the host-microcontroller to the PC.
        kModuleIdentification     = 12,  ///< Identifies the module instances managed by the Kernel to the PC.
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
        kOneBool  = 1,  ///< 1 8-bit boolean.
        kOneUint8 = 2,  ///< 1 unsigned 8-bit integer.
        kOneInt8  = 3,  ///< 1 signed 8-bit integer.

        // 2 bytes total
        kTwoBools  = 4,  ///< An array of 2 8-bit booleans.
        kTwoUint8s = 5,  ///< An array of 2 unsigned 8-bit integers.
        kTwoInt8s  = 6,  ///< An array of 2 signed 8-bit integers.
        kOneUint16 = 7,  ///< 1 unsigned 16-bit integer.
        kOneInt16  = 8,  ///< 1 signed 16-bit integer.

        // 3 bytes total
        kThreeBools  = 9,   ///< An array of 3 8-bit booleans.
        kThreeUint8s = 10,  ///< An array of 3 unsigned 8-bit integers.
        kThreeInt8s  = 11,  ///< An array of 3 signed 8-bit integers.

        // 4 bytes total
        kFourBools  = 12,  ///< An array of 4 8-bit booleans.
        kFourUint8s = 13,  ///< An array of 4 unsigned 8-bit integers.
        kFourInt8s  = 14,  ///< An array of 4 signed 8-bit integers.
        kTwoUint16s = 15,  ///< An array of 2 unsigned 16-bit integers.
        kTwoInt16s  = 16,  ///< An array of 2 signed 16-bit integers.
        kOneUint32  = 17,  ///< 1 unsigned 32-bit integer.
        kOneInt32   = 18,  ///< 1 signed 32-bit integer.
        kOneFloat32 = 19,  ///< 1 single-precision 32-bit floating-point number.

        // 5 bytes total
        kFiveBools  = 20,  ///< An array of 5 8-bit booleans.
        kFiveUint8s = 21,  ///< An array of 5 unsigned 8-bit integers.
        kFiveInt8s  = 22,  ///< An array of 5 signed 8-bit integers.

        // 6 bytes total
        kSixBools     = 23,  ///< An array of 6 8-bit booleans.
        kSixUint8s    = 24,  ///< An array of 6 unsigned 8-bit integers.
        kSixInt8s     = 25,  ///< An array of 6 signed 8-bit integers.
        kThreeUint16s = 26,  ///< An array of 3 unsigned 16-bit integers.
        kThreeInt16s  = 27,  ///< An array of 3 signed 16-bit integers.

        // 7 bytes total
        kSevenBools  = 28,  ///< An array of 7 8-bit booleans.
        kSevenUint8s = 29,  ///< An array of 7 unsigned 8-bit integers.
        kSevenInt8s  = 30,  ///< An array of 7 signed 8-bit integers.

        // 8 bytes total
        kEightBools  = 31,  ///< An array of 8 8-bit booleans.
        kEightUint8s = 32,  ///< An array of 8 unsigned 8-bit integers.
        kEightInt8s  = 33,  ///< An array of 8 signed 8-bit integers.
        kFourUint16s = 34,  ///< An array of 4 unsigned 16-bit integers.
        kFourInt16s  = 35,  ///< An array of 4 signed 16-bit integers.
        kTwoUint32s  = 36,  ///< An array of 2 unsigned 32-bit integers.
        kTwoInt32s   = 37,  ///< An array of 2 signed 32-bit integers.
        kTwoFloat32s = 38,  ///< An array of 2 single-precision 32-bit floating-point numbers.
        kOneUint64   = 39,  ///< 1 unsigned 64-bit integer.
        kOneInt64    = 40,  ///< 1 signed 64-bit integer.
        kOneFloat64  = 41,  ///< 1 double-precision 64-bit floating-point number.

        // 9 bytes total
        kNineBools  = 42,  ///< An array of 9 8-bit booleans.
        kNineUint8s = 43,  ///< An array of 9 unsigned 8-bit integers.
        kNineInt8s  = 44,  ///< An array of 9 signed 8-bit integers.

        // 10 bytes total
        kTenBools    = 45,  ///< An array of 10 8-bit booleans.
        kTenUint8s   = 46,  ///< An array of 10 unsigned 8-bit integers.
        kTenInt8s    = 47,  ///< An array of 10 signed 8-bit integers.
        kFiveUint16s = 48,  ///< An array of 5 unsigned 16-bit integers.
        kFiveInt16s  = 49,  ///< An array of 5 signed 16-bit integers.

        // 11 bytes total
        kElevenBools  = 50,  ///< An array of 11 8-bit booleans.
        kElevenUint8s = 51,  ///< An array of 11 unsigned 8-bit integers.
        kElevenInt8s  = 52,  ///< An array of 11 signed 8-bit integers.

        // 12 bytes total
        kTwelveBools   = 53,  ///< An array of 12 8-bit booleans.
        kTwelveUint8s  = 54,  ///< An array of 12 unsigned 8-bit integers.
        kTwelveInt8s   = 55,  ///< An array of 12 signed 8-bit integers.
        kSixUint16s    = 56,  ///< An array of 6 unsigned 16-bit integers.
        kSixInt16s     = 57,  ///< An array of 6 signed 16-bit integers.
        kThreeUint32s  = 58,  ///< An array of 3 unsigned 32-bit integers.
        kThreeInt32s   = 59,  ///< An array of 3 signed 32-bit integers.
        kThreeFloat32s = 60,  ///< An array of 3 single-precision 32-bit floating-point numbers.

        // 13 bytes total
        kThirteenBools  = 61,  ///< An array of 13 8-bit booleans.
        kThirteenUint8s = 62,  ///< An array of 13 unsigned 8-bit integers.
        kThirteenInt8s  = 63,  ///< An array of 13 signed 8-bit integers.

        // 14 bytes total
        kFourteenBools  = 64,  ///< An array of 14 8-bit booleans.
        kFourteenUint8s = 65,  ///< An array of 14 unsigned 8-bit integers.
        kFourteenInt8s  = 66,  ///< An array of 14 signed 8-bit integers.
        kSevenUint16s   = 67,  ///< An array of 7 unsigned 16-bit integers.
        kSevenInt16s    = 68,  ///< An array of 7 signed 16-bit integers.

        // 15 bytes total
        kFifteenBools  = 69,  ///< An array of 15 8-bit booleans.
        kFifteenUint8s = 70,  ///< An array of 15 unsigned 8-bit integers.
        kFifteenInt8s  = 71,  ///< An array of 15 signed 8-bit integers.

        // 16 bytes total
        kEightUint16s = 72,  ///< An array of 8 unsigned 16-bit integers.
        kEightInt16s  = 73,  ///< An array of 8 signed 16-bit integers.
        kFourUint32s  = 74,  ///< An array of 4 unsigned 32-bit integers.
        kFourInt32s   = 75,  ///< An array of 4 signed 32-bit integers.
        kFourFloat32s = 76,  ///< An array of 4 single-precision 32-bit floating-point numbers.
        kTwoUint64s   = 77,  ///< An array of 2 unsigned 64-bit integers.
        kTwoInt64s    = 78,  ///< An array of 2 signed 64-bit integers.
        kTwoFloat64s  = 79,  ///< An array of 2 double-precision 64-bit floating-point numbers.

        // 18 bytes total
        kNineUint16s = 80,  ///< An array of 9 unsigned 16-bit integers.
        kNineInt16s  = 81,  ///< An array of 9 signed 16-bit integers.

        // 20 bytes total
        kTenUint16s   = 82,  ///< An array of 10 unsigned 16-bit integers.
        kTenInt16s    = 83,  ///< An array of 10 signed 16-bit integers.
        kFiveUint32s  = 84,  ///< An array of 5 unsigned 32-bit integers.
        kFiveInt32s   = 85,  ///< An array of 5 signed 32-bit integers.
        kFiveFloat32s = 86,  ///< An array of 5 single-precision 32-bit floating-point numbers.

        // 22 bytes total
        kElevenUint16s = 87,  ///< An array of 11 unsigned 16-bit integers.
        kElevenInt16s  = 88,  ///< An array of 11 signed 16-bit integers.

        // 24 bytes total
        kTwelveUint16s = 89,  ///< An array of 12 unsigned 16-bit integers.
        kTwelveInt16s  = 90,  ///< An array of 12 signed 16-bit integers.
        kSixUint32s    = 91,  ///< An array of 6 unsigned 32-bit integers.
        kSixInt32s     = 92,  ///< An array of 6 signed 32-bit integers.
        kSixFloat32s   = 93,  ///< An array of 6 single-precision 32-bit floating-point numbers.
        kThreeUint64s  = 94,  ///< An array of 3 unsigned 64-bit integers.
        kThreeInt64s   = 95,  ///< An array of 3 signed 64-bit integers.
        kThreeFloat64s = 96,  ///< An array of 3 double-precision 64-bit floating-point numbers.

        // 26 bytes total
        kThirteenUint16s = 97,  ///< An array of 13 unsigned 16-bit integers.
        kThirteenInt16s  = 98,  ///< An array of 13 signed 16-bit integers.

        // 28 bytes total
        kFourteenUint16s = 99,   ///< An array of 14 unsigned 16-bit integers.
        kFourteenInt16s  = 100,  ///< An array of 14 signed 16-bit integers.
        kSevenUint32s    = 101,  ///< An array of 7 unsigned 32-bit integers.
        kSevenInt32s     = 102,  ///< An array of 7 signed 32-bit integers.
        kSevenFloat32s   = 103,  ///< An array of 7 single-precision 32-bit floating-point numbers.

        // 30 bytes total
        kFifteenUint16s = 104,  ///< An array of 15 unsigned 16-bit integers.
        kFifteenInt16s  = 105,  ///< An array of 15 signed 16-bit integers.

        // 32 bytes total
        kEightUint32s  = 106,  ///< An array of 8 unsigned 32-bit integers.
        kEightInt32s   = 107,  ///< An array of 8 signed 32-bit integers.
        kEightFloat32s = 108,  ///< An array of 8 single-precision 32-bit floating-point numbers.
        kFourUint64s   = 109,  ///< An array of 4 unsigned 64-bit integers.
        kFourInt64s    = 110,  ///< An array of 4 signed 64-bit integers.
        kFourFloat64s  = 111,  ///< An array of 4 double-precision 64-bit floating-point numbers.

        // 36 bytes total
        kNineUint32s  = 112,  ///< An array of 9 unsigned 32-bit integers.
        kNineInt32s   = 113,  ///< An array of 9 signed 32-bit integers.
        kNineFloat32s = 114,  ///< An array of 9 single-precision 32-bit floating-point numbers.

        // 40 bytes total
        kTenUint32s   = 115,  ///< An array of 10 unsigned 32-bit integers.
        kTenInt32s    = 116,  ///< An array of 10 signed 32-bit integers.
        kTenFloat32s  = 117,  ///< An array of 10 single-precision 32-bit floating-point numbers.
        kFiveUint64s  = 118,  ///< An array of 5 unsigned 64-bit integers.
        kFiveInt64s   = 119,  ///< An array of 5 signed 64-bit integers.
        kFiveFloat64s = 120,  ///< An array of 5 double-precision 64-bit floating-point numbers.

        // 44 bytes total
        kElevenUint32s  = 121,  ///< An array of 11 unsigned 32-bit integers.
        kElevenInt32s   = 122,  ///< An array of 11 signed 32-bit integers.
        kElevenFloat32s = 123,  ///< An array of 11 single-precision 32-bit floating-point numbers.

        // 48 bytes total
        kTwelveUint32s  = 124,  ///< An array of 12 unsigned 32-bit integers.
        kTwelveInt32s   = 125,  ///< An array of 12 signed 32-bit integers.
        kTwelveFloat32s = 126,  ///< An array of 12 single-precision 32-bit floating-point numbers.
        kSixUint64s     = 127,  ///< An array of 6 unsigned 64-bit integers.
        kSixInt64s      = 128,  ///< An array of 6 signed 64-bit integers.
        kSixFloat64s    = 129,  ///< An array of 6 double-precision 64-bit floating-point numbers.

        // 52 bytes total
        kThirteenUint32s  = 130,  ///< An array of 13 unsigned 32-bit integers.
        kThirteenInt32s   = 131,  ///< An array of 13 signed 32-bit integers.
        kThirteenFloat32s = 132,  ///< An array of 13 single-precision 32-bit floating-point numbers.

        // 56 bytes total
        kFourteenUint32s  = 133,  ///< An array of 14 unsigned 32-bit integers.
        kFourteenInt32s   = 134,  ///< An array of 14 signed 32-bit integers.
        kFourteenFloat32s = 135,  ///< An array of 14 single-precision 32-bit floating-point numbers.
        kSevenUint64s     = 136,  ///< An array of 7 unsigned 64-bit integers.
        kSevenInt64s      = 137,  ///< An array of 7 signed 64-bit integers.
        kSevenFloat64s    = 138,  ///< An array of 7 double-precision 64-bit floating-point numbers.

        // 60 bytes total
        kFifteenUint32s  = 139,  ///< An array of 15 unsigned 32-bit integers.
        kFifteenInt32s   = 140,  ///< An array of 15 signed 32-bit integers.
        kFifteenFloat32s = 141,  ///< An array of 15 single-precision 32-bit floating-point numbers.

        // 64 bytes total
        kEightUint64s  = 142,  ///< An array of 8 unsigned 64-bit integers.
        kEightInt64s   = 143,  ///< An array of 8 signed 64-bit integers.
        kEightFloat64s = 144,  ///< An array of 8 double-precision 64-bit floating-point numbers.

        // 72 bytes total
        kNineUint64s  = 145,  ///< An array of 9 unsigned 64-bit integers.
        kNineInt64s   = 146,  ///< An array of 9 signed 64-bit integers.
        kNineFloat64s = 147,  ///< An array of 9 double-precision 64-bit floating-point numbers.

        // 80 bytes total
        kTenUint64s  = 148,  ///< An array of 10 unsigned 64-bit integers.
        kTenInt64s   = 149,  ///< An array of 10 signed 64-bit integers.
        kTenFloat64s = 150,  ///< An array of 10 double-precision 64-bit floating-point numbers.

        // 88 bytes total
        kElevenUint64s  = 151,  ///< An array of 11 unsigned 64-bit integers.
        kElevenInt64s   = 152,  ///< An array of 11 signed 64-bit integers.
        kElevenFloat64s = 153,  ///< An array of 11 double-precision 64-bit floating-point numbers.

        // 96 bytes total
        kTwelveUint64s  = 154,  ///< An array of 12 unsigned 64-bit integers.
        kTwelveInt64s   = 155,  ///< An array of 12 signed 64-bit integers.
        kTwelveFloat64s = 156,  ///< An array of 12 double-precision 64-bit floating-point numbers.

        // 104 bytes total
        kThirteenUint64s  = 157,  ///< An array of 13 unsigned 64-bit integers.
        kThirteenInt64s   = 158,  ///< An array of 13 signed 64-bit integers.
        kThirteenFloat64s = 159,  ///< An array of 13 double-precision 64-bit floating-point numbers.

        // 112 bytes total
        kFourteenUint64s  = 160,  ///< An array of 14 unsigned 64-bit integers.
        kFourteenInt64s   = 161,  ///< An array of 14 signed 64-bit integers.
        kFourteenFloat64s = 162,  ///< An array of 14 double-precision 64-bit floating-point numbers.

        // 120 bytes total
        kFifteenUint64s  = 163,  ///< An array of 15 unsigned 64-bit integers.
        kFifteenInt64s   = 164,  ///< An array of 15 signed 64-bit integers.
        kFifteenFloat64s = 165,  ///< An array of 15 double-precision 64-bit floating-point numbers.

        // Extended prototypes (codes 166-255)

        // bool extended (16 bytes to 248 bytes)
        kSixteenBools              = 166,  ///< An array of 16 8-bit booleans.
        kTwentyFourBools           = 167,  ///< An array of 24 8-bit booleans.
        kThirtyTwoBools            = 168,  ///< An array of 32 8-bit booleans.
        kFortyBools                = 169,  ///< An array of 40 8-bit booleans.
        kFortyEightBools           = 170,  ///< An array of 48 8-bit booleans.
        kFiftyTwoBools             = 171,  ///< An array of 52 8-bit booleans.
        kTwoHundredFortyEightBools = 172,  ///< An array of 248 8-bit booleans.

        // uint8_t extended (16 bytes to 248 bytes)
        kSixteenUint8s               = 173,  ///< An array of 16 unsigned 8-bit integers.
        kEighteenUint8s              = 174,  ///< An array of 18 unsigned 8-bit integers.
        kTwentyUint8s                = 175,  ///< An array of 20 unsigned 8-bit integers.
        kTwentyTwoUint8s             = 176,  ///< An array of 22 unsigned 8-bit integers.
        kTwentyFourUint8s            = 177,  ///< An array of 24 unsigned 8-bit integers.
        kTwentyEightUint8s           = 178,  ///< An array of 28 unsigned 8-bit integers.
        kThirtyTwoUint8s             = 179,  ///< An array of 32 unsigned 8-bit integers.
        kThirtySixUint8s             = 180,  ///< An array of 36 unsigned 8-bit integers.
        kFortyUint8s                 = 181,  ///< An array of 40 unsigned 8-bit integers.
        kFortyFourUint8s             = 182,  ///< An array of 44 unsigned 8-bit integers.
        kFortyEightUint8s            = 183,  ///< An array of 48 unsigned 8-bit integers.
        kFiftyTwoUint8s              = 184,  ///< An array of 52 unsigned 8-bit integers.
        kSixtyFourUint8s             = 185,  ///< An array of 64 unsigned 8-bit integers.
        kNinetySixUint8s             = 186,  ///< An array of 96 unsigned 8-bit integers.
        kOneHundredTwentyEightUint8s = 187,  ///< An array of 128 unsigned 8-bit integers.
        kOneHundredNinetyTwoUint8s   = 188,  ///< An array of 192 unsigned 8-bit integers.
        kTwoHundredFortyFourUint8s   = 189,  ///< An array of 244 unsigned 8-bit integers.
        kTwoHundredFortyEightUint8s  = 190,  ///< An array of 248 unsigned 8-bit integers.

        // int8_t extended (16 bytes to 248 bytes)
        kSixteenInt8s              = 191,  ///< An array of 16 signed 8-bit integers.
        kTwentyFourInt8s           = 192,  ///< An array of 24 signed 8-bit integers.
        kThirtyTwoInt8s            = 193,  ///< An array of 32 signed 8-bit integers.
        kFortyInt8s                = 194,  ///< An array of 40 signed 8-bit integers.
        kFortyEightInt8s           = 195,  ///< An array of 48 signed 8-bit integers.
        kFiftyTwoInt8s             = 196,  ///< An array of 52 signed 8-bit integers.
        kNinetyTwoInt8s            = 197,  ///< An array of 92 signed 8-bit integers.
        kOneHundredThirtyTwoInt8s  = 198,  ///< An array of 132 signed 8-bit integers.
        kOneHundredSeventyTwoInt8s = 199,  ///< An array of 172 signed 8-bit integers.
        kTwoHundredTwelveInt8s     = 200,  ///< An array of 212 signed 8-bit integers.
        kTwoHundredFortyFourInt8s  = 201,  ///< An array of 244 signed 8-bit integers.
        kTwoHundredFortyEightInt8s = 202,  ///< An array of 248 signed 8-bit integers.

        // uint16_t extended (32 bytes to 248 bytes)
        kSixteenUint16s              = 203,  ///< An array of 16 unsigned 16-bit integers.
        kTwentyUint16s               = 204,  ///< An array of 20 unsigned 16-bit integers.
        kTwentyFourUint16s           = 205,  ///< An array of 24 unsigned 16-bit integers.
        kTwentySixUint16s            = 206,  ///< An array of 26 unsigned 16-bit integers.
        kThirtyTwoUint16s            = 207,  ///< An array of 32 unsigned 16-bit integers.
        kFortyEightUint16s           = 208,  ///< An array of 48 unsigned 16-bit integers.
        kSixtyFourUint16s            = 209,  ///< An array of 64 unsigned 16-bit integers.
        kNinetySixUint16s            = 210,  ///< An array of 96 unsigned 16-bit integers.
        kOneHundredTwentyTwoUint16s  = 211,  ///< An array of 122 unsigned 16-bit integers.
        kOneHundredTwentyFourUint16s = 212,  ///< An array of 124 unsigned 16-bit integers.

        // int16_t extended (32 bytes to 248 bytes)
        kSixteenInt16s              = 213,  ///< An array of 16 signed 16-bit integers.
        kTwentyInt16s               = 214,  ///< An array of 20 signed 16-bit integers.
        kTwentyFourInt16s           = 215,  ///< An array of 24 signed 16-bit integers.
        kTwentySixInt16s            = 216,  ///< An array of 26 signed 16-bit integers.
        kThirtyTwoInt16s            = 217,  ///< An array of 32 signed 16-bit integers.
        kFortyEightInt16s           = 218,  ///< An array of 48 signed 16-bit integers.
        kSixtyFourInt16s            = 219,  ///< An array of 64 signed 16-bit integers.
        kNinetySixInt16s            = 220,  ///< An array of 96 signed 16-bit integers.
        kOneHundredTwentyTwoInt16s  = 221,  ///< An array of 122 signed 16-bit integers.
        kOneHundredTwentyFourInt16s = 222,  ///< An array of 124 signed 16-bit integers.

        // uint32_t extended (64 bytes to 248 bytes)
        kSixteenUint32s    = 223,  ///< An array of 16 unsigned 32-bit integers.
        kTwentyUint32s     = 224,  ///< An array of 20 unsigned 32-bit integers.
        kTwentyFourUint32s = 225,  ///< An array of 24 unsigned 32-bit integers.
        kThirtyTwoUint32s  = 226,  ///< An array of 32 unsigned 32-bit integers.
        kFortyEightUint32s = 227,  ///< An array of 48 unsigned 32-bit integers.
        kSixtyTwoUint32s   = 228,  ///< An array of 62 unsigned 32-bit integers.

        // int32_t extended (64 bytes to 248 bytes)
        kSixteenInt32s    = 229,  ///< An array of 16 signed 32-bit integers.
        kTwentyInt32s     = 230,  ///< An array of 20 signed 32-bit integers.
        kTwentyFourInt32s = 231,  ///< An array of 24 signed 32-bit integers.
        kThirtyTwoInt32s  = 232,  ///< An array of 32 signed 32-bit integers.
        kFortyEightInt32s = 233,  ///< An array of 48 signed 32-bit integers.
        kSixtyTwoInt32s   = 234,  ///< An array of 62 signed 32-bit integers.

        // float extended (64 bytes to 248 bytes)
        kSixteenFloat32s    = 235,  ///< An array of 16 single-precision 32-bit floating-point numbers.
        kTwentyFloat32s     = 236,  ///< An array of 20 single-precision 32-bit floating-point numbers.
        kTwentyFourFloat32s = 237,  ///< An array of 24 single-precision 32-bit floating-point numbers.
        kThirtyTwoFloat32s  = 238,  ///< An array of 32 single-precision 32-bit floating-point numbers.
        kFortyEightFloat32s = 239,  ///< An array of 48 single-precision 32-bit floating-point numbers.
        kSixtyTwoFloat32s   = 240,  ///< An array of 62 single-precision 32-bit floating-point numbers.

        // uint64_t extended (128 bytes to 248 bytes)
        kSixteenUint64s    = 241,  ///< An array of 16 unsigned 64-bit integers.
        kTwentyUint64s     = 242,  ///< An array of 20 unsigned 64-bit integers.
        kTwentyFourUint64s = 243,  ///< An array of 24 unsigned 64-bit integers.
        kThirtyOneUint64s  = 244,  ///< An array of 31 unsigned 64-bit integers.

        // int64_t extended (128 bytes to 248 bytes)
        kSixteenInt64s    = 245,  ///< An array of 16 signed 64-bit integers.
        kTwentyInt64s     = 246,  ///< An array of 20 signed 64-bit integers.
        kTwentyFourInt64s = 247,  ///< An array of 24 signed 64-bit integers.
        kThirtyOneInt64s  = 248,  ///< An array of 31 signed 64-bit integers.

        // double extended (128 bytes to 248 bytes)
        kSixteenFloat64s    = 249,  ///< An array of 16 double-precision 64-bit floating-point numbers.
        kTwentyFloat64s     = 250,  ///< An array of 20 double-precision 64-bit floating-point numbers.
        kTwentyFourFloat64s = 251,  ///< An array of 24 double-precision 64-bit floating-point numbers.
        kThirtyOneFloat64s  = 252,  ///< An array of 31 double-precision 64-bit floating-point numbers.
    };

    // AVR-compatible type traits for compile-time array introspection. Mirrors std:: counterparts to serve as drop-in
    // replacements on platforms that lack <type_traits>.

    /// @brief Determines whether a type is a C-style array. False by default.
    template <typename T>
    struct is_array
    {
            static constexpr bool value = false;
    };

    /// @brief Partial specialization that activates for bounded C-style arrays.
    template <typename T, size_t N>
    struct is_array<T[N]>
    {
            static constexpr bool value = true;
    };

    /// @brief Convenient variable template for is_array.
    template <typename T>
    constexpr bool is_array_v = is_array<T>::value;  // NOLINT(*-dynamic-static-initializers)

    /// @brief Retrieves the number of elements in a C-style array. Zero for non-array types.
    template <typename T>
    struct array_extent
    {
            static constexpr size_t value = 0;
    };

    /// @brief Partial specialization that retrieves the element count of a bounded C-style array.
    template <typename T, size_t N>
    struct array_extent<T[N]>
    {
            static constexpr size_t value = N;
    };

    /// @brief Convenient variable template for array_extent.
    template <typename T>
    constexpr size_t array_extent_v = array_extent<T>::value;  // NOLINT(*-dynamic-static-initializers)

    /// @brief Retrieves the element type of C-style array. Identity for non-array types.
    template <typename T>
    struct remove_extent
    {
            using type = T;
    };

    /// @brief Partial specialization that strips the array extent to yield the element type.
    template <typename T, size_t N>
    struct remove_extent<T[N]>
    {
            using type = T;
    };

    /// @brief Convenient alias template for remove_extent.
    template <typename T>
    using remove_extent_t = typename remove_extent<T>::type;

    /**
     * @brief Maps a scalar element type to a row index in the prototype lookup table.
     *
     * @tparam T The scalar type to resolve. Must be one of: bool, uint8_t, int8_t, uint16_t, int16_t, uint32_t,
     * int32_t, float, uint64_t, int64_t, or double.
     * @returns The row index (0-10) corresponding to the element type.
     */
    template <typename T>
    constexpr uint8_t PrototypeTypeIndex()
    {
        using axtlmc_shared_assets::is_same_v;

        // Validates the type to produce a clear compile error for unsupported types.
        static_assert(
            is_same_v<T, bool> || is_same_v<T, uint8_t> || is_same_v<T, int8_t> || is_same_v<T, uint16_t> ||
                is_same_v<T, int16_t> || is_same_v<T, uint32_t> || is_same_v<T, int32_t> || is_same_v<T, float> ||
                is_same_v<T, uint64_t> || is_same_v<T, int64_t> || is_same_v<T, double>,
            "Unsupported element type for prototype resolution. Supported types: bool, uint8_t, int8_t, uint16_t, "
            "int16_t, uint32_t, int32_t, float, uint64_t, int64_t, double."
        );

        // Uses a ternary chain instead of if-constexpr for compatibility with older constexpr implementations.
        return is_same_v<T, bool>     ? 0
             : is_same_v<T, uint8_t>  ? 1
             : is_same_v<T, int8_t>   ? 2
             : is_same_v<T, uint16_t> ? 3
             : is_same_v<T, int16_t>  ? 4
             : is_same_v<T, uint32_t> ? 5
             : is_same_v<T, int32_t>  ? 6
             : is_same_v<T, float>    ? 7
             : is_same_v<T, uint64_t> ? 8
             : is_same_v<T, int64_t>  ? 9
                                      : 10;  // double (guaranteed by static_assert above)
    }

    /**
     * @brief Compile-time lookup table that maps (element count, element type) to kPrototypes enum codes.
     *
     * Rows correspond to element counts 1 through 248 (index = count - 1).
     * Columns correspond to element types (see PrototypeTypeIndex): bool(0), uint8_t(1), int8_t(2),
     * uint16_t(3), int16_t(4), uint32_t(5), int32_t(6), float(7), uint64_t(8), int64_t(9), double(10).
     * A value of 0 indicates that the (type, count) pair has no assigned prototype code.
     */
    // NOLINTBEGIN(*-avoid-c-arrays, *-magic-numbers)
    constexpr uint8_t kPrototypeLookup[248][11] = {
        {1,   2,   3,   7,   8,   17,  18,  19,  39,  40,  41 }, // count = 1
        {4,   5,   6,   15,  16,  36,  37,  38,  77,  78,  79 }, // count = 2
        {9,   10,  11,  26,  27,  58,  59,  60,  94,  95,  96 }, // count = 3
        {12,  13,  14,  34,  35,  74,  75,  76,  109, 110, 111}, // count = 4
        {20,  21,  22,  48,  49,  84,  85,  86,  118, 119, 120}, // count = 5
        {23,  24,  25,  56,  57,  91,  92,  93,  127, 128, 129}, // count = 6
        {28,  29,  30,  67,  68,  101, 102, 103, 136, 137, 138}, // count = 7
        {31,  32,  33,  72,  73,  106, 107, 108, 142, 143, 144}, // count = 8
        {42,  43,  44,  80,  81,  112, 113, 114, 145, 146, 147}, // count = 9
        {45,  46,  47,  82,  83,  115, 116, 117, 148, 149, 150}, // count = 10
        {50,  51,  52,  87,  88,  121, 122, 123, 151, 152, 153}, // count = 11
        {53,  54,  55,  89,  90,  124, 125, 126, 154, 155, 156}, // count = 12
        {61,  62,  63,  97,  98,  130, 131, 132, 157, 158, 159}, // count = 13
        {64,  65,  66,  99,  100, 133, 134, 135, 160, 161, 162}, // count = 14
        {69,  70,  71,  104, 105, 139, 140, 141, 163, 164, 165}, // count = 15
        {166, 173, 191, 203, 213, 223, 229, 235, 241, 245, 249}, // count = 16
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   174, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 18
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   175, 0,   204, 214, 224, 230, 236, 242, 246, 250}, // count = 20
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   176, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 22
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {167, 177, 192, 205, 215, 225, 231, 237, 243, 247, 251}, // count = 24
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   206, 216, 0,   0,   0,   0,   0,   0  }, // count = 26
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   178, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 28
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   244, 248, 252}, // count = 31
        {168, 179, 193, 207, 217, 226, 232, 238, 0,   0,   0  }, // count = 32
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   180, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 36
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {169, 181, 194, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 40
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   182, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 44
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {170, 183, 195, 208, 218, 227, 233, 239, 0,   0,   0  }, // count = 48
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {171, 184, 196, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 52
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   228, 234, 240, 0,   0,   0  }, // count = 62
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   185, 0,   209, 219, 0,   0,   0,   0,   0,   0  }, // count = 64
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   197, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 92
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   186, 0,   210, 220, 0,   0,   0,   0,   0,   0  }, // count = 96
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   211, 221, 0,   0,   0,   0,   0,   0  }, // count = 122
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   212, 222, 0,   0,   0,   0,   0,   0  }, // count = 124
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   187, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 128
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   198, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 132
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   199, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 172
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   188, 0,   0,   0,   0,   0,   0,   0,   0,   0  }, // count = 192
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   200, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 212
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   189, 201, 0,   0,   0,   0,   0,   0,   0,   0  }, // count = 244
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  },
        {172, 190, 202, 0,   0,   0,   0,   0,   0,   0,   0  }  // count = 248
    };

    // NOLINTEND(*-avoid-c-arrays, *-magic-numbers)

    /**
     * @brief Resolves the kPrototypes enum value for the given C++ type at compile time.
     *
     * Supports all 11 scalar types (bool through double) and C-style arrays of those types at
     * supported element counts up to the 248-byte payload cap. A lookup result of 0 indicates an
     * unsupported (type, count) pair and triggers a static_assert. uint8_t arrays offer the densest
     * count coverage and can serve as a generic bytes buffer for sending arbitrary packed structures.
     *
     * @tparam ObjectType The type of the data object to resolve. Must be a supported scalar or a
     * bounded C-style array of a supported scalar at a supported element count.
     * @returns The kPrototypes enum value corresponding to the object type.
     */
    template <typename ObjectType>
    constexpr kPrototypes ResolvePrototype()
    {
        // Validates that the (element type, count) pair has a registered prototype code.
        static_assert(
            !is_array_v<ObjectType> ||
                (array_extent_v<ObjectType> >= 1 && array_extent_v<ObjectType> <= 248 &&
                 kPrototypeLookup[array_extent_v<ObjectType> - 1][PrototypeTypeIndex<remove_extent_t<ObjectType>>()] !=
                     0),
            "Unsupported array element count for this type. "
            "Use a supported count or cast to uint8_t[N]."
        );

        // For arrays, remove_extent_t yields the element type and array_extent_v the count.
        // For scalars, remove_extent_t is the identity and count defaults to 1.
        return static_cast<kPrototypes>(kPrototypeLookup[(is_array_v<ObjectType> ? array_extent_v<ObjectType> : 1) - 1]
                                                        [PrototypeTypeIndex<remove_extent_t<ObjectType>>()]);
    }

    /**
     * @struct RepeatedModuleCommand
     * @brief Instructs the addressed Module instance to run the specified command repeatedly (recurrently).
     */
    struct RepeatedModuleCommand
    {
            uint8_t module_type  = 0;      ///< The type (family) code of the module to which the command is addressed.
            uint8_t module_id    = 0;      ///< The ID of the specific module instance within the broader module family.
            uint8_t return_code  = 0;      ///< The acknowledgment code for the message, if set to a non-zero value.
            uint8_t command      = 0;      ///< The code of the command to execute.
            bool noblock         = false;  ///< Determines whether to allow concurrent execution of other commands.
            uint32_t cycle_delay = 0;      ///< The delay, in microseconds, before repeating (cycling) the command.
    } PACKED_STRUCT;

    /**
     * @struct OneOffModuleCommand
     * @brief Instructs the addressed Module instance to run the specified command exactly once (non-recurrently).
     */
    struct OneOffModuleCommand
    {
            uint8_t module_type = 0;      ///< The type (family) code of the module to which the command is addressed.
            uint8_t module_id   = 0;      ///< The ID of the specific module instance within the broader module family.
            uint8_t return_code = 0;      ///< The acknowledgment code for the message, if set to a non-zero value.
            uint8_t command     = 0;      ///< The code of the command to execute.
            bool noblock        = false;  ///< Determines whether to allow concurrent execution of other commands.
    } PACKED_STRUCT;

    /**
     * @struct DequeueModuleCommand
     * @brief Instructs the addressed Module instance to clear (empty) its command queue.
     */
    struct DequeueModuleCommand
    {
            uint8_t module_type = 0;  ///< The type (family) code of the module to which the command is addressed.
            uint8_t module_id   = 0;  ///< The ID of the specific module instance within the broader module family.
            uint8_t return_code = 0;  ///< The acknowledgment code for the message, if set to a non-zero value.
    } PACKED_STRUCT;

    /**
     * @struct KernelCommand
     * @brief Instructs the Kernel to run the specified command exactly once.
     */
    struct KernelCommand
    {
            uint8_t return_code = 0;  ///< The acknowledgment code for the message, if set to a non-zero value.
            uint8_t command     = 0;  ///< The code of the command to execute.
    } PACKED_STRUCT;

    /**
     * @struct ModuleParameters
     * @brief Instructs the addressed Module instance to update its parameters with the included data.
     */
    struct ModuleParameters
    {
            uint8_t module_type = 0;  ///< The type (family) code of the module to which parameters are addressed.
            uint8_t module_id   = 0;  ///< The ID of the specific module instance within the broader module family.
            uint8_t return_code = 0;  ///< The acknowledgment code for the message, if set to a non-zero value.
    } PACKED_STRUCT;

    /**
     * @struct ModuleData
     * @brief Communicates that the Module has encountered a notable event and includes an additional data object.
     *
     * @note Use the ModuleState structure for messages that only need to transmit an event state-code.
     */
    struct ModuleData
    {
            uint8_t protocol;     ///< The message protocol used by this structure.
            uint8_t module_type;  ///< The type (family) code of the module that sent the data message.
            uint8_t module_id;    ///< The ID of the specific module instance within the broader module family.
            uint8_t command;      ///< The command the Module was executing when it sent the data message.
            uint8_t event;        ///< The event that prompted the data transmission.
            uint8_t prototype;    ///< The prototype code for the data object transmitted with the message.
    } PACKED_STRUCT;

    /**
     * @struct KernelData
     * @brief Communicates that the Kernel has encountered a notable event and includes an additional data object.
     *
     * @note Use the KernelState structure for messages that only need to transmit the state event-code.
     */
    struct KernelData
    {
            uint8_t protocol;   ///< The message protocol used by this structure.
            uint8_t command;    ///< The command the Kernel was executing when it sent the data message.
            uint8_t event;      ///< The event that prompted the data transmission.
            uint8_t prototype;  ///< The prototype code for the data object transmitted with the message.
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
            uint8_t protocol;     ///< The message protocol used by this structure.
            uint8_t module_type;  ///< The type (family) code of the module that sent the data message.
            uint8_t module_id;    ///< The ID of the specific module instance within the broader module family.
            uint8_t command;      ///< The command the Module was executing when it sent the data message.
            uint8_t event;        ///< The event that prompted the data transmission.
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
            uint8_t protocol;  ///< The message protocol used by this structure.
            uint8_t command;   ///< The command the Kernel was executing when it sent the data message.
            uint8_t event;     ///< The event that prompted the data transmission.
    } PACKED_STRUCT;

}  // namespace axmc_communication_assets

#endif  //AXMC_SHARED_ASSETS_H
