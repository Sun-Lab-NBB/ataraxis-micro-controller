/**
 * @file
 * @brief The header-only file that stores all assets intended to be shared between ataraxis-transport-layer library
 * classes wrapped into a common namespace.
 *
 * @section axtl_sa_description Description:
 *
 * This file aggregates all general assets that have to be shared between multiple classes of the
 * ataraxis-transport-layer library.
 *
 * This file contains:
 * - kCOBSProcessorCodes structure that stores status bytecodes used by the COBSProcessor to report it's runtime status.
 * - kCRCProcessorCodes structure that stores status bytecodes used by the CRCProcessor to report it's runtime status.
 * - kTLStatusCodes structure that stores status bytecodes used by the TransportLayer to report it's runtime status.
 * - A custom implementation of the is_same_v() standard method used to verify that two input objects have the same
 * type.
 *
 * @section axtl_sa_developer_notes Developer Notes:
 *
 * The primary reason for having this file is to store all byte-code enumerations in the same place. To simplify
 * error handling, all codes available through this namespace have to be unique relative to each other. For example, if
 * the value 11 is used to represent 'Standby' state for CRCProcessor, no other status should be using the value 11).
 *
 * @section axtl_sa_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 */

#ifndef AXTL_SHARED_ASSETS_H
#define AXTL_SHARED_ASSETS_H

#include <Arduino.h>

/**
 * @namespace axtl_shared_assets
 * @brief Provides all assets (structures, enumerations, functions) that are intended to be shared between the classes
 * of the ataraxis-transport-layer library.
 *
 * The shared assets are primarily used to simplify library development by storing co-dependent assets in the same
 * place. Additionally, it simplifies using these assets with template classes from the library.
 */
namespace axtl_shared_assets
{
    /**
     * @enum kCOBSProcessorCodes
     * @brief Assigns meaningful names to all status codes used by the COBSProcessor class.
     *
     * @note Due to a unified approach to status-coding in this library, this enumeration should only use code values in
     * the range of 11 through 50. This is to simplify chained error handling in the TransportLayer class of the
     * library.
     */
    enum class kCOBSProcessorCodes : uint8_t
    {
        kStandby                       = 11,  ///< The value used to initialize the cobs_status variable
        kEncoderTooSmallPayloadSize    = 12,  ///< Encoder failed to encode payload because payload size is too small
        kEncoderTooLargePayloadSize    = 13,  ///< Encoder failed to encode payload because payload size is too large
        kEncoderPacketLargerThanBuffer = 14,  ///< Encoded payload buffer is too small to accommodate the packet
        kPayloadAlreadyEncoded         = 15,  ///< Cannot encode payload as it is already encoded (overhead value != 0)
        kPayloadEncoded                = 16,  ///< Payload was successfully encoded into a transmittable packet
        kDecoderTooSmallPacketSize     = 17,  ///< Decoder failed to decode packet because packet size is too small
        kDecoderTooLargePacketSize     = 18,  ///< Decoder failed to decode packet because packet size is too large
        kDecoderPacketLargerThanBuffer = 19,  ///< Packet size to be decoded is larger than the storage buffer size
        kDecoderUnableToFindDelimiter  = 20,  ///< Decoder failed to find the delimiter at the end of the packet
        kDecoderDelimiterFoundTooEarly = 21,  ///< Decoder found a delimiter before reaching the end of the packet
        kPacketAlreadyDecoded          = 22,  ///< Cannot decode packet as it is already decoded (overhead value == 0)
        kPayloadDecoded                = 23,  ///< Payload was successfully decoded from the received packet
    };

    /**
     * @enum kCRCProcessorCodes
     * @brief Assigns meaningful names to all status codes used by the CRCProcessor class.
     *
     * @note Due to a unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 51 through 100. This is to simplify chained error handling in the
     * TransportLayer class of the library.
     */
    enum class kCRCProcessorCodes : uint8_t
    {
        kStandby                            = 51,  ///< The value used to initialize the crc_status variable
        kCalculateCRCChecksumBufferTooSmall = 52,  ///< Checksum calculator failed due to packet exceeding buffer space
        kCRCChecksumCalculated              = 53,  ///< Checksum was successfully calculated
        kAddCRCChecksumBufferTooSmall       = 54,  ///< Not enough remaining buffer space to add checksum to buffer
        kCRCChecksumAddedToBuffer           = 55,  ///< Checksum was successfully added to the buffer
        kReadCRCChecksumBufferTooSmall      = 56,  ///< Not enough remaining space inside buffer to get checksum from it
        kCRCChecksumReadFromBuffer          = 57,  ///< Checksum was successfully read from the buffer
    };

    /**
     * @enum kTransportLayerStatusCodes
     * @brief Assigns meaningful names to all status codes used by the TransportLayer class.
     *
     * @note Due to a unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 101 through 150. This is to simplify chained error handling in the
     * TransportLayer class of the library.
     */
    enum class kTransportLayerStatusCodes : uint8_t
    {
        kStandby                      = 101,  ///< The default value used to initialize the transfer_status variable
        kPacketConstructed            = 102,  ///< Packet was sucessfully constructed
        kPacketSent                   = 103,  ///< Packet was successfully transmitted
        kPacketStartByteFound         = 104,  ///< Packet start byte was found
        kPacketStartByteNotFoundError = 105,  ///< Packet start byte was not found in the incoming stream
        kPayloadSizeByteFound         = 106,  ///< Payload size byte was found
        kPayloadSizeByteNotFound      = 107,  ///< Payload size byte was not found in the incoming stream
        kInvalidPayloadSize           = 108,  ///< Received payload size is not valid
        kPacketTimeoutError           = 109,  ///< Packet parsing failed due to stalling (reception timeout)
        kNoBytesToParseFromBuffer     = 110,  ///< Stream class reception buffer had no packet bytes to parse
        kPacketParsed                 = 111,  ///< Packet was successfully parsed
        kCRCCheckFailed               = 112,  ///< CRC check failed, incoming packet corrupted
        kPacketValidated              = 113,  ///< Packet was successfully validated
        kPacketReceived               = 114,  ///< Packet was successfully received
        kWriteObjectBufferError       = 115,  ///< Not enough space in the buffer payload region to write the object
        kObjectWrittenToBuffer        = 116,  ///< The object has been written to the buffer
        kReadObjectBufferError        = 117,  ///< Not enough bytes in the buffer payload region to read the object from
        kObjectReadFromBuffer         = 118,  ///< The object has been read from the buffer
    };

    // Since Arduino Mega (the lower-end board this code was tested with) boards do not have access to 'cstring' header
    // that is available to Teensy, some useful assets have to be reimplemented manually. They are implemented in as
    // similar of a way as possible to be drop-in replaceable with std:: namespace.

    /**
     * @brief A type trait that determines if two types are the same.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     *
     * This struct is used to compare two types at compile-time. It defines a static constant member `value` which is
     * set to `false` by default, indicating that the two types are not the same.
     */
    template <typename T, typename U>
    struct is_same
    {
        /// The default value used by the specification for two different input types.
        static constexpr bool value = false;
    };

    /**
      * @brief Specialization of is_same for the case when both types are the same.
      *
      * @tparam T The type to compare.
      *
      * This specialization is used when both type parameters are the same. In this case, the static constant member
      * `value` is set to `true`, indicating that the types are indeed the same.
      */
    template <typename T>
    struct is_same<T, T>
    {
        /// The default value used by the specification for two identical types.
        static constexpr bool value = true;
    };

    /**
     * @brief A helper variable template that provides a convenient way to access the value of is_same.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     *
     * This variable template is declared as `constexpr`, allowing it to be used in compile-time expressions. It
     * provides a more concise way to check if two types are the same, without the need to explicitly access the
     * `value` member of the `is_same` struct.
     *
     * Example usage:
     * @code
     * static_assert(is_same_v<int, int>, "int and int are the same");
     * static_assert(!is_same_v<int, float>, "int and float are not the same");
     * @endcode
     */
    template <typename T, typename U>
    constexpr bool is_same_v = is_same<T, U>::value;  // NOLINT(*-dynamic-static-initializers)

}  // namespace axtl_shared_assets

#endif  //AXTL_SHARED_ASSETS_H