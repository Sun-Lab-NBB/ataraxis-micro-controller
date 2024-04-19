/**
 * @file
 * @brief The header-only file that stores all assets intended to be shared between SerializedTransferProtocol library
 * classes wrapped into a common namespace.
 *
 * @subsection description Description:
 *
 * This file aggregates all general assets that have to be shared between multiple classes of the
 * SerializedTransferProtocol library.
 *
 * This file contains:
 * - kCOBSProcessorCodes structure that stores status bytecodes used by the COBSProcessor to report it's runtime status.
 * - kCRCProcessorCodes structure that stores status bytecodes used by the CRCProcessor to report it's runtime status.
 *
 * @subsection developer_notes Developer Notes:
 *
 * The primary reason for having this file is to store all byte-code enumerations in the same place. To simplify
 * error handling, all codes available through this namespace have to be unique relative to each other (that is, if the
 * value 11 is used to represent 'Standby' state for CRCProcessor, no other status should be using the value 11).
 */

#ifndef AMC_STP_SHARED_ASSETS_H
#define AMC_STP_SHARED_ASSETS_H

/**
 * @namespace stp_shared_assets
 * @brief Provides all assets (structures, enumerations, functions) that are intended to be shared between the classes
 * of the SerializedTransferProtocol library.
 *
 * The shared assets are primarily used to simplify library development by storing co-dependent assets in the
 * same place. Additionally, it simplifies using these assets with template classes (currently only CRCProcessor) from
 * the library.
 */
namespace stp_shared_assets
{
    /**
     * @enum kCOBSProcessorCodes
     * @brief Assigns meaningful names to all status codes used by the COBSProcessor class.
     *
     * @note Due to a unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 51 through 100. This is to simplify chained error handling in the
     * SerializedTransferProtocol class of the library.
     */
    enum class kCOBSProcessorCodes : uint8_t
    {
        kStandby                       = 11,  ///< The value used to initialize the cobs_status variable
        kEncoderTooSmallPayloadSize    = 12,  ///< Encoder failed to encode payload because payload size is too small
        kEncoderTooLargePayloadSize    = 13,  ///< Encoder failed to encode payload because payload size is too large
        kEncoderPacketLargerThanBuffer = 14,  ///< Encoded payload buffer is too small to accommodate the packet
        kPayloadAlreadyEncoded         = 15,  ///< Cannot encode payload as it is already encoded (overhead != 0)
        kPayloadEncoded                = 16,  ///< Payload was successfully encoded into a transmission packet
        kDecoderTooSmallPacketSize     = 17,  ///< Decoder failed to decode packet because packet size is too small
        kDecoderTooLargePacketSize     = 18,  ///< Decoder failed to decode packet because packet size is too large
        kDecoderPacketLargerThanBuffer = 19,  ///< Decoded packet size is larger than the buffer size (would not fit)
        kDecoderUnableToFindDelimiter  = 20,  ///< Decoder failed to find the delimiter at the end of the packet
        kDecoderDelimiterFoundTooEarly = 21,  ///< Decoder found a delimiter before reaching the end of the packet
        kPacketAlreadyDecoded          = 22,  ///< Cannot decode packet as it is already decoded (overhead == 0)
        kPayloadDecoded                = 23,  ///< Payload was successfully decoded from he transmission packet
    };

    /**
     * @enum kCRCProcessorCodes
     * @brief Assigns meaningful names to all status codes used by the CRCProcessor class.
     *
     * @note Due to a unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 51 through 100. This is to simplify chained error handling in the
     * SerializedTransferProtocol class of the library.
     */
    enum class kCRCProcessorCodes : uint8_t
    {
        kStandby                            = 51,  ///< The value used to initialize the crc_status variable
        kCalculateCRCChecksumBufferTooSmall = 52,  ///< Checksum calculator failed due to packet exceeding buffer space
        kCRCChecksumCalculated              = 53,  ///< Checksum was successfully calculated
        kAddCRCChecksumBufferTooSmall       = 54,  ///< Not enough remaining space inside buffer to add checksum to it
        kCRCChecksumAddedToBuffer           = 55,  ///< Checksum was successfully added to the buffer
        kReadCRCChecksumBufferTooSmall      = 56,  ///< Not enough remaining space inside buffer to get checksum from it
        kCRCChecksumReadFromBuffer          = 57,  ///< Checksum was successfully read from the buffer
    };

    /**
     * @enum kSerializedTransferProtocolStatusCodes
     * @brief Assigns meaningful names to all status codes used by the SerializedTransferProtocol class.
     *
     * @note Due to a unified approach to error-code handling in this library, this enumeration should only use code
     * values in the range of 101 through 150. This is to simplify chained error handling in the
     * SerializedTransferProtocol class of the library.
     */
    enum class kSerializedTransferProtocolStatusCodes : uint8_t
    {
        kStandby                      = 101,  ///< The default value used to initialize the transfer_status variable
        kPacketConstructed            = 102,  ///< Packet construction succeeded
        kPacketSent                   = 103,  ///< Packet transmission succeeded
        kPacketStartByteFound         = 104,  ///< Packet start byte was found
        kPacketStartByteNotFoundError = 105,  ///< Packet start byte was not found in the incoming stream
        kPacketDelimiterByteFound     = 106,  ///< Packet delimiter byte was found
        kPacketOutOfBufferSpaceError  = 107,  ///< Packet delimiter byte was not found before using up all buffer space
        kPacketTimeoutError           = 108,  ///< Packet parsing failed due to stalling (reception timeout)
        kPostambleTimeoutError        = 109,  ///< Postamble parsing failed due to staling (reception timeout)
        kPacketParsed                 = 110,  ///< Packet parsing succeeded
        kCRCCheckFailed               = 111,  ///< CRC check failed, incoming packet corrupted
        kPacketValidated              = 112,  ///< Packet validation succeeded
        kPacketReceived               = 113,  ///< Packet reception succeeded
        kWritePayloadTooSmallError    = 114,  ///< Writing to buffer failed due to not enough payload space
        kBytesWrittenToBuffer         = 115,  ///< Writing to buffer succeeded
        kReadPayloadTooSmallError     = 116,  ///< Reading from buffer failed due to not enough payload size
        kBytesReadFromBuffer          = 117,  ///< Reading from buffer succeeded
        kNoBytesToParseFromBuffer     = 118   ///< Stream class reception buffer had no packet bytes to parse
    };

    // Since Arduino Uno (the lower-end board this code was tested with) boards do not get access to 'cstring' header
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
    struct is_same {
        static const bool value = false;
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
    struct is_same<T, T> {
        static const bool value = true;
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
    constexpr bool is_same_v = is_same<T, U>::value;

}  // namespace stp_shared_assets

#endif  //AMC_STP_SHARED_ASSETS_H
