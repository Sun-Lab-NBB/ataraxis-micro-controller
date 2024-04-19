/**
 * @file
 * @brief The header file for the COBSProcessor class, which is used to encode and decode data arrays (buffers) using
 * Consistent Overhead Byte Stuffing (COBS) scheme.
 *
 * @subsection description Description:
 * COBS is a widely used byte-stuffing protocol that ensures a particular byte value is never present in the input data
 * array (payload). In the broader scope of serial communication, COBS is used to force a particular byte value, known
 * as packet delimiter, to only be present at specific points of the transmitted packets, making it suitable for
 * reliably separating (delimiting) packets.
 *
 * For COBS definition, see the original paper:
 * S. Cheshire and M. Baker, "Consistent overhead byte stuffing," in IEEE/ACM Transactions on Networking, vol. 7,
 * no. 2, pp. 159-172, April 1999, doi: @a 10.1109/90.769765.
 *
 * This file contains two methods packaged into the COBSProcessor class namespace:
 * - EncodePayload(): takes in the buffer containing an arbitrary sized payload between 1 and 254 bytes and encodes
 * it using COBS.
 * - DecodePayload(): takes in the buffer that contains a COBS-encoded payload, together with the prepended overhead
 * byte value and ending with the appended delimiter value, and restores it to the un-encoded state (decodes it).
 * - kCOBSProcessorParameters structure that is used to statically configure class behavior.
 *
 * @subsection developer_notes Developer Notes:
 * This class is a helper class that is used by the main SerializedTransferProtocol class to encode and decode payloads
 * using COBS scheme. It is not meant to be used on its own and should always be called from the SerializedTransferProtocol
 * class. The class methods expect particularly formatted and organized inputs to function properly and will behave
 * unexpectedly if input expectations are not met.
 *
 * The decoder method of the class contains two payload integrity checks, which are intended to be a fallback for the
 * CRC checker. Between the CRC and COBS decoding verification, it should be very unlikely that data corruption during
 * transmission is not noticed and reported to caller. Since this codebase is designed primarily for science
 * applications, data integrity is a major focus for all classes in this library.
 *
 * @note Due to the limitations of transmitting data as byte-values and COBS specifications, the maximum payload size
 * the class can handle is 254 bytes. The payload buffer itself is expected to accommodate at most 256 bytes to account
 * for the overhead byte and the delimiter byte, in addition to the 254 data-bytes of the payload. See method
 * descriptions below for more information. The limitations only impose the maximum size, the payloads can be less than
 * 254 bytes if this si appropriate for your specific use case.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - stdint.h for fixed-width integer types.
 * - stp_shared_assets.h For COBS-related status codes.
 *
 * @attention Due to the use of templates as a much safer approach to array operations, this class only exists as an
 * .h file with no matching .cpp file. While templates can be defined in a .tpp, for embedded programming it is easier
 * to use the .h file.
 */

#ifndef AMC_COBS_PROCESSOR_H
#define AMC_COBS_PROCESSOR_H

//Dependencies
#include "Arduino.h"
#include "stp_shared_assets.h"

/**
 * @class COBSProcessor
 *
 * @brief Provides methods for in-place encoding and decoding input payload arrays with sizes ranging from 1 to 254
 * bytes using the input delimiter byte value and payload_size.
 *
 * @attention This class assumes that the input buffer that holds the data to be processed reserves space for the
 * overhead byte value at index 0 and the delimiter byte value at index payload_size + 1. Given the overall constraint
 * of using COBS to encode 8-bit data, that only allows payload sizes of 254 bytes at maximum. This means that the input
 * buffer at a maximum needs to be of size 254+2 == 256 bytes. Ths reserves index 0 for overhead byte and index 255 for
 * delimiter byte.
 *
 * @note Do not use this class outside of the SerializedTransferProtocol class unless you know what you are doing, as it is
 * the job of the SerializedTransferProtocol class to make sure the data buffer(s) used by the methods of this class are
 * configured appropriately. If buffers are not appropriately sized, this can lead to undefined behavior and memory
 * corruption.
 *
 * Example instantiation:
 * @code
 * COBSProcessor cobs_class;
 * @endcode
 */
class COBSProcessor
{
  public:

    /// Stores the latest runtime status of the COBSProcessor. This variable is primarily designed to communicate
    /// the specific errors encountered during encoding or decoding in the form of byte-codes taken from the
    /// kCOBSProcessorCodes enumeration (available through stp_shared_assets namespace). Use the communicated status to
    /// precisely determine the runtime status of any class method.
    uint8_t cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kStandby);

    /**
     * @struct kCOBSProcessorParameters
     * @brief Stores hardcoded COBS encoding parameters that specify packet and payload size limits
     *
     * These parameters are mostly used for error-checking inputs to COBS processing methods in an effort to minimize
     * the potential to generate invalid packets.
     *
     * @attnetion It is generally not recommended to change these parameters as they are currently configured to allow
     * any valid input to be COBS-encoded. These parameter only control maximum and minimum input sizes, within these
     * limits the input can be of any supported size. The input itself can be modified through configuring appropriate
     * SerializedTransferProtocol parameters.
     */
    struct kCOBSProcessorParameters
    {
        static constexpr uint8_t kMinPayloadSize = 1;    ///< Prevents sending empty payloads
        static constexpr uint8_t kMaxPayloadSize = 254;  ///< Maximum payload size is 255 - 1 due to COBS encoding
        static constexpr uint8_t kMinPacketSize  = 3;    ///< Minimum packet size is overhead + delimiter + payload byte
        static constexpr uint16_t kMaxPacketSize = 256;  ///< Maximum packet size is maximum payload size + 2
    };

    /**
     * @brief Encodes the input payload according to COBS scheme in-place.
     *
     * Specifically, loops over the payload stored in the input payload_buffer and replaces every instance of the input
     * delimiter_byte_value with the distance to the next delimiter_byte_value or the end of the payload.
     *
     * Updates the overhead byte expected to be found under the index 0 of the input payload_buffer to store the
     * distance to the nearest delimiter_byte_value and inserts an unencoded delimiter_byte_value at the end of the
     * payload (last index of the payload + 1).
     *
     * In so-doing, implements a classic COBS encoding scheme as described by the original paper. Uses a modern approach
     * of backward-looping over the payload with project-specific heuristics. Instead of dynamically recreating the
     * buffer, works in-place and assumes the buffer is already configured in a way that supports in-place
     * COBS-encoding.
     *
     * @warning Expects the overhead byte placeholder of the input buffer to be set to 0. Otherwise, considers the call
     * an attempt to encode and already encoded data and aborts with an error to prevent data corruption from
     * re-encoding.
     *
     * @attention Assumes that the input payload_buffer is organized in a way that index 0 is reserved for the overhead
     * byte, payload is found between indices 1 and 254 (maximum payload size is 254, but it can be less) and index
     * payload_size + 1 (maximum 255) is reserved for the delimiter byte. If this assumption is violated, the method
     * will behave unexpectedly and may corrupt your data.
     *
     * @note To ensure assumptions are met, this method is not intended to be called directly. Instead, it should be
     * called by the SerializedTransferProtocol class.
     *
     * @tparam buffer_size The size of the input buffer array. This size is used to ensure any memory modifications
     * carried out using the buffer stay within the bounds of the buffer. This prevents undefined behavior and / or data
     * corruption due to modifying memory allocated for other objects.
     * @param payload_buffer The buffer (array) that holds the payload to be encoded. Assumes the array reserves index 0
     * for storing the overhead byte value and payload_size + 1 index for storing the delimiter byte value. Has to be at
     * least payload_size + 2 bytes in size.
     * @param payload_size The size of the payload in bytes. Should be between 1 and 254 inclusive. Unlike many array
     * operations, this does NOT assume the payload is found starting with index 0, but rather assumes starting
     * index is 1.
     * @param delimiter_byte_value The value between 0 and 255 (any value that fits into uint8_t range) that will be
     * used as packet delimiter. All instances of this value inside the input payload will be eliminated as per COBS
     * scheme. It is highly advised to use the value of 0 (0x00), since this is the only value that the overhead byte
     * cannot be set to. Any other value runs the risk of being present both at the end of the encoded packet
     * (delimiter byte) and the overhead byte position. This library as a whole is designed to work around this
     * potential discrepancy, but this may be problematic if COBSProcessor class is combined with other libraries.
     *
     * @returns uint16_t The size of the encoded packet in bytes, which includes the overhead byte value and the
     * delimiter byte value. Successful method runtimes return payload_size + 2, while failed runtimes return 0. Use
     * cobs_status class variable to obtain specific runtime error code if the method fails (can be interpreted by using
     * kCOBSProcessorCodes enumeration available through stp_shared_assets namespace).
     *
     * Example usage:
     * @code
     * COBSProcessor cobs_class;
     * uint8_t payload_buffer[7] = {0, 1, 2, 3, 4, 0};
     * uint16_t payload_size = 4;
     * uint8_t delimiter_byte_value = 0;
     * uint16_t packet_size = cobs_class.EncodePayload(payload_buffer, payload_size, delimiter_byte_value);
     * @endcode
     */
    template <size_t buffer_size>
    uint16_t EncodePayload(
        uint8_t (&payload_buffer)[buffer_size],
        const uint8_t payload_size,
        const uint8_t delimiter_byte_value
    )
    {
        // Prevents encoding empty payloads (as it is generally meaningless)
        if (payload_size < static_cast<uint8_t>(kCOBSProcessorParameters::kMinPayloadSize))
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kEncoderTooSmallPayloadSize);
            return 0;
        }

        // Prevents encoding too large payloads (due to COBS limitations)
        else if (payload_size > static_cast<uint8_t>(kCOBSProcessorParameters::kMaxPayloadSize))
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kEncoderTooLargePayloadSize);
            return 0;
        }

        // Prevents encoding if the input buffer size is not enough to accommodate the packet that would be
        // generated by this method. This guards against out-of-bounds memory access.
        if (static_cast<uint16_t>(buffer_size) < static_cast<uint16_t>(payload_size + 2))
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kEncoderPacketLargerThanBuffer);
            return 0;
        }

        // Checks that the input buffer's overhead byte placeholder is set to 0, which is not a valid value for an
        // encoded buffer's overhead byte, so an overhead byte set to 0 indicates that the payload inside the buffer is
        // not encoded. This check is used to prevent accidentally running the encoder on already encoded data, which
        // will corrupt it.
        if (payload_buffer[0] != 0)
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kPayloadAlreadyEncoded);
            return 0;
        }

        // Tracks discovered delimiter_byte_value instances during the loop below to support iterative COBS encoding.
        uint8_t last_delimiter_index = 0;

        // Determines start and end indices for the loop below based on the requested payload_size.
        // Sets payload_end_index to payload_size rather than payload_size-1 to account for the fact that the buffer
        // always comes with index 0 reserved for overhead buffer. This means that its length is always at least
        // payload_size+1, making the last index of the payload == payload_size and not payload_size-1.
        uint8_t payload_end_index   = payload_size;
        uint8_t payload_start_index = 1;  // payload start index is always 1 because of the overhead byte

        // Appends the delimiter_byte_value to the end of the payload buffer. Usually, this step is carried out at the
        // end of the encoding sequence, but since this method uses a reverse loop, it starts with the newly added
        // delimiter byte.
        payload_buffer[payload_end_index + 1] = delimiter_byte_value;

        // Loops over the requested payload size in reverse and encodes all instances of delimiter_byte using COBS
        // scheme. Specifically, transforms every instance of delimiter_byte_value into a chain of distance-pointers
        // that allow to traverse from the prepended overhead_byte to the appended delimiter_byte. To enable this,
        // overhead_byte stores the distance to the first delimiter_byte_value variable, which then is converted to
        // store the distance to the next delimiter_byte_value, all the way until the appended delimiter_byte is
        // reached. This way, the only instance of delimiter_byte_value will be found at the very end of the payload
        // (and the overhead byte, at worst, will store the distance of 255, to point straight to that value).
        for (uint8_t i = payload_end_index; i >= payload_start_index; i--)
        {
            if (payload_buffer[i] == delimiter_byte_value)
            {
                if (last_delimiter_index == 0)
                {
                    // If delimiter_byte_value is encountered and last_delimiter_index is still set to the default value
                    // of 0, computes the distance from index i to the end of the payload + 1. This is the distance to
                    // the delimiter byte value appended to the end of the payload. Overwrites the variable with the
                    // computed distance, encoding it according to the COBS scheme.
                    payload_buffer[i] = (payload_end_index + 1) - i;
                }
                else
                {
                    // If last_delimiter_index is set to a non-0 value, uses it to calculate the distance from the
                    // evaluated index to the last (encoded) delimiter byte value and overwrites the variable with that
                    // distance value.
                    payload_buffer[i] = last_delimiter_index - i;
                }

                // Updates last_delimiter_index with the index of the last encoded variable
                last_delimiter_index = i;
            }
        }

        // Once all instances of delimiter_byte_value have been encoded, sets the overhead byte (index 0 of buffer) to
        // store the distance to the closest delimiter_byte_value instance. That is either the last_delimiter_index if
        // it is not 0 or payload_end_index+1 if it is 0 (then overhead byte stores the distance to the appended
        // delimiter_byte_value).
        if (last_delimiter_index != 0)
            payload_buffer[0] = last_delimiter_index;  // Technically last_delimiter_index - 0 to convert to distance
        else payload_buffer[0] = payload_end_index + 1;

        // Sets the status to indicate that encoding was successful.
        cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kPayloadEncoded);

        // Returns the size of the payload accounting for the addition of the overhead byte and the delimiter byte to
        // flank the payload. Once this method is done running, the payload looks like this:
        // [overhead byte] ... [payload] ... [delimiter byte]. A maximum total size of this construct is 256 bytes.
        return static_cast<uint16_t>(payload_size + 2);
    }

    /**
     * @brief Decodes the input packet using COBS scheme in-place.
     *
     * Specifically, uses COBS-derived heuristics to find and decode all values that were encoded by a COBS encoding
     * scheme. To do so, finds the overhead_byte assumed to be located under index 0 of the input payload_buffer and
     * uses it to quickly traverse the payload by jumping across the distances encoded by each successively sampled
     * variable. During this process, replaces each traversed variable with the input delimiter_byte_value. This process
     * is carried out until the method encounters an unencoded delimiter_byte_value, at which point it, by definition,
     * has reached the end of the buffer (provided the buffer has been correctly COBS-encoded).
     *
     * The method accepts packet_size as an input and uses it to ensure that the method aborts if it is not able to
     * find delimiter_byte at the end of the packet. If this happens, that indicates the data is corrupted and the CRC
     * check (expected to be used together with COBS encoding) failed to recognize the error. As such, this method
     * doubles as a data corruption checker and, together with the CRC check, it makes uncaught data corruption
     * extremely unlikely.
     *
     * @warning Expects the input packet's overhead byte to be set to a value other than 0. If it is set to 0, the
     * method interprets this call as an attempt to decode an already decoded data and aborts with an error to prevent
     * data corruption. The method sets the overhead byte of any validly encoded packet to 0 before entering the
     * decoding loop (regardless of decoding result) to indicate decoding has been attempted.
     *
     * @attention Assumes that the input payload_buffer is organized in a way that index 0 is reserved for the overhead
     * byte, payload is found between indices 1 and 254 (maximum payload size is 254, but it can be less) and index
     * payload_size + 1 (maximum 255) is reserved for the delimiter byte. If this assumption is violated, the method
     * will not be able to decode the packet.
     *
     * @note To ensure assumptions are met, this method is not intended to be called directly. Instead, it should be
     * called by the SerializedTransferProtocol class.
     *
     * @tparam buffer_size The size of the input buffer, calculated automatically via template. This ensures that all
     * buffer-manipulating operations are performed within the buffer boundaries. This is crucial for avoiding
     * unexpected behavior and/or data corruption.
     * @param packet_buffer The buffer (array) that holds the COBS-encoded packet to be decoded. Assumes the buffer
     * stores the overhead byte at index 0 and the delimiter byte at index packet_size - 1. Has to be large enough to
     * accommodate the declared packet_size.
     * @param packet_size The size of the packet to decode in bytes. Note, unlike EncodePayload() method, in this
     * case the size should include the overhead byte and the delimiter byte. This is used as an extra security check to
     * make sure the method will not attempt to jump out of the payload bounds and as a fallback if CRC check fails to
     * detect any data corruption / errors introduced during the transmission. Has to be 3 bytes at a minimum and 256
     * bytes at a maximum. Should not include CRC bytes or any other postamble information (See SerializedTransferProtocol
     * class for more details).
     * @param delimiter_byte_value The value between 0 and 255 (any value that fits into uint8_t range) that was used
     * as packet delimiter. The methods assumes that all instances of this value inside the payload are replaced with
     * COBS-encoded values and the only instance of this value is found at the very end of the payload. The only
     * exception to this rule is the overhead byte, the decoder is designed to work around the overhead byte being set
     * to the delimiter_byte_value. This value is used both to restore the encoded variables during decoding forward
     * pass and as an extra corruption-check, as the algorithm expects to only find the instance of this value at the
     * end of the packet.
     *
     * @returns The size of the payload in bytes, which excludes the overhead byte value and the delimiter byte value.
     * Successful method runtimes return packet_size - 2, while failed runtimes return 0. Use cobs_status class variable
     * to obtain specific runtime error code if the method fails (can be interpreted by using kCOBSProcessorCodes
     * enumeration available through stp_shared_assets namespace).
     *
     * Example usage:
     * @code
     * COBSProcessor cobs_class;
     * uint8_t packet_buffer[7] = {4, 2, 3, 4, 0, 11, 11};
     * uint16_t packet_size = 5;
     * uint8_t delimiter_byte_value = 0;
     * uint16_t payload_size = cobs_class.DecodePayload(packet_buffer, packet_size, delimiter_byte_value);
     * @endcode
     */
    template <size_t buffer_size>
    uint16_t
    DecodePayload(uint8_t (&packet_buffer)[buffer_size], const uint16_t packet_size, const uint8_t delimiter_byte_value)
    {
        // Ensures input packets are at least a minimum valid size in length (at least 3: overhead byte, one data byte,
        // delimiter byte).
        if (packet_size < static_cast<uint16_t>(kCOBSProcessorParameters::kMinPacketSize))
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kDecoderTooSmallPacketSize);
            return 0;
        }

        // Ensures input packets do not exceed the maximum allowed size (up to 256: due to byte-encoding using COBS
        // scheme).
        else if (packet_size > kCOBSProcessorParameters::kMaxPacketSize)
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kDecoderTooLargePacketSize);
            return 0;
        }

        // Ensures that the buffer is large enough to store the declared packet size. This guards against accessing
        // memory outside the buffer boundaries during runtime.
        if (static_cast<uint16_t>(buffer_size) < packet_size)
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kDecoderPacketLargerThanBuffer);
            return 0;
        }

        // Verifies that the packet's overhead byte is not set to 0. This method resets the overhead byte of
        // decoded buffers to 0 to indicate that the packet has been decoded. Running decoding on the same data twice
        // will corrupt the data, which is avoided via this check.
        if (packet_buffer[0] == 0)
        {
            cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kPacketAlreadyDecoded);
            return 0;
        }

        // Starts reading from the overhead byte at index 0, uses uint16_t to deal with overflows
        uint16_t read_index = 0;

        // Tracks distance to the next delimiter_byte_value. Initializes to the value obtained from reading the overhead
        // byte, which points to the first (or only) occurrence of the delimiter_byte_value in the packet.
        auto next_index = static_cast<uint16_t>(packet_buffer[read_index]);

        // Resets the overhead byte to 0 to indicate that the buffer has been through a decoding cycle, even if the
        // cycle (see below) fails.
        packet_buffer[read_index] = 0;

        // Loops until a delimiter_byte_value is found or read_index exceeds packet limits. Checks for the NEXT
        // read index at the beginning of each loop (before it is advanced by aggregating the next_index into the
        // read_index). The loop has to avoid processing the overhead byte to support the rare case where a non-zero
        // delimiter is used and the overhead byte itself matches the delimiter (would trigger early breaking from
        // within the loop). The loop also has to evaluate whether the 'jumped to' index is still within the input
        // packet before the jump is actually made. To reconcile the two cases, the loop checks whether the NEXT
        // iteration of the read index is within the packet constraints before it updates the read_index inside the
        // loop. This is further supported by the fact both indices use uint16_t and, as such, should be immune to
        // overflow given the packet size checks and uint8_t buffer. In a 'worst case scenario' next_index extracted
        // from the buffer would be 255, added to read_index set to 255, which produces a value still well within the
        // uint16_t range, but above the maximum allowed packet_size of ~256.
        while ((read_index + next_index) < packet_size)
        {
            // Jumps to the next encoded delimiter byte's position by distance aggregation. If the overhead_byte was set
            // to 255, this operation would jump straight to the end of the packet.
            read_index += next_index;

            // Checks if the value obtained from indexing updated read_index matches the packet delimiter value
            if (packet_buffer[read_index] == delimiter_byte_value)
            {
                // If the read index matches the packet_size - 1 (is the last retrievable index, according to the packet
                // size) returns the size of the decoded payload, which is equal to read_index - 1. Due to the delimiter
                // and overflow, the real payload size is the size of the data between index 1 (inclusive) and the index
                // before the delimiter. At this point in runtime, read_index should be pointing at the delimiter byte,
                // so the index before that is read_index - 1. Since the payload is found between indices 1 and
                // read_index - 1, its length matches read_index - 1.
                if (read_index == packet_size - 1)
                {
                    // Sets the status to indicate that encoding was successful.
                    cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kPayloadDecoded);

                    // Returns the decoded payload index
                    return static_cast<uint16_t>(read_index - 1);
                }

                // If the delimiter byte was found earlier than expected, this indicates data corruption that was not
                // caught by the CRC check. IN this case, issues an appropriate error codes and breaks method runtime.
                else
                {
                    cobs_status =
                        static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kDecoderDelimiterFoundTooEarly);
                    return 0;
                }
            }

            // If the loop has not been broken, updates next_index with the next jump distance by reading the
            // value of the encoded variable
            next_index = packet_buffer[read_index];

            // Restores the original delimiter_byte_value (decodes the variable value)
            packet_buffer[read_index] = delimiter_byte_value;
        }

        // If a point is reached where the read_index does not point at the delimiter_byte_value variable, but exceeds
        // packet_size - 1 (at most 255), this means that the packet is in some way malformed. Well-formed packets
        // should always end in delimiter_byte_value reachable by traversing COBS-encoding variables.
        cobs_status = static_cast<uint8_t>(stp_shared_assets::kCOBSProcessorCodes::kDecoderUnableToFindDelimiter);
        return 0;
    }
};

#endif  //AMC_COBS_PROCESSOR_H
