/**
 * @file
 * @brief The header file for the Core Communication class, which is used to establish and maintain bidirectional
 * communication with the rest of the Ataraxis-compatible systems over the USB or UART interface.
 *
 * @section comm_description Description:
 *
 * @note A single instance of this class should be created inside the main.cpp file and provided as a reference input
 * to all other classes from this library that need to communicate with other systems.
 *
 * @subsection developer_notes Developer Notes:
 *
 * @attention This is a high-level API wrapper for the low-level AtaraxisTransportLayer class. It is used to define the
 * payload microstructure and abstracts away the low-level handling of serial packet formation, transmission, parsing
 * and validation.
 *
 * Given the importance of class error codes for diagnosing any potential problems, all data packing and unpacking
 * functions use fixed status byte-codes transmitted through the communication_status variable alongside returned
 * boolean values. Returned codes can be identified by passing them through the kSerialPCCommunicationCodes enumeration
 * that is available through class namespace.
 *
 * The PC error-messaging functionality is executed regardless of caller behavior as it is deemed critically important
 * to notify the main client of any errors encountered at this stage. Specifically, any failure of any methods,
 * available through this class, triggers full buffer and communication variables reset followed by constructing and
 * sending a standardized error message to the PC.
 *
 * @note The error-messaging method contains the fallbacks that avoids sending messages if this results in the
 * controller being stuck in an infinite loop. Therefore, the absence of errors does not automatically mean that the
 * library works as expected and extensive testing is heavily encouraged prior to using this library for production
 * runtimes.
 *
 * @subsubsection rx_tx_data_structures_overview Rx/Tx Data Structures Overview
 *
 * These datastructures are used to serialize and de-serialize the data for Serial transmission. Each incoming or
 * outgoing serial stream consists of a package macrostructure (handled by AtaraxisTransportLayer class), and
 * microstructure, defined using header structures available through this class. Each header structure contains the
 * information critical for properly addressing the packet.
 *
 * All data that follows the header is organized into sub-structures with two fields: ID and value. The ID fields are
 * used to determine the size, type and specific purpose of the bundled data-value. Specifically, it can identify unique
 * parameters, data-values, commands, etc. This system relies on sender and receiver sharing the same set of unique IDs
 * that can be used to individuate all data-values. The data IDs only need to be unique within the broader context of
 * the message header structure (i.e: given the specific combination of header structure values).
 *
 * @note The physical payload limit is 254 bytes, so there is a cap on how much data can be sent as a single packet.
 * This class contains validation mechanisms that raise errors if the buffer overflows, although this is considered an
 * unlikely scenario if the class is used as intended.
 *
 * @attention The PC and this class should be fully aligned on the datastructures that make up each stream (Rx or Tx),
 * their contents and positions inside the stream. Otherwise, correct serialization and de-serialization would not be
 * possible. Great care should be taken when writing command and data communication subroutines for new Modules and it
 * is highly advised to test all communication protocols prior to using them during live experiments.
 *
 * @subsubsection rx_tx_datastructures_visualization Byte-Stream Anatomy Visualized:
 *
 * This is a rough outline of how the Rx and Tx data streams look. Each |block| represents a datastructure either
 * defined inside this class or provided by AtaraxisTransportLayer class. Only the datastructures defined inside this
 * class are relevant for data serialization and de-serialization, AtaraxisTransportLayer handles processing the
 * starting and ending structures of each stream prior to de-serializing the custom datastructures of this project.
 *
 * - **Protocol**: |Start| |Packet Layout| |General ID| |Protocol Metadata| |PackedObject 1| ... |PackedObject n| |End|.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - ataraxis_transport_layer.h for low-level methods for sending and receiving messages over USB or UART Serial port.
 * - shared_assets.h for globally shared static message byte-codes.
 *
 * @see serial_pc_communication.cpp for method implementation.
 */

#ifndef AXTL_COMMUNICATION_H
#define AXTL_COMMUNICATION_H

// Dependencies:
#include <Arduino.h>
#include "shared_assets.h"
#include "transport_layer.h"

/**
 * @class Communication
 * @brief Exposes methods that allow interfacing with project Ataraxis infrastructure via an internal binding of the
 * AtaraxisTransportLayer library.
 *
 * This class handles all communications between the microcontroller and the rest of the Ataraxis infrastructure over
 * the USB (preferred) or UART serial interface. It relies on the AtaraxisTransportLayer library, which is
 * purpose-built to handle the necessary low-level manipulations. The class acts as a static linker between the
 * unique custom modules and the generalized Ataraxis infrastructure, which simplifies the integration of custom code
 * into the existing framework at-large.
 *
 * @note This interface is intended to be used both by custom module classes (classes derived from the BaseModule
 * class) and the Kernel class. Overall, this class defines and maintains a rigid payload microstructure that acts on
 * top of the serialized message structure enforced by the AtaraxisTransportLayer. Combined, these two classes
 * define a robust communication system specifically designed to support realtime communication between Ataraxis
 * infrastructure.
 *
 * @warning This class is explicitly designed to hide most of the configuration parameters used to control the
 * communication interface from the user, as Ataraxis architecture is already fine-tuned to be optimal for most
 * use-cases. Do not modify any of the internal parameters or subclasses of this class unless you know what you are
 * doing. Ataraxis as a whole is a complex project and even minor changes to any of its components may lead to undefined
 * behavior.
 *
 * Example instantiation:
 * @code
 * Serial.begin(9600);  // Initializes serial interface.
 * Communication(Serial) comm_class;  // Instantiates the Communication class.
 * @endcode
 */
template <uint16_t kSerialBufferSize>
class Communication
{
  public:

    /// Tracks the most recent class runtime status. This is mainly helpful for unit testing, as during production
    /// runtime most of the status codes are sent over to other Ataraxis systems for evaluation instead of being
    /// processed locally.
    uint8_t communication_status = static_cast<uint8_t>(shared_assets::kCoreStatusCodes::kCommunicationStandby);

    /**
     * @brief Instantiates a new Communication class object.
     *
     * Critically, the constructor class also initializes the AtaraxisTransportLayer as part of it's instantiation
     * cycle, which is essential for the correct functioning of the communication interface.
     *
     * @param communication_port A reference to the fully configured instance of a Stream interface class, such as
     * Serial or USBSerial. This class is used as a low-level access point that physically manages the hardware used to
     * transmit and receive the serialized data.
     *
     * Example instantiation:
     * @code
     * Serial.begin(9600);  // Initializes serial interface.
     * Communication(Serial) comm_class;  // Instantiates the Communication class.
     * @endcode
     */
    explicit Communication(Stream& communication_port) :
        _serial_protocol(
            communication_port,  // References the Stream class instance (Serial or USBSerial)
            0x1021,  // Polynomial to be used for CRC, should be the same bit-width as used by serial_protocol's template
            0xFFFF,  // The value that CRC checksum is initialized to
            0x0000,  // The final value the calculated CRC checksum is XORed with
            129,     // The value used to denote the start of a packet
            0,       // The value used as packet delimiter
            20000,   // The maximum number of microseconds allowed between receiving consecutive bytes of the packet.
            false    // A boolean flag that specifies whether to record start_byte detection errors
        )
    {}

    /**
     * @brief Resets the transmission_buffer and any AtaraxisTransportLayer and Communication class tracker
     * variables implicated in the data transmission process.
     *
     * @note Generally, this method is called by all other class methods where necessary and does not need to be called
     * manually while using the Communication class. The method is publicly exposed to support testing and for users
     * with specific, non-standard use-cases that may require this functionality.
     *
     * @attention This method does not reset the _outgoing_header variable of the Communication class.
     */
    void ResetTransmissionState()
    {
        _serial_protocol.ResetTransmissionBuffer(
        );                               // Resets the necessary AXTL class variables (including the buffer).
        _transmission_buffer_index = 0;  // Resets local transmission_buffer index tracker.

        // Critically, this method does NOT reset the _outgoing_header class variable. This is done on purpose to optimize
        // certain message header manipulations.
    }

    /**
     * @brief Resets the reception_buffer and any AtaraxisTransportLayer and Communication class tracker
     * variables implicated in the data reception process.
     *
     * @note Generally, this method is called by all other class methods where necessary and does not need to be called
     * manually while using the Communication class. The method is publicly exposed to support testing and for users
     * with specific, non-standard use-cases that may require this functionality.
     *
     * @attention This method does not reset the _incoming_header variable of the Communication class.
     */
    void ResetReceptionState()
    {
        // Resets the protocol buffer and trackers
        _serial_protocol.ResetReceptionBuffer();
        _reception_buffer_index = 0;
    }

    /**
     *
     * @tparam ObjectType The type object to be packed into the payload region of the transmission buffer. This is
     * inferred automatically by the template constructor.
     * @param protocol The code of the protocol to be used to transmit the packed data.
     * @param object_id The unique ID-code for the transmitted object. Has to be unique with respect to the values of
     * the id and metadata structures.
     * @param object The object to be packed into the buffer. The method uses referencing to copy the object data into
     * the buffer.
     *
     * @return Boolean true if the operation succeeds and false otherwise.
     */
    template <typename ObjectType>
    bool PackDataObject(uint8_t protocol, uint8_t object_id, ObjectType& object)
    {
        uint16_t header_size;
        if (protocol == static_cast<uint8_t>(axmc_shared_assets::kCommunicationProtocols::kPackedObjectCommand))
        {
            header_size = kCommandHeaderSize;
        }
        else if (protocol == static_cast<uint8_t>(axmc_shared_assets::kCommunicationProtocols::kPackedObjectData))
        {
            header_size = kDataHeaderSize;
        }

        // Specifies the appropriate PackedValue structure (using the input value datatype) and instantiates it using
        // the input data.
        PackedObject<value_type> value_structure(id_code, value);

        // Shifts the write index to make space for the header structures (ID and metadata). This way, the message can
        // be constructed without providing the header or protocol metadata information.
        const uint16_t write_index = _transmission_buffer_index + header_size;

        // Writes the structure instantiated above to the transmission buffer of the AtaraxisTransportLayer class.
        uint16_t next_index = _serial_protocol.WriteData(value_structure, write_index);

        // Handles cases when writing to buffer fails, which is indicated by the returned next_index being 0.
        if (next_index == 0)
        {
        }

        // Note, if writing was successful, the status has already been set via the main data-size-based conditional
        // block
        return true;
    }

    // bool ReceiveMessage();

    // bool SendData();

    // bool SendCommand();

  private:
    /// Stores the minimum valid size of incoming payloads. Currently, the smallest message structure is the
    /// ParameterMessage structure. This structure contains 3 'header' bytes, followed by, as a minimum, 1-parameter
    /// byte. Also, this statically includes the 'protocol' header byte.
    static constexpr uint16_t kMinimumPayloadSize = sizeof(communication_assets::ParameterMessage) + 2;

    /// Stores the size of the CRC Checksum postamble in bytes. This is directly dependent on the variable type used
    /// for the PolynomialType template parameter when specializing the TransportLayer class (see below).
    /// At the time of writing, valid values are 1 (for uint8_t), 2 (for uint16_t) and 4 (for uint32_t).
    static constexpr uint8_t kCRCSize = 2;

    /// Stores the size of the preamble and the COBS metadata variables in bytes. There is no reliable heuristic for
    /// calculating this beyond knowing what the particular version of TransportLayer uses internally, but
    /// COBS statically reserves 2 bytes (overhead and delimiter) and at the time of writing the preamble consists of
    /// start_byte and payload_size (another 2 bytes).
    static constexpr uint8_t kMetadataSize = 4;

    /// Combines the CRC and Metadata sizes to produce the final size of the payload region that would be reserved for
    /// the TransportLayer variables. This is necessary to calculate the available reception buffer space, accounting
    /// for the fact the TransportLayer class needs space for its variables in addition to the payload. Note; this
    /// does not include the protocol byte, which is considered part of the payload managed by the Communication class.
    static constexpr uint8_t kReservedNonPayloadSize = kCRCSize + kMetadataSize;

    /**
     * @brief The maximum size of the payloads expected to be sent and received by this microcontroller.
     *
     * Calculates the maximum feasible payload region size to use for the communication messages. Mostly, this is
     * important for received payloads, as transmitted payloads do not have to fully fit into the buffer (it helps
     * though). Many microcontroller architectures limit the size of the circular buffers used by the Serial class to
     * receive data. Therefore, if the incoming message exceeds circular buffer size, it may be clipped unexpectedly
     * when using a very fast baudrate (the main runtime is not able to 'retrieve' the received bytes to free the
     * buffer in time). To avoid this issue, this variable calculates and stores the highest feasible buffer size given
     * either the static 254 limitation (due to COBS) or the difference of system-reserved space and the buffer size
     * supported by the architecture. Note, the latter choice relies on statically defining buffer sizes for different
     * board architectures (see preprocessor directives at the top of this file) and may not work for your specific
     * board if it is not one of the explicitly defined architectures.
     */
    static constexpr uint16_t kMaximumPayloadSize =  // NOLINT(*-dynamic-static-initializers)
        min(kSerialBufferSize - kReservedNonPayloadSize, 254);

    /**
     *  The specialization of the AtaraxisTransportLayer class that handles all low-level operations required to
     *  send and receive data. Note, the AXTL class is specialized statically and initialized as part of the
     *  Communication class initialization. As such, to alter the configuration of the class, you may need to alter
     *  specialization, construction or both initialization steps.
     *
     *  @warning Do not alter AXTL specialization unless you know what you are doing. This is especially important for
     *  the maximum and minimum payload size parameters, which are meant to be calculated using preprocessor directives
     *  found at the top of the Communication.h file. These parameters are very important for the correct functioning of
     *  the Communication class and ideally should not be tempered with.
     *
     *  @attention If you chose to modify any of the specialization parameters, make sure the Communication class
     *  Constructor and _serial_protocol variable declaration are internally consistent. This
     *  will likely require at the very least reading the main AtaraxisTransportLayer class documentation alongside
     *  the Communication class documentation and, preferably, thorough isolated testing.
     */
    TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize, kMinimumPayloadSize> _serial_protocol;

    /// Stores the index of the nearest unused byte in the payload region of the transmission_buffer, relative to the
    /// beginning of the buffer. This is used internally to sequentially and iteratively write to the payload region of
    /// the transmission_buffer. This tracker is critically important for working with the transmission_buffer and,
    /// therefore, is not directly exposed to the user.
    uint16_t _transmission_buffer_index = 0;

    /// Stores the index to the nearest unread byte in the payload region of the reception_buffer, relative to the
    /// beginning of the buffer. This tracker is similar to the transmission_buffer_index, but used to read data from
    /// the reception buffer. Given its' critical importance for processing received data, it is not exposed to the
    /// user.
    uint16_t _reception_buffer_index = 0;

    /// Stores the currently active protocol code. This is used in multiple internal checks to allow the end-users to
    /// work with PackedObjects without processing the header and metadata structures. Specifically, the user is
    /// required to provide a protocol code when reading PackedObjects, which is compared to the value of this variable.
    /// If the values mismatch, the AXMC raises an error, as this is a violation of the safety standards. For example,
    /// this mechanism is used when the user is filling the payload for a data message to ensure the message is sent
    /// before allowing writing a new data message.
    uint8_t _protocol_code = static_cast<uint8_t>(communication_assets::kProtocols::kUndefined);
};

#endif  //AXTL_COMMUNICATION_H
