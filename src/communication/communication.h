/**
 * @file
 * @brief The header file for the Core Communication class, which is used to establish and maintain bidirectional
 * communication with the rest of the Ataraxis-compatible systems over the USB or UART interface.
 *
 * @section comms_description Description:
 *
 * @note A single instance of this class should be created inside the main.cpp file and provided as a reference input
 * to all other classes from this library that need to communicate with the PC.
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
#include "../communication_assets.h"
#include "shared_assets.h"
#include "transport_layer.h"

// Statically defines the size of the Serial class reception buffer associated with different Arduino and Teensy board
// architectures. This is required to ensure the Communication class is configured appropriately. If you need to adjust
// the STP buffers (for example, because you manually increased the buffer size used by the Serial class of your board),
// do it by editing or specifying a new preprocessor directive below. It is HIGHLY advised not to tamper with these
// settings, however, and to always have the kSerialBufferSize set exactly to the size of the Serial class reception
// buffer.
#if defined(ARDUINO_ARCH_SAM)
// Arduino Due (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

#elif defined(ARDUINO_ARCH_SAMD)
// Arduino Zero, MKR series (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

#elif defined(ARDUINO_ARCH_NRF52)
// Arduino Nano 33 BLE (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

// Note, teensies are identified based on the processor model. This WILL need to be updated for future versions of
// Teensy boards.
#elif defined(CORE_TEENSY)
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
    defined(__IMXRT1062__)
// Teensy 3.x, 4.x (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 1024;
#else
// Teensy 2.0, Teensy++ 2.0 (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;
#endif

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__)
// Arduino Uno, Mega, and other AVR-based boards (UART serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 64;

#else
// Default fallback for unsupported boards is the reasonable minimum buffer size
static constexpr uint16_t kSerialBufferSize = 64;

#endif

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
class Communication
{
  public:

    /// Tracks the most recent class runtime status. This is mainly helpful for unit testing, as during production
    /// runtime most of the status codes are sent over to other Ataraxis systems for evaluation instead of being
    /// processed locally.
    uint8_t communication_status = static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kStandby);

    /**
     * @struct ID
     * @brief The header structure used by all incoming or outgoing message payloads, that stores the information
     * necessary to properly process the payload.
     *
     * @attention The layout of the ID structure is the same for all payloads. This structure forms the backbone of the
     * highly flexible communication system of the project Ataraxis and it primarily determines the addressee of each
     * transmitted message.
     *
     * The ID structure is expected to occupy the first 4 bytes of every incoming and outgoing message payload.
     */
    struct ID
    {
        /// The ID of the communication protocol. This ID specifies which of the multiple supported message protocols
        /// (layouts) were used to compose the message. This is the most significant variable of the entire message as
        /// it controls how all following values are interpreted.
        uint8_t protocol = 0;

        /// The type-code of the module to which the message is addressed OR which sent the message. Module types are
        /// families of modules, e.g. water valve, lick sensor, IR camera, etc. The specific use of this variable is
        /// determined by the protocol. Commonly, command protocols use this field to address the command and data
        /// protocols use this field to specify the source. Note, module-type IDs should be unique across all used
        /// systems.
        uint8_t module_type = 0;

        /// The specific module ID within the broader module family specified by module_type. Specifically, if module
        /// type addresses the payload to a water valve, this field would supply the ID of the specific water-valve to
        /// use. This is needed to target specific modules (physical) when multiple modules of the same type are used.
        uint8_t module_id = 0;

        /// The use of this field enables 'delivery-report' functionality. Specifically, when this is set to
        /// a non-zero value, the Communication class will send this code back to the sender upon successfully
        /// processing the received message to notify the sender that the message was received intact. This allows
        /// optionally ensuring message delivery. Setting this field to 255 disables delivery assurance.
        uint8_t return_code = 0;
    } __attribute__((packed));

    /**
     * @struct PackedObject
     * @brief The structure used by the PackedObject protocol family to bundle each transmitted object (value) with a
     * unique ID code.
     *
     * @tparam ObjectType The type of the transmitted object. This allows using the same structure definition to
     * flexibly support many object types, improving code maintainability. This works for any valid object type,
     * including structures and arrays.
     *
     * This structure serves as a standardized building-block from which the majority of the PackedObject protocol
     * family payloads are assembled. Specifically, it bundles each transmitted object (value, array, structure, etc.)
     * with a unique ID code (at least when combined with the general header ID structure and protocol metadata
     * structure).  Overall, this offers a good mix of robustness and flexibility that is generally sufficient for
     * the vast majority of use-cases.
     *
     * @attention A major downside of using these structures as building blocks is that the receiver and the sender
     * need to use the same procedures for packing and unpacking each unique ID, including inferring the object size
     * and type from the unique ID.
     *
     * @warning This structure is used both by Data and Command protocol variants. While the protocols themselves differ
     * in terms of the protocol metadata structure that comes after the general header, they are otherwise assembled
     * from PackedObject structures.
     */
    template <typename ObjectType>
    struct PackedObject
    {
        /// The id for the input object. Object-bound IDs are used to uniquely identify the transmitted objects given a
        /// particular combination of the general header ID structure and the protocol metadata structure. The ID
        /// is used to infer the size and type of the object by the receiver.
        uint8_t id = 0;

        /// The transmitted object. This can be any valid object type, as long as it fits into the constrains imposed
        /// by the maximum message payload size.
        ObjectType object = 0;
    } __attribute__((packed));

    /**
     * @struct CommandMetadata
     * @brief The metadata structure used by CommandPackedObject protocol to store the command-specific metadata.
     *
     * This data block comes after the general header ID structure and it contains information that has to be present
     * for every command to work as intended. For example, this includes the unique command code to execute (and there
     * can only be one command code per each Command message).
     *
     * @attention This structure has to be included between General ID and any PackedObject structures for each Command
     * message.
     */
    struct CommandMetadata
    {
        /// The unique ID of the command to execute. This works much like PackedObject IDs do, but allows to separate
        /// commands from general packed objects, which typically communicate command-specific parameters.
        uint8_t command = 0;

        /// Determines whether the command needs to be executed once or repeatedly cycles with a certain periodicity.
        /// Together with the cycle_duration (see below), this allows communicating both one-shot and cyclic command
        /// runtimes.
        bool cycle = 0;

        /// The period of time, in milliseconds, to delay before repeating (cycling) the command. This is only used if
        /// cycle field is set to True.
        uint32_t cycle_duration = 0;
    } __attribute__((packed));

    /**
     * @struct DataMetadata
     * @brief The metadata structure used by DataPackedObject protocol to store the data-specific metadata.
     *
     * Similar to CommandMetadata, this structure stores specific parameters necessary for data messages to work as
     * intended.
     *
     * @attention This structure has to be included between General ID and any PackedObject structures for each Data
     * message.
     */
    struct DataMetadata
    {
        /// The unique ID of the command that was being executed when the data message was sent. Since the AXMC has to be
        /// executing a command to produce data, this explicitly links any data message to the ongoing command.
        uint8_t command = 0;

        /// The major unique data-type code. Broadly speaking, data messages can be divided into multiple types,
        /// such as sensor data, error messages, warnings, etc. This code is used to communicate the kind of the data
        /// message. In turn, this may inform how the message is processed by the receiver.
        uint8_t type = 0;

        /// Stores the code that describes the event. This can be a specific error code or data-event code. This
        /// is used similar to the 'command' code and allows identifying the main event that triggered the message.
        /// The message can include additional PAckedObjects to communicate event data-points or include additional
        /// information codes.
        uint8_t event = 0;
    } __attribute__((packed));

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
    explicit Communication(Stream& communication_port);

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
    void ResetTransmissionState();

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
    void ResetReceptionState();

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
    bool PackDataObject(uint8_t protocol, uint8_t object_id, ObjectType &object)
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
    /// Stores the minimum valid size of incoming payloads. Currently, this necessarily includes the general ID,
    /// 4 bytes (the size of the smallest Protocol Metadata section) and 2 bytes for the smallest supported
    /// PackedObject.
    static constexpr uint16_t kMinimumPayloadSize = sizeof(ID) + 6;

    /// Stores the size of the CRC Checksum postamble in bytes. This is directly dependent on the variable type used
    /// for the PolynomialType template parameter when specializing the AtaraxisTransportLayer class (see below).
    /// At the time of writing, valid values are 1 (for uint8_t), 2 (for uint16_t) and 4 (for uint32_t).
    static constexpr uint8_t kCRCSize = 2;

    /// Stores the size of the preamble and the COBS metadata variables in bytes. There is no reliable heuristic for
    /// calculating this beyond knowing what the particular version of AtaraxisTransportLayer uses internally, but
    /// COBS statically reserves 2 bytes (overhead and delimiter) and at the time of writing the preamble consisted of
    /// start_byte and payload_size (another 2 bytes).
    static constexpr uint8_t kMetadataSize = 4;

    /// Combines the CRC and Metadata sizes to produce the final size of the payload region that would be reserved for
    /// the protocol variables. This is needed to appropriately discount the available reception buffer space to account
    /// for the fact the AXTL class needs space for its variables in addition to the payload. Note, this does NOT
    /// include the general ID structure or any protocol Metadata structure, as they are considered parts of the actual
    /// payload that are handled separately by the Communication class itself.
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
    static constexpr uint16_t kMaximumPayloadSize = min(kSerialBufferSize - kReservedNonPayloadSize, 254);

    /// Stores the first index inside the buffer payload region that can be used to store PackedObjects. This allows
    /// reserving buffer space to account for the header ID and command Metadata structures. In turn, this information
    /// allows working with PackedObjects independently of the header and metadata information.
    static constexpr uint16_t kCommandHeaderSize = sizeof(CommandMetadata) + sizeof(ID);

    /// Sames as kCommandHeaderSize, but for data messages.
    static constexpr uint16_t kDataHeaderSize = sizeof(DataMetadata) + sizeof(ID);

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
    AtaraxisTransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize, kMinimumPayloadSize> _serial_protocol;

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

    /// Stores the ID structure used for the outgoing messages. While the structure is typically also stored in the
    /// transmission_buffer, having an independent storage variable helps when the buffer needs to be recreated and for
    /// certain other scenarios.
    ID _outgoing_header;

    /// Stores the ID structure extracted from incoming messages. While this structure is typically also stored in the
    /// reception_buffer, having an independent storage variable for the structure is beneficial for optimizing ID data
    /// readout procedures.
    ID _incoming_header;

    /// Stores the currently active protocol code. This is used in multiple internal checks to allow the end-users to
    /// work with PackedObjects without processing the header and metadata structures. Specifically, the user is
    /// required to provide a protocol code when reading PackedObjects, which is compared to the value of this variable.
    /// If the values mismatch, the AXMC raises an error, as this is a violation of the safety standards. For example,
    /// this mechanism is used when the user is filling the payload for a data message to ensure the message is sent
    /// before allowing writing a new data message.
    uint8_t _protocol_code = axmc_shared_assets::kCommunicationProtocols::kUndefined;
};

#endif  //AXTL_COMMUNICATION_H
