/**
 * @file
 * @brief The header file for the Communication class, which is used to establish and maintain bidirectional
 * communication with other Ataraxis systems over the USB or UART interface.
 *
 * @section comm_description Description:
 *
 * @note A single instance of this class should be created inside the main.cpp file and provided as a reference input
 * to Kernel and (base) Module-derived classes.
 *
 * Communication class functions as an opinionated extension of the TransportLayer class, enforcing consistent
 * message structures and runtime error handling behaviors.
 *
 * @subsection developer_notes Developer Notes:
 *
 * This is a high-level API wrapper for the low-level TransportLayer class. It is used to define the
 * communicated payload microstructure and abstracts away all communication steps.
 *
 * @attention For this class to work as intended, its configuration, message layout structures and status codes should
 * be the same as those used by the Ataraxis system this class communicates with. Make sure that both communicating
 * parties are configured appropriately!
 *
 * @subsubsection rx_tx_data_structures_overview Rx/Tx Data Structures Overview
 *
 * These datastructures are used to serialize and deserialize the data for Serial transmission. Each incoming or
 * outgoing serial stream consists of a package macrostructure (handled by TransportLayer class), and microstructure,
 * defined via Communication class message layouts (Protocols). Each supported message layout, available through the
 * communication_assets namespace, is uniquely configured to optimally support the intended communication format.
 *
 * @attention It is expected that every incoming or outgoing payload uses the first non-metadata byte to store the
 * protocol code. This code is used to interpret the rest of the message. Further, it is expected that each transmitted
 * packet contains a single message.
 *
 * @note The physical payload limit is 254 bytes, so there is a cap on how much data can be sent as a single packet.
 * This class contains validation mechanisms that raise errors if the buffer overflows, although this is considered an
 * unlikely scenario if the class is used as intended.
 *
 * @subsubsection rx_tx_datastructures_visualization Byte-Stream Anatomy Visualized:
 *
 * This is a rough outline of how the Rx and Tx data streams look. Each |block| represents a structure either provided
 * by this class or TransportLayer class. Only the datastructures defined inside this class are relevant for data
 * serialization and deserialization, TransportLayer handles processing the preamble and postamble structures of
 * each stream prior to deserializing the custom datastructures of this project. Note, only the Protocol Code and
 * Message Payload count towards the 254-byte limit for message payload.
 *
 * - **Protocol**: |Start Byte| |Header Preamble| |Protocol Code| |Message Payload| |Delimiter Byte| |CRC Postamble|.
 *
 * @subsection dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - transport_layer.h for low-level methods for sending and receiving messages over USB or UART Serial port.
 * - shared_assets.h for globally shared assets, including communication_assets namespace.
 *
 * @see serial_pc_communication.cpp for method implementation.
 */

#ifndef AXMC_COMMUNICATION_H
#define AXMC_COMMUNICATION_H

// Dependencies:
#include <Arduino.h>
#include "shared_assets.h"
#include "transport_layer.h"

/**
 * @class Communication
 * @brief Exposes methods that allow establishing and maintaining bidirectional serialized communication with other
 * Ataraxis systems via the internal binding of the TransportLayer library.
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
 * constexpr uint16_t kSerialBufferSize = 256;  // Controller port buffer size.
 * Communication<kSerialBufferSize> comm_class(Serial);  // Instantiates the Communication class.
 * @endcode
 */
template <uint16_t kSerialBufferSize>
class Communication
{
    public:

        /**
         * @brief This enumeration stores command byte-codes for all class methods designed to be called externally.
         *
         *
         */
        enum class kCommunicationCommands : uint8_t
        {

        };

        /// Tracks the most recent class runtime status.
        uint8_t communication_status = static_cast<uint8_t>(shared_assets::kCoreStatusCodes::kCommunicationStandby);

        /**
         * @brief Instantiates a new Communication class object.
         *
         * Critically, the constructor also initializes the TransportLayer class as part of its runtime, which is
         * essential for the correct functioning of the communication interface.
         *
         * @notes Make sure that the referenced Stream interface is initialized via its begin() method before calling
         * Communication class methods.
         *
         * @param communication_port A reference to a Stream interface class, such as Serial or USBSerial. This class
         * is used as a low-level access point that physically manages the hardware used to transmit and receive the
         * serialized data.
         *
         * Example instantiation:
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         * @endcode
         */
        explicit Communication(Stream& communication_port) :
            _transport_layer(
                communication_port,  // Stream
                0x1021,              // 16-bit CRC Polynomial
                0xFFFF,              // Initial CRC value
                0x0000,              // Final CRC XOR value
                129,                 // Start byte value
                0,                   // Delimiter byte value
                20000,               // Packet reception timeout (in microseconds)
                false                // Disables start byte detection errors
            )
        {}

        /**
         * @brief Resets the transmission_buffer and any TransportLayer and Communication class tracker variables
         * implicated in the data transmission process.
         *
         * @note This method is called by all other class methods where necessary and does not need to be called
         * manually. It is kept public to support testing.
         */
        void ResetTransmissionState()
        {
            _transport_layer.ResetTransmissionBuffer();
        }

        /**
         * @brief Resets the reception_buffer and any AtaraxisTransportLayer and Communication class tracker
         * variables implicated in the data reception process.
         *
         * @note This method is called by all other class methods where necessary and does not need to be called
         * manually. It is kept public to support testing.
         */
        void ResetReceptionState()
        {
            _transport_layer.ResetReceptionBuffer();
        }

        /**
         * @brief Packages the input data into a DataMessage structure and sends it over to the connected Ataraxis
         * system.
         *
         * This method is used both for sending data and error messages. The message will be interpreted according to
         * the bundled event_code. This codebase views errors as instances of generic events, same as data or 'success'
         * messages.
         *
         * Data messages consist of a rigid header structure that addresses the data payload and a flexible
         * data object, that can be used to pack additional information to be sent along with the message. Messages
         * without data can overwrite object_size argument with 0 and include a placeholder 'data' object, such as a
         * zero-value uint8_t variable.
         *
         * @tparam ObjectType The type of the data object to be sent along with the message. This is inferred
         * automatically by the template constructor.
         * @param event_code The byte-code specifying the error event.
         * @param module_type The byte-code specifying the type of the module that encountered the error.
         * @param module_id The ID byte-code of the specific module within the broader module_type family that
         * encountered the error.
         * @param command The byte-code specifying the command that triggered the error.
         * @param object Additional error data object to be sent along with the message. Currently, every error message
         * has to contain a data object, but you can use a sensible placeholder for calls that do not have a valid
         * objet to include.
         * @param object_size The size of the transmitted object, in bytes. This is calculated automatically based on
         * the object type. Do not overwrite this argument.
         *
         * @code
         * Communication comm_class(Serial);  // Instantiates the Communication class.
         * Serial.begin(9600);  // Initializes serial interface.
         *
         * // Sends the error message:
         * const uint8_t module_type = 112        // Example module type
         * const uint8_t module_id = 12;          // Example module ID
         * const uint8_t command = 88;            // Example command code
         * const uint8_t error_code = 221;        // Example error code
         * const uint8_t placeholder_object = 0;  // Meaningless, placeholder object
         * comm_class.SendErrorMessage(module_type, module_id, command, error_code, placeholder_object);
         * @endcode
         */
        template <typename ObjectType>
        void SendDataMessage(
            const uint8_t module_type,
            const uint8_t module_id,
            const uint8_t command,
            const uint8_t event_code,
            const ObjectType& object,
            const size_t object_size = sizeof(ObjectType)
        )
        {
            // Clears out the transmission buffer to prepare it for sending the message
            _transport_layer.ResetTransmissionBuffer();

            // Packages data into the message structure
            communication_assets::DataMessage<ObjectType> message;
            message.module_type = module_type;
            message.module_id   = module_id;
            message.command     = command;
            message.event       = event_code;
            message.object_size = static_cast<uint8_t>(object_size);
            message.object      = object;

            // Constructs the payload by writing the protocol code, followed by the message structure generated above
            uint16_t next index = _transport_layer.WriteData(
                static_cast<uint8_t>(communication_assets::kProtocols::kData),
                kProtocolIndex
            );
            _transport_layer.WriteData(message, kMessageIndex);

            // Sends data to the other Ataraxis system
            _transport_layer.SendData();
        }

        bool ReceiveMessage()
        {
            // Attempts to receive the next available message
            if (_transport_layer.ReceiveData())
            {
                // Parses the protocol code of the received message and returns true
                if (const uint16_t next_index = _transport_layer.ReadData(_protocol_code, kProtocolIndex);
                    next_index != 0)
                {
                    // Returns True to indicate that the message was received and its protocol code was extracted
                    return true;
                }
                else
                {
                    // If protocol extraction fails, uses the TransportLayer status code to send an error message
                    communication_status = _transport_layer.transfer_status;
                }
            }

            // The reception protocol can 'fail' gracefully if the reception buffer does not have enough bytes to
            // attempt message reception. In this case, the TransportLayer status is set to the
            // kNoBytesToParseFromBuffer constant. If reception failed with any other code, sets class status to an
            // error status. In turn, this triggers error-handling in the Kernel class (the only class that should be
            // calling this method).
            if (_transport_layer.transfer_status !=
                shared_assets::kTransportLayerStatusCodes::kNoBytesToParseFromBuffer)
            {
                communication_status =
                    static_cast<uint8_t>(shared_assets::kCoreStatusCodes::kCommunicationReceptionError);
            }

            return false;
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
            uint16_t next_index = _transport_layer.WriteData(value_structure, write_index);

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
        /// Stores the minimum valid incoming payload size. Currently, this is the size of the ParameterMessage header
        /// (4 bytes) + 1 Protocol byte + minimal Parameter object size (1 byte). This value is used to optimize
        /// incoming message reception behavior.
        static constexpr uint16_t kMinimumPayloadSize = 6;

        /// Stores the size of the CRC Checksum postamble in bytes. This is directly dependent on the variable type used
        /// for the PolynomialType template parameter when specializing the TransportLayer class. At the time of
        /// writing, valid values are 1 (for uint8_t), 2 (for uint16_t) and 4 (for uint32_t). The local TransportLayer
        /// binding uses CRC-16 by default (byte size: 2).
        static constexpr uint8_t kCRCSize = 2;

        /// Stores the size of the metadata variables used by the TransportLayer class. At the time of writing, this
        /// includes: the start byte, payload size byte, COBS overhead byte and COBS delimiter byte. A total of 4 bytes.
        static constexpr uint8_t kMetadataSize = 4;

        /// Combines the CRC and Metadata sizes to calculate the transmission buffer space reserved for TransportLayer
        /// variables. This value is used to calculate the (remaining) payload buffer space.
        static constexpr uint8_t kReservedNonPayloadSize = kCRCSize + kMetadataSize;

        /// Calculates maximum transmitted and received payload size. At a maximum, this can be 254 bytes
        /// (COBS limitation). For most controllers, this will be a lower value that depends on the available buffer
        /// space and the space reserved for TransportLayer variables.
        static constexpr uint16_t kMaximumPayloadSize =  // NOLINT(*-dynamic-static-initializers)
            min(kSerialBufferSize - kReservedNonPayloadSize, 254);

        /// Stores the index of the protocol code in the incoming message payload sequence. The protocol code should
        /// always occupy the first byte of each received and transmitted payload.
        static constexpr uint16_t kProtocolIndex = 0;

        /// Stores the first index of the message data region in the incoming payload sequence. This applies to any
        /// supported incoming or outgoing message structure.
        static constexpr uint16_t kMessageIndex = 1;

        /// Stores the first index of the Parameter message object data in the incoming message payload sequence.
        /// Parameter messages are different from other messages, as the parameter object itself is not part of the
        /// message structure. However, the message header has a fixed size, so the first index of the parameter data
        /// is static: it is the size of the ParameterMessage structure + Protocol byte (1).
        static constexpr uint16_t kParameterObjectIndex = sizeof(communication_assets::ParameterMessage) + 1;

        /// The bound TransportLayer instance that abstracts low-level data communication steps. This class statically
        /// specializes and initializes the TransportLayer to use sensible defaults. It is highly advised not to alter
        /// default initialization parameters unless you know what you are doing.
        TransportLayer<uint16_t, kMaximumPayloadSize, kMaximumPayloadSize, kMinimumPayloadSize> _transport_layer;

        /// Stores the protocol code of the last received message.
        uint8_t _protocol_code = static_cast<uint8_t>(communication_assets::kProtocols::kUndefined);

        communication_assets::CommandMessage _command_message;

        communication_assets::ParameterMessage<kMaximumPayloadSize-kMinimumPayloadSize+1> _parameter_message;
};

#endif  //AXMC_COMMUNICATION_H
