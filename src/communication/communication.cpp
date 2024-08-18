/**
 * @file
 * @brief The implementation file for the Communication class.
 *
 * @section comm_cpp_developer_notes Developer Notes:
 *
 * @attention Many class methods attempt to notify the connected system if they run into an error. While this may not
 * be successful, all methods are written in a way that prevents runtime deadlocking. It is expected that the Kernel
 * class, which functions as the main runtime controller, handles the necessary delays and additional procedures
 * associated with Communication class methods encountering runtime errors.
 *
 * @note This implementation file only contains non-template methods. All class template methods are implemented inside
 * the header file as customary for other AMC classes.
 *
 * @section comm_cpp_dependencies
 * - communication.h for class declarations.
 * - digitalWriteFast.h for fast pin manipulation functions.
 * - elapsedMillis.h for the timeout timers.
 *
 * @see Communication.h for method documentation.
 */

// Header file
#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "communication.h"

// Constructor implementation. Accepts a reference to Stream class instance and initializes the pre-specialized instance
// of the SerializedTransferProtocol class (this is the necessary pre-requisite to enable communication).
Communication::Communication(Stream& communication_port) :
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

// Resets all variables, buffers, and classes to support constructing and transmitting (sending) a new message to the
// connected Ataraxis system.
void Communication::ResetTransmissionState()
{
    _serial_protocol.ResetTransmissionBuffer();  // Resets the necessary AXTL class variables (including the buffer).
    _transmission_buffer_index = 0;              // Resets local transmission_buffer index tracker.

    // Critically, this method does NOT reset the _outgoing_header class variable. This is done on purpose to optimize
    // certain message header manipulations.
}

/// Resets all variables, buffers, and classes that store received data. This is often done to reset after data
/// reception failure
void Communication::ResetReceptionState()
{
    // Resets the protocol buffer and trackers
    _serial_protocol.ResetReceptionBuffer();
    _reception_buffer_index = 0;
}
