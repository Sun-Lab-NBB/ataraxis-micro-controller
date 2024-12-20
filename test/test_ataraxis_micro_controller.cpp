// Due to certain issues with reconnecting to Teensy boards when running separate test suits, this test suit acts as a
// single centralized hub for running all available tests for all supported classes and methods of the
// ataraxis-micro-controller library. Declare all required tests using separate functions (as needed) and then add the
// tests to be evaluated to the RunUnityTests function at the bottom of this file. Comment unused tests out if needed.

// This test suit only covers the Communication class. Kernel and Modules are tested with a fully functional
// communication interface using Python bindings. The idea behind this design pattern is that this test suit, together
// with TransportLayer tests, ensures that the microcontroller can talk to the PC. The PC then takes over the testing.

// Dependencies
#include <Arduino.h>  // For Arduino functions
#include <unity.h>    // This is the C testing framework, no connection to the Unity game engine
#include "axmc_shared_assets.h"
#include "cobs_processor.h"  // COBSProcessor class
#include "communication.h"   // Communication class
#include "crc_processor.h"   // CRCProcessor class
#include "stream_mock.h"     // StreamMock class required for Communication class testing

// This function is called automatically before each test function. Currently not used.
void setUp()
{}

// This function is called automatically after each test function. Currently not used.
void tearDown()
{}

// Tests the functioning of the Communication class SendDataMessage() method. This test checks both the Module-targeted
// version and the overloaded Kernel-targeted version. Since under the current configuration failing to send a Data
// message is essentially impossible without deliberately modifying the source code, this possibility is not
// evaluated here.
void TestSendDataMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    // Defines static message payload components.
    constexpr uint8_t module_type = 112;  // Example module type
    constexpr uint8_t module_id   = 12;   // Example module ID
    constexpr uint8_t command     = 88;   // Example command code
    constexpr uint8_t event_code  = 221;  // Example event code
    constexpr uint8_t test_object = 255;  // Test object

    // Defines the prototype and prototype byte-code. This tests uses kOneUint8, but this principle is the same for all
    // valid prototype codes
    constexpr auto prototype      = axmc_communication_assets::kPrototypes::kOneUint8;
    constexpr auto prototype_code = static_cast<uint8_t>(axmc_communication_assets::kPrototypes::kOneUint8);

    // Kernel test
    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    comm_class.SendDataMessage(command, event_code, prototype, test_object);
    constexpr uint16_t expected_kernel[5] = {kernel_protocol, command, event_code, prototype_code, test_object};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 5; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // Module test
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData);
    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    constexpr uint16_t expected_module[7] =
        {module_protocol, module_type, module_id, command, event_code, prototype_code, test_object};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 7; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the functioning of the Communication class SendStateMessage() method. This test checks both the Module-targeted
// version and the overloaded Kernel-targeted version. Since under the current configuration failing to send a State
// message is essentially impossible without deliberately modifying the source code, this possibility is not
// evaluated here.
void TestSendStateMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    // Defines static message payload components.
    constexpr uint8_t module_type = 112;  // Example module type
    constexpr uint8_t module_id   = 12;   // Example module ID
    constexpr uint8_t command     = 88;   // Example command code
    constexpr uint8_t event_code  = 221;  // Example event code

    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelState);
    comm_class.SendStateMessage(command, event_code);
    constexpr uint16_t expected_kernel[3] = {kernel_protocol, command, event_code};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 3; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // Module test
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleState);
    comm_class.SendStateMessage(module_type, module_id, command, event_code);
    constexpr uint16_t expected_module[5] = {module_protocol, module_type, module_id, command, event_code};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 5; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the functioning of the Communication class SendCommunicationErrorMessage() method. This test checks both the
// Module-targeted version and the overloaded Kernel-targeted version.
void SendCommunicationErrorMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    // Defines static payload components.
    constexpr uint8_t module_type   = 1;
    constexpr uint8_t module_id     = 2;
    constexpr uint8_t command       = 3;
    constexpr uint8_t error_code    = 4;
    comm_class.communication_status = 189;  // Manually sets the Communication class status code.
    uint8_t tl_status = comm_class.GetTransportLayerStatus();  // Extracts the current Transport Layer status.

    // Defines the prototype byte-code. Communication errors contain two uint8 values: Communication and
    // TransportLayer status codes.
    constexpr auto prototype_code = static_cast<uint8_t>(axmc_communication_assets::kPrototypes::kTwoUint8s);

    // Kernel error message
    // Error messages use the appropriate Data protocol code.
    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    comm_class.SendCommunicationErrorMessage(command, error_code);
    const uint16_t expected_kernel[6] = {kernel_protocol, 3, 4, prototype_code, 189, tl_status};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 6; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();                                               // Resets the mock port
    comm_class.communication_status = 65;                            // Resets the communication class status.
    tl_status = comm_class.GetTransportLayerStatus();  // Re-extracts the current Transport Layer status.

    // Module error message
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData);
    comm_class.SendCommunicationErrorMessage(module_type, module_id, command, error_code);
    const uint16_t expected_module[8] = {module_protocol, 1, 2, 3, 4, prototype_code, 65, tl_status};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 8; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the functioning of the Communication class SendServiceMessage() method for all valid protocols.
void TestSendServiceMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    constexpr uint8_t service_code = 111;  // Uses the same test service code for all messages.

    // Reception message
    auto protocol_code                   = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kReceptionCode);
    const uint16_t expected_reception[2] = {protocol_code, service_code};
    comm_class.SendServiceMessage(protocol_code, service_code);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 2; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_reception[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // Identification message
    protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kIdentification);
    const uint16_t expected_identification[2] = {protocol_code, service_code};
    comm_class.SendServiceMessage(protocol_code, service_code);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 2; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_identification[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the error-handling behavior of the Communication class SendServiceMessage() method.
void TestSendServiceMessageErrors()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    // Attempts to pass an invalid protocol code.
    comm_class.SendServiceMessage(200, 112);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol),
        comm_class.communication_status
    );
}

// Tests the errors associated with the ReceiveMessage() method of the Communication class
// These tests focus specifically on errors raised by only this method; COBS and CRC related errors should be
// tested by their respective test functions.
void TestReceiveMessageReceptionErrorNoCRCandCOBSCalculation()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, COMMAND, NO_BLOCK,
    // CYCLE_DELAY[4], DELIMITER, CRC[2]
    const uint8_t test_buffer[16] = {129, 10, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Converts the test_buffer (uint8_t array) into an uint16_t array for compatibility with the rx_buffer.
    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    // Does NOT calculate the CRC for the COBS-encoded buffer.
    // Copies the fully encoded package into the rx_buffer to simulate packet reception and test ReceiveData() method.
    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kReceptionError),
        comm_class.communication_status
    );
}

// kCommunicationParsingError
void TestReceiveMessageReceivedNoBytesToReceive()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kNoBytesToReceive),
        comm_class.communication_status
    );
}

void TestReceiveMessageReceivedRepeatedModuleCommand()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<1, 2> cobs_class;

    // Zero out command_message fields
    comm_class.repeated_module_command.module_type = 0;
    comm_class.repeated_module_command.module_id   = 0;
    comm_class.repeated_module_command.return_code = 0;
    comm_class.repeated_module_command.command     = 0;
    comm_class.repeated_module_command.noblock     = false;
    comm_class.repeated_module_command.cycle_delay = 0;

    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, COMMAND, NO_BLOCK,
    // CYCLE_DELAY[4], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }
    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.repeated_module_command.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.repeated_module_command.module_id);    // Check module_id
    TEST_ASSERT_EQUAL_UINT8(4, comm_class.repeated_module_command.return_code);  // Check return_code
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.repeated_module_command.command);      // Check command
    TEST_ASSERT_FALSE(comm_class.repeated_module_command.noblock);               // Check noblock
}

void TestReceiveMessageReceivedOneOffModuleCommand()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, COMMAND, NOBLOCK, DELIMITER, CRC[2]
    uint8_t test_buffer[12] = {129, 6, 0, 2, 0, 5, 8, 0, 0, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[12] = {};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.one_off_module_command.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(5, comm_class.one_off_module_command.module_id);    // Check module_id
    TEST_ASSERT_EQUAL_UINT8(8, comm_class.one_off_module_command.return_code);  // Check return_code
}

void TestReceiveMessageReceivedDequeueModuleCommand()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // The correct layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, DELIMITER, CRC[2]
    uint8_t test_buffer[10] = {129, 4, 0, 3, 19, 3, 0, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[10] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(19, comm_class.module_dequeue.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.module_dequeue.module_id);     // Check module_id
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.module_dequeue.return_code);   // Check return_code
}

void TestReceiveMessageReceivedKernelCommand()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // The correct layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, RETURN_CODE, COMMAND, DELIMITER, CRC[2]
    uint8_t test_buffer[9] = {129, 3, 0, 4, 0, 3, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[9] = {};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.kernel_command.return_code);  // Check return_code
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.kernel_command.command);      // Check return_code
}

void TestReceiveMessageReceivedModuleParameters()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // Instantiates a ModuleParameters with payload.
    // The layout is: START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE_TYPE, MODULE_ID, RETURN_CODE, DELIMITER, CRC[2]
    uint8_t test_buffer[10] = {129, 4, 0, 5, 1, 20, 1, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[10] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.module_parameter.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(20, comm_class.module_parameter.module_id);   // Check module_id
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.module_parameter.return_code);  // Check return_code
}

void TestReceiveMessageReceivedKernelParameters()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // The current layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, RETURN_CODE, DYNAMIC_PARAMETERS, DELIMITER, CRC[2]
    uint8_t test_buffer[10] = {129, 4, 0, 6, 1, 1, 1, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[10] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_parameters.return_code);                     // Check return_code
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_parameters.dynamic_parameters.action_lock);  // Check action_lock
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_parameters.dynamic_parameters.ttl_lock);     // Check ttl_lock
}

void TestReceiveMessageInvalidProtocolErrorArbitraryProtocolValue()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, OBJECT_SIZE, OBJECT, DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 100, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};  // 100 is an invalid protocol code

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kInvalidProtocol),
        comm_class.communication_status
    );
}

void TestReceiveMessageParsingErrorRepeatedModuleMissingParameter()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // Zeroes-out the test structure
    comm_class.repeated_module_command.module_type = 0;
    comm_class.repeated_module_command.module_id   = 0;
    comm_class.repeated_module_command.return_code = 0;
    comm_class.repeated_module_command.command     = 0;
    comm_class.repeated_module_command.noblock     = false;
    comm_class.repeated_module_command.cycle_delay = 0;

    // The correct layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, COMMAND, NO_BLOCK, CYCLE[4], DELIMITER, CRC[2]

    // This is missing a byte for CYCLE-DELAY
    uint8_t test_buffer[16] = {129, 9, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }
    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParsingError),
        comm_class.communication_status
    );
}

// Tests the errors associated with the ExtractParameter() method of the Communication class
// These tests focus specifically on errors raised by only this method; COBS and CRC errors should be
// tested by their respective test functions.
void TestExtractParametersExtractionForbidden()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<1, 2> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE_TYPE, MODULE_ID, OBJECT[6], DELIMITER, CRC[2]
    // The message is not a ModuleParameters message
    uint8_t test_buffer[16] = {129, 10, 0, 3, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    uint8_t extract_into[6] = {0};
    comm_class.ExtractModuleParameters(extract_into);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kExtractionForbidden),
        comm_class.communication_status
    );
}

void TestExtractParametersArrayDestination()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<1, 2> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL,  MODULE_TYPE, MODULE_ID, OBJECT[6], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 5, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    uint8_t extract_into[6] = {0};
    comm_class.ExtractModuleParameters(extract_into);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParametersExtracted),
        comm_class.communication_status
    );
}

void TestExtractParametersStructDestination()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<1, 2> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, OBJECT[6], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 5, 2, 3, 4, 9, 1, 2, 3, 4, 5, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    // Define a test array to extract parameters into
    struct TestStructure
    {
            uint8_t id      = 1;
            uint8_t data[5] = {0};
    } test_structure;

    // Call the ExtractParameters function, expecting a successful extraction
    comm_class.ExtractModuleParameters(test_structure);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParametersExtracted),
        comm_class.communication_status
    );
}

void TestExtractParametersSizeMismatchDestinationSizeLarger()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<1, 2> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL,  MODULE, ID, RETURN_CODE, OBJECT[3], DELIMITER, CRC[2]
    uint8_t test_buffer[16] = {129, 10, 0, 5, 2, 3, 4, 5, 1, 5, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[16] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    // Input structure with larger size than transmitted data
    struct DataMessage
    {
            uint8_t id       = 0;
            uint8_t data[10] = {0};
    } data_message;

    bool success = comm_class.ExtractModuleParameters(data_message);

    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParameterMismatch),
        comm_class.communication_status
    );
}

void TestExtractParametersSizeMismatchDestinationSizeSmaller()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<1, 2> cobs_class;

    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL,  MODULE, ID, RETURN_CODE, OBJECT[8], DELIMITER, CRC[2]
    uint8_t test_buffer[18] = {129, 12, 0, 5, 2, 3, 4, 5, 1, 2, 3, 4, 5, 18, 30, 0, 0, 0};

    // Simulates COBS encoding the buffer.
    uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[18] = {0};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    // Input structure with smaller size than transmitted data
    struct DataMessage
    {
            uint8_t id   = 0;
            uint8_t data = 0;
    } data_message;

    const bool success = comm_class.ExtractModuleParameters(data_message);

    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParameterMismatch),
        comm_class.communication_status
    );
}

void TestExtractParametersParsingErrorLargeTransmittedData()
{
    StreamMock<254> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor<> cobs_class;

    // Instantiates an array with larger size than the max payload size
    // Currently, the layout is:
    // START, PAYLOAD_SIZE, OVERHEAD, PROTOCOL, MODULE, ID, RETURN_CODE, OBJECT, DELIMITER, CRC[2]

    uint8_t test_buffer[266] = {};

    // Simulates COBS encoding the buffer.
    test_buffer[0]             = 129;
    test_buffer[1]             = 4;
    test_buffer[2]             = 0;
    test_buffer[3]             = 5;
    test_buffer[4]             = 2;
    test_buffer[5]             = 3;
    test_buffer[6]             = 4;
    const uint16_t packet_size = cobs_class.EncodePayload(test_buffer, 0);

    // Calculates the CRC for the COBS-encoded buffer.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, packet_size);

    // Adds the CRC to the end of the buffer.
    crc_class.AddCRCChecksumToBuffer(test_buffer, packet_size + 2, crc_checksum);

    uint16_t test_buffer_uint16[266] = {};
    for (size_t i = 0; i < sizeof(test_buffer); ++i)
    {
        test_buffer_uint16[i] = static_cast<uint16_t>(test_buffer[i]);
    }

    memcpy(mock_port.rx_buffer, test_buffer_uint16, sizeof(test_buffer_uint16));

    comm_class.ReceiveMessage();

    uint8_t extract_into[256] = {};
    comm_class.ExtractModuleParameters(extract_into);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationCodes::kParsingError),
        comm_class.communication_status
    );
}

// Specifies the test functions to be executed and controls their runtime. Use this function to determine which tests
// are run during test runtime and in what order. Note, each test function is encapsulated and will run even if it
// depends on other test functions ran before it which fail the tests.
int RunUnityTests()
{
    UNITY_BEGIN();

    // SendDataMessage
    RUN_TEST(TestSendDataMessage);

    // SendStateMessage
    RUN_TEST(TestSendStateMessage);

    // SendCommunicationErrorMessage
    RUN_TEST(SendCommunicationErrorMessage);

    // SendServiceMessage
    RUN_TEST(TestSendServiceMessage);
    RUN_TEST(TestSendServiceMessageErrors);

    // ReceiveMessage
    RUN_TEST(TestReceiveMessageReceptionErrorNoCRCandCOBSCalculation);
    RUN_TEST(TestReceiveMessageReceivedNoBytesToReceive);
    RUN_TEST(TestReceiveMessageReceivedRepeatedModuleCommand);
    RUN_TEST(TestReceiveMessageReceivedOneOffModuleCommand);
    RUN_TEST(TestReceiveMessageReceivedDequeueModuleCommand);
    RUN_TEST(TestReceiveMessageReceivedKernelCommand);
    RUN_TEST(TestReceiveMessageReceivedModuleParameters);
    RUN_TEST(TestReceiveMessageReceivedKernelParameters);
    RUN_TEST(TestReceiveMessageInvalidProtocolErrorArbitraryProtocolValue);
    RUN_TEST(TestReceiveMessageParsingErrorRepeatedModuleMissingParameter);

    //ExtractParameter
    RUN_TEST(TestExtractParametersExtractionForbidden);
    RUN_TEST(TestExtractParametersArrayDestination);
    RUN_TEST(TestExtractParametersStructDestination);
    RUN_TEST(TestExtractParametersSizeMismatchDestinationSizeSmaller);
    RUN_TEST(TestExtractParametersSizeMismatchDestinationSizeLarger);
    RUN_TEST(TestExtractParametersParsingErrorLargeTransmittedData);

    return UNITY_END();
}

// Defines the baud rates for different boards. Note, this list is far from complete and was designed for the boards
// the author happened to have at hand at the time of writing these tests. When testing on an architecture not
// explicitly covered below, it may be beneficial to provide the baudrate optimized for the tested platform.

// For Arduino Due, the maximum non-doubled stable rate is 5.25 Mbps at 84 MHz cpu clock.
#if defined(ARDUINO_SAM_DUE)
#define SERIAL_BAUD_RATE 5250000

// For Uno, Mega, and other 16 MHz AVR boards, the maximum stable non-doubled rate is 1 Mbps.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__) ||  \
    defined(__AVR_ATmega32U4__)
#define SERIAL_BAUD_RATE 1000000

// For all other boards the default 9600 rate is used. Note, this is a very slow rate, and it is very likely your board
// supports a faster rate. Select the most appropriate rate based on the CPU clock of your board, as it directly affects
// the error rate at any given baudrate. Boards that use a USB port (e.g., Teensy) ignore the baudrate value and
// instead default to the fastest speed supported by their USB interface (480 Mbps for most modern ports).
#else
#define SERIAL_BAUD_RATE 9600
#endif

// This is necessary for the Arduino framework testing to work as expected, which includes teensy. All tests are
// run inside setup function as they are intended to be one-shot tests
void setup()
{
    // Starts the serial connection.
    Serial.begin(SERIAL_BAUD_RATE);

    // Waits ~2 seconds before the Unity test runner establishes connection with a board Serial interface.
    delay(2000);

    // Runs the required tests
    RunUnityTests();

    // Stops the serial communication interface.
    Serial.end();
}

// Nothing here as all tests are done in a one-shot fashion using 'setup' function above
void loop()
{}
