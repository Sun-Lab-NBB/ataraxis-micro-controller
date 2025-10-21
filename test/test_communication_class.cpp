// Dependencies
#include <Arduino.h>             // For Arduino functions
#include <unity.h>               // This is the C testing framework, no connection to the Unity game engine
#include "axmc_shared_assets.h"  // Library shared assets
#include "cobs_processor.h"      // COBSProcessor class
#include "communication.h"       // Communication class
#include "crc_processor.h"       // CRCProcessor class
#include "stream_mock.h"         // StreamMock class required for Communication class testing

// This function is called automatically before each test function. Currently not used.
void setUp()
{}

// This function is called automatically after each test function. Currently not used.
void tearDown()
{}

// Tests the functioning of the Communication's SendDataMessage() method.
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
    constexpr uint16_t expected_kernel[6] = {kernel_protocol, command, event_code, prototype_code, test_object, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 6; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // Module test
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData);
    comm_class.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object);
    constexpr uint16_t expected_module[8] =
        {module_protocol, module_type, module_id, command, event_code, prototype_code, test_object, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 8; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the functioning of the Communication's SendStateMessage() method.
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
    constexpr uint16_t expected_kernel[4] = {kernel_protocol, command, event_code, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // Module test
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleState);
    comm_class.SendStateMessage(module_type, module_id, command, event_code);
    constexpr uint16_t expected_module[6] = {module_protocol, module_type, module_id, command, event_code, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 6; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the functioning of the Communication's SendCommunicationErrorMessage() method.
void SendCommunicationErrorMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    // Defines static payload components.
    constexpr uint8_t module_type   = 1;
    constexpr uint8_t module_id     = 2;
    constexpr uint8_t command       = 3;
    constexpr uint8_t error_code    = 4;
    comm_class.communication_status = 189;                     // Manually sets the Communication class status code.
    uint8_t tl_status = comm_class.GetTransportLayerStatus();  // Extracts the current Transport Layer status.

    // Defines the prototype byte-code. Communication errors contain two uint8 values: Communication and
    // TransportLayer status codes.
    constexpr auto prototype_code = static_cast<uint8_t>(axmc_communication_assets::kPrototypes::kTwoUint8s);

    // Kernel error message
    // Error messages use the appropriate Data protocol code.
    constexpr uint16_t kernel_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    comm_class.SendCommunicationErrorMessage(command, error_code);
    const uint16_t expected_kernel[7] = {kernel_protocol, 3, 4, prototype_code, 189, tl_status, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 7; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_kernel[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();                                 // Resets the mock port
    comm_class.communication_status = 65;              // Resets the communication class status.
    tl_status = comm_class.GetTransportLayerStatus();  // Re-extracts the current Transport Layer status.

    // Module error message
    constexpr uint16_t module_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleData);
    comm_class.SendCommunicationErrorMessage(module_type, module_id, command, error_code);
    const uint16_t expected_module[9] = {module_protocol, 1, 2, 3, 4, prototype_code, 65, tl_status, 0};
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 9; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_module[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }
}

// Tests the functioning of the Communication's SendServiceMessage() method for all valid protocols.
void TestSendServiceMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);

    constexpr uint8_t service_code = 111;  // Uses the same test service code for all messages.

    // Reception message
    constexpr uint16_t expected_reception[2] = {
        static_cast<uint8_t>(axmc_communication_assets::kProtocols::kReceptionCode),
        service_code
    };
    comm_class.SendServiceMessage<axmc_communication_assets::kProtocols::kReceptionCode>(service_code);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 2; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_reception[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // ControllerIdentification message
    constexpr uint16_t expected_identification[2] = {
        static_cast<uint8_t>(axmc_communication_assets::kProtocols::kControllerIdentification),
        service_code
    };
    comm_class.SendServiceMessage<axmc_communication_assets::kProtocols::kControllerIdentification>(service_code);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 2; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(expected_identification[i], mock_port.tx_buffer[i + 3]);  // Verifies the message data
    }

    mock_port.reset();  // Resets the mock port

    // ModuleIdentification message
    constexpr uint16_t module_type_id                    = 300;
    constexpr uint16_t expected_module_identification[3] = {
        static_cast<uint8_t>(axmc_communication_assets::kProtocols::kModuleIdentification),
        44,
        1
    };
    comm_class.SendServiceMessage<axmc_communication_assets::kProtocols::kModuleIdentification>(module_type_id);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageSent),
        comm_class.communication_status
    );
    for (int i = 0; i < 3; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(
            expected_module_identification[i],
            mock_port.tx_buffer[i + 3]
        );  // Verifies the message data
    }
}

// Tests the functioning of the Communication's ReceiveMessage() method.
void TestReceiveMessage()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies the correct non-error no-success scenario, where the buffer does not contain any bytes to receive.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kNoBytesToReceive),
        comm_class.communication_status
    );

    mock_port.reset();  // Resets the mock port

    // Verifies RepeatedModuleCommand reception.
    uint8_t test_buffer_1[16] = {129, 10, 0, 1, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_1);
    crc_class.CalculateChecksum<false>(test_buffer_1);
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Receives and verifies the message data.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.repeated_module_command.module_type);   // Check module_type
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.repeated_module_command.module_id);     // Check module_id
    TEST_ASSERT_EQUAL_UINT8(4, comm_class.repeated_module_command.return_code);   // Check return_code
    TEST_ASSERT_EQUAL_UINT8(5, comm_class.repeated_module_command.command);       // Check command
    TEST_ASSERT_FALSE(comm_class.repeated_module_command.noblock);                // Check noblock
    TEST_ASSERT_EQUAL_UINT32(0, comm_class.repeated_module_command.cycle_delay);  // Check cycle_delay

    mock_port.reset();  // Resets the mock port

    // Verifies OneOffModuleCommand reception.
    uint8_t test_buffer_2[12] = {129, 6, 0, 2, 0, 1, 2, 3, 1, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_2);
    crc_class.CalculateChecksum<false>(test_buffer_2);
    for (size_t i = 0; i < sizeof(test_buffer_2); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_2[i]);
    }

    // Receives and verifies the message data.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(0, comm_class.one_off_module_command.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.one_off_module_command.module_id);    // Check module_id
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.one_off_module_command.return_code);  // Check return_code
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.one_off_module_command.command);      // Check command
    TEST_ASSERT_TRUE(comm_class.one_off_module_command.noblock);                // Check noblock

    mock_port.reset();  // Resets the mock port

    // Verifies DequeModuleCommand reception.
    uint8_t test_buffer_3[10] = {129, 4, 0, 3, 1, 2, 3, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_3);
    crc_class.CalculateChecksum<false>(test_buffer_3);
    for (size_t i = 0; i < sizeof(test_buffer_3); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_3[i]);
    }

    // Receives and verifies the message data.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.module_dequeue.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.module_dequeue.module_id);    // Check module_id
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.module_dequeue.return_code);  // Check return_code

    mock_port.reset();  // Resets the mock port

    // Verifies KernelCommand reception.
    uint8_t test_buffer_4[9] = {129, 3, 0, 4, 1, 2, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_4);
    crc_class.CalculateChecksum<false>(test_buffer_4);
    for (size_t i = 0; i < sizeof(test_buffer_4); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_4[i]);
    }

    // Receives and verifies the message data.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.kernel_command.return_code);  // Check return_code
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.kernel_command.command);      // Check return_code

    mock_port.reset();  // Resets the mock port

    // Verifies ModuleParameters reception. Note, this only verifies the header of the message. Parameter extraction is
    // verified by a different test function.
    uint8_t test_buffer_5[11] = {129, 4, 0, 5, 1, 2, 3, 4, 0, 0, 0};  // 4 simulates parameter object data

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_5);
    crc_class.CalculateChecksum<false>(test_buffer_5);
    for (size_t i = 0; i < sizeof(test_buffer_5); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_5[i]);
    }

    // Receives and verifies the message data.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kMessageReceived),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(1, comm_class.module_parameters_header.module_type);  // Check module_type
    TEST_ASSERT_EQUAL_UINT8(2, comm_class.module_parameters_header.module_id);    // Check module_id
    TEST_ASSERT_EQUAL_UINT8(3, comm_class.module_parameters_header.return_code);  // Check return_code

    mock_port.reset();  // Resets the mock port
}

// Tests the error-handling behavior of the Communication's ReceiveMessage() method.
void TestReceiveMessageErrors()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies that failing one of the reception steps, such as COBS decoding or CRC verification, correctly raises
    // kReceptionError.
    const uint8_t test_buffer_1[16] = {129, 10, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Skips COBS and CRC to produce an invalid packet. Writes the invalid packet into the mock reception buffer.
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Triggers and verifies the error.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kReceptionError),
        comm_class.communication_status
    );

    mock_port.reset();  // Resets the mock port

    // Verifies that receiving a message with an invalid protocol code correctly raises kInvalidProtocol. Note,
    // protocols used by the outgoing messages (such as KernelData) are also considered invalid.
    constexpr auto invalid_protocol = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kKernelData);
    uint8_t test_buffer_2[16]       = {129, 10, 0, invalid_protocol, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_2);
    crc_class.CalculateChecksum<false>(test_buffer_2);
    for (size_t i = 0; i < sizeof(test_buffer_2); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_2[i]);
    }

    // Triggers and verifies the error.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kInvalidProtocol),
        comm_class.communication_status
    );

    mock_port.reset();  // Resets the mock port

    // Verifies that receiving an incomplete message (message that deviates from its mandated layout) correctly raises
    // kParsingError.
    uint8_t test_buffer_3[16] = {129, 9, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_3);
    crc_class.CalculateChecksum<false>(test_buffer_3);
    for (size_t i = 0; i < sizeof(test_buffer_3); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_3[i]);
    }

    // Triggers and verifies the error.
    comm_class.ReceiveMessage();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParsingError),
        comm_class.communication_status
    );
}

// Tests the functioning of the Communication's ExtractModuleParameters() method.
void TestExtractModuleParameters()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies that extracting parameters into an array works as expected
    uint8_t test_buffer_1[16] = {129, 10, 0, 5, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_1);
    crc_class.CalculateChecksum<false>(test_buffer_1);
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Defines the test array which serves as a parameter extraction target.
    uint8_t extract_data[6] = {};

    // Receives the message, extracts and verifies parameter data.
    comm_class.ReceiveMessage();
    comm_class.ExtractModuleParameters(extract_data);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParametersExtracted),
        comm_class.communication_status
    );
    const uint8_t expected_data[6] = {
        5,
        1,
        2,
        3,
        4,
        5,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_data, extract_data, sizeof(expected_data));

    mock_port.reset();  // Resets the mock port

    //Verifies that extracting parameter data into a structure works as expected
    uint8_t test_buffer_2[16] = {129, 10, 0, 5, 2, 3, 4, 9, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_2);
    crc_class.CalculateChecksum<false>(test_buffer_2);
    for (size_t i = 0; i < sizeof(test_buffer_2); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_2[i]);
    }

    // Defines the test structure which serves as a parameter extraction target.
    struct TestStructure
    {
            uint8_t id      = 1;
            uint8_t data[5] = {};
    } PACKED_STRUCT test_structure;  // Has to be packed to properly align the data

    // Call the ExtractParameters function, expecting a successful extraction
    comm_class.ReceiveMessage();
    comm_class.ExtractModuleParameters(test_structure);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParametersExtracted),
        comm_class.communication_status
    );
    TEST_ASSERT_EQUAL_UINT8(9, test_structure.id);
    const uint8_t expected_data_2[5] = {
        1,
        2,
        3,
        4,
        5,
    };
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_data_2, test_structure.data, sizeof(expected_data_2));
}

// Tests the error-handling behavior of the Communication's ExtractModuleParameters() method.
void TestExtractModuleParametersErrors()
{
    StreamMock<60> mock_port;
    Communication comm_class(mock_port);
    CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
    COBSProcessor cobs_class;

    // Verifies that calling ExtractParameters() after receiving a message with a protocol code other than
    // kModuleParameters raises the ExtractionForbidden error.
    constexpr auto protocol_code = static_cast<uint8_t>(axmc_communication_assets::kProtocols::kUndefined);
    uint8_t test_buffer_1[16]    = {129, 10, 0, protocol_code, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 0, 0};

    // Packages test message data into the mock reception buffer.
    cobs_class.EncodePayload(test_buffer_1);
    crc_class.CalculateChecksum<false>(test_buffer_1);
    for (size_t i = 0; i < sizeof(test_buffer_1); ++i)
    {
        mock_port.rx_buffer[i] = static_cast<int16_t>(test_buffer_1[i]);
    }

    // Triggers and verifies the error.
    comm_class.ReceiveMessage();
    uint8_t extract_into[6] = {};
    comm_class.ExtractModuleParameters(extract_into);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kExtractionForbidden),
        comm_class.communication_status
    );

    mock_port.reset();  // Resets the mock port

    // Verifies that calling ExtractParameters() with a prototype whose size does not match the size of the parameters
    // block inside the serial buffer raises a kParameterMismatch error.
    comm_class.protocol_code = 5;  // Manually sets the protocol code to kModuleParameters

    // Prototype is larger than stored data size
    uint8_t invalid_prototype_2[12] = {};  // Prototype is smaller than stored data size
    TEST_ASSERT_FALSE(comm_class.ExtractModuleParameters(invalid_prototype_2));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axmc_shared_assets::kCommunicationStatusCodes::kParameterMismatch),
        comm_class.communication_status
    );
}

// Specifies the test functions executed at runtime.
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

    // ReceiveMessage
    RUN_TEST(TestReceiveMessage);
    RUN_TEST(TestReceiveMessageErrors);

    // ExtractModuleParameters
    RUN_TEST(TestExtractModuleParameters);
    RUN_TEST(TestExtractModuleParametersErrors);

    return UNITY_END();
}

// Defines the baud rates for different boards.

// For Arduino Due, the maximum non-doubled stable rate is 5.25 Mbps at 84 MHz cpu clock.
#if defined(ARDUINO_SAM_DUE)
#define SERIAL_BAUD_RATE 5250000

// For Uno, Mega, and other 16 MHz AVR boards, the maximum stable non-doubled rate is 1 Mbps.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__) ||  \
defined(__AVR_ATmega32U4__)
#define SERIAL_BAUD_RATE 1000000

// For all other boards the default 9600 rate is used.
#else
#define SERIAL_BAUD_RATE 9600
#endif

// This is necessary for the Arduino framework testing to work as expected, which includes teensy. All tests are
// run inside the setup function as they are intended to be one-shot tests
void setup()
{
    // Starts the serial connection.
    Serial.begin(SERIAL_BAUD_RATE);

    // Waits ~2 seconds before the Unity test runner establishes connection with a board Serial interface. For teensy,
    // this is less important, since it uses a USB interface which does not reset the board on connection.
    delay(2000);

    // Runs the required tests
    RunUnityTests();

    // Stops the serial communication interface.
    Serial.end();
}

// Nothing here as all tests are done in a one-shot fashion using the 'setup' function above
void loop()
{}
