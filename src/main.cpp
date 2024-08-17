#include <Arduino.h>
#include <elapsedMillis.h>
#include "transport_layer.h"

// Initializes the serial protocol class. Note, passes Serial class reference but does not open the serial port.
TransportLayer<uint16_t>
    protocol(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000, false);  // NOLINT(*-interfaces-global-init)
uint8_t in_data[7] = {1, 2, 3, 4, 5, 6, 7};
elapsedMicros timer;

void setup()
{
    Serial.begin(115200);  // Opens the Serial port, baud rate is not relevant for teensies
}

uint32_t prev_cycle = 0;
uint16_t add_index  = 0;

void loop()
{
    if (protocol.Available())
    {
        timer = 0;
        if (protocol.ReceiveData())
        {
            protocol.ReadData(in_data);
            add_index = protocol.WriteData(in_data, 0);
            protocol.WriteData(prev_cycle, add_index);
            protocol.SendData();
            prev_cycle = timer;
        }
    }
}
