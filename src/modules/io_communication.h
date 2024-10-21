//
// Created by ikond on 10/19/2024.
//

#ifndef IO_COMMUNICATION_H
#define IO_COMMUNICATION_H

#include <Arduino.h>
#include "module.h"
#include "shared_assets.h"

template <uint8_t kOutputPin = 255, uint8_t kInputPin = 255>
class IOCommunication final : public Module
{
    public:
        IOCommunication(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters)
        {}

        static void test_function()
        {}

        bool SetCustomParameters() override
        {
            // Example implementation: this could involve setting module-specific parameters from Communication data
            // Here we assume a generic placeholder for a parameter object. You can customize based on your application
            return false;
        }

        bool RunActiveCommand() override
        {
            // Example logic for handling custom commands
            switch (GetActiveCommand())
            {
                case 1:
                    // Example command logic
                    digitalWrite(kOutputPin, HIGH);
                    break;
                case 2: digitalWrite(kOutputPin, LOW); break;
                case 3:  // Blinker test
                    if (GetCommandStage() == 1)
                    {
                        AdvanceCommandStage();
                    }

                    if (GetCommandStage() == 2)
                    {
                        if (WaitForMicros(1000000))
                        {
                            digitalWrite(LED_BUILTIN, HIGH);
                            delay(1000);
                            digitalWrite(LED_BUILTIN,LOW);
                            CompleteCommand();
                        }
                    }
                    break;
                case 4: break;
                default: return false;  // Unrecognized command
            }
            return true;  // Command executed successfully
        }

        bool SetupModule() override
        {
            // Set up the module, e.g., setting pin modes
            pinMode(kOutputPin, OUTPUT);
            pinMode(kInputPin, INPUT);
            return true;
        }

        bool ResetCustomAssets() override
        {
            // Reset custom parameters or assets here, if necessary
            // For instance, resetting custom internal states or objects
            return true;
        }

        ~IOCommunication() override = default;
};

#endif  //IO_COMMUNICATION_H
