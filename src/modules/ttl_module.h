//
// Created by ikond on 10/19/2024.
//

#ifndef AXMC_TTL_MODULE_H
#define AXMC_TTL_MODULE_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "module.h"
#include "shared_assets.h"

template <const uint8_t kOutputPin, const uint8_t kInputPin>
class TTLModule final : public Module
{
        // The only reason why pins are accessed via template parameter is to enable this static assert here
        static_assert(kOutputPin != kInputPin, "Input and output pins cannot be the same.");

    public:

        TTLModule(
            const uint8_t module_type,
            const uint8_t module_id,
            Communication& communication,
            const shared_assets::DynamicRuntimeParameters& dynamic_parameters
        ) :
            Module(module_type, module_id, communication, dynamic_parameters)
        {}

        /// Overwrites the custom_parameters structure memory with the data extracted from the Communication
        /// reception buffer.
        bool SetCustomParameters() override
        {
            // Extracts the received parameters into the _custom_parameters structure of the class. If extraction fails,
            // returns false. This instructs the Kernel to execute the necessary steps to send an error message to the
            // PC.
            if (!_communication.ExtractParameters(_custom_parameters)) return false;
            return true;
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
                        if (WaitForMicros(1))
                        {
                            CompleteCommand();
                        }
                    }
                    break;
                case 4: break;
                default: return false;  // Unrecognized command
            }
            return true;  // Command executed successfully
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            pinMode(kOutputPin, OUTPUT);
            pinMode(kInputPin, INPUT);
            return true;
        }

        /// Resets the custom_parameters structure fields to their default values.
        bool ResetCustomAssets() override
        {
            _custom_parameters.command = 0;
            _custom_parameters.stage   = 0;
            return true;
        }

        ~TTLModule() override = default;

    private:
        struct CustomRuntimeParameters
        {
                uint8_t command = 0;  ///< Currently executed (in-progress) command.
                uint8_t stage   = 0;  ///< Stage of the currently executed command.
        } _custom_parameters;
};

#endif  //AXMC_TTL_MODULE_H
