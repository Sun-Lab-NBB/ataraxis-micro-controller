/**
 * @file
 * @brief The header file for the EncoderModule class, which allows interfacing with a rotary encoder to track the
 * direction and distance of a circular object movement.
 *
 * @attention This file is written in a way that is @b NOT compatible with any other library that uses
 * AttachInterrupt(). Disable the 'ENCODER_USE_INTERRUPTS' macro defined at the top of the file to make this file
 * compatible with other interrupt libraries.
 */

#ifndef AXMC_ENCODER_MODULE_H
#define AXMC_ENCODER_MODULE_H

// Note, this definition has to precede Encoder.h inclusion. This increases the resolution of the encoder, but
// interferes with any other library that makes use of AttachInterrupt() function.
#define ENCODER_USE_INTERRUPTS

#include <Arduino.h>
#include <Encoder.h>
#include "module.h"
#include "shared_assets.h"

/**
 * @brief Wraps an Encoder class instance and provides access to its pulse counter to evaluate the direction and
 * distance of a circular object movement.
 *
 * This module is specifically designed to interface with quadrature encoders and track the direction and number of
 * encoder pulses between class method calls. In turn, this allows monitoring the direction and distance moved by the
 * rotating object connected to the encoder.
 *
 * @note Largely, this class is an API wrapper around the Paul Stoffregen's Encoder library, and it relies on efficient
 * interrupt logic to increase encoder tracking precision. To achieve the best performance, make sure both Pin A and
 * Pin B are hardware interrupt pins.
 *
 * @tparam kPinA the digital interrupt pin connected to the 'A' channel of the quadrature encoder.
 * @tparam kPinB the digital interrupt pin connected to the 'B' channel of the quadrature encoder.
 * @tparam kPinX the digital pin connected to the index signal of the quadrature encoder. If your encoder does not
 * have an index pin, you will still have to 'sacrifice' one of the unused pins to provide a valid number to this
 * argument.
 * @tparam kInvertDirection if set to true, inverts the sign of the value returned by the encoder. By default, when
 * Pin B is triggered before Pin A, the pulse counter decreases, which corresponds to CW movement. When pin A is
 * triggered before pin B, the counter increases, which corresponds to the CCW movement. This flag allows reversing
 * this relationship, which is helpful to account for the mounting and wiring of the tracked rotating object.
 */
template <const uint8_t kPinA, const uint8_t kPinB, const uint8_t kPinX, const bool kInvertDirection>
class EncoderModule final : public Module
{
        // Ensures that the encoder pins are different.
        static_assert(kPinA != kPinB, "The two EncoderModule interrupt pins cannot be the same.");

        // Also ensures that encoder pins do not interfere with LED pin.
        static_assert(
            kPinA != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Channel A pin for EncoderModule "
            "instance."
        );
        static_assert(
            kPinB != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Channel B pin for EncoderModule "
            "instance."
        );
        static_assert(
            kPinX != LED_BUILTIN,
            "LED-connected pin is reserved for LED manipulation. Select a different Index pin for EncoderModule "
            "instance."
        );

    public:
        /// Assigns meaningful names to byte status-codes used to communicate module events to the PC. Note,
        /// this enumeration has to use codes 51 through 255 to avoid interfering with shared kCoreStatusCodes
        /// enumeration inherited from base Module class.
        enum class kCustomStatusCodes : uint8_t
        {
            kRotatedCCW = 51,  ///< The encoder was rotated in the CCW direction.
            kRotatedCW  = 52,  ///< The encoder was rotated in the CW.
            kPPR        = 53,  ///< The estimated Pulse-Per-Revolution (PPR) value of the encoder.
        };

        /// Assigns meaningful names to module command byte-codes.
        enum class kModuleCommands : uint8_t
        {
            kCheckState = 1,  ///< Gets the change in pulse counts and sign relative to last check.
            kReset      = 2,  ///< Resets the encoder's pulse tracker to 0.
            kGetPPR     = 3,  ///< Estimates the Pulse-Per-Revolution (PPR) of the encoder.
        };

        /// Initializes the class by subclassing the base Module class and instantiating the
        /// internal Encoder class.
        EncoderModule(
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
            return _communication.ExtractModuleParameters(_custom_parameters);
        }

        /// Executes the currently active command.
        bool RunActiveCommand() override
        {
            // Depending on the currently active command, executes the necessary logic.
            switch (static_cast<kModuleCommands>(GetActiveCommand()))
            {
                // ReadEncoder
                case kModuleCommands::kCheckState: ReadEncoder(); return true;
                // ResetEncoder
                case kModuleCommands::kReset: ResetEncoder(); return true;
                // GetPPR
                case kModuleCommands::kGetPPR: GetPPR(); return true;
                // Unrecognized command
                default: return false;
            }
        }

        /// Sets up module hardware parameters.
        bool SetupModule() override
        {
            // Since Encoder does not IndexPin, it has to be set to INPUT here
            pinMode(kPinX, INPUT);

            // Re-initializing the encoder class leads to global runtime shutdowns and is not really needed. Therefore,
            // instead of resetting the encoder hardware, the setup only resets the pulse counter.
            _encoder.write(0);

            // Resets the custom_parameters structure fields to their default values.
            _custom_parameters.report_CCW      = true;  // Defaults to report rotation in the CCW direction.
            _custom_parameters.report_CW       = true;  // Defaults to report rotation in the CW direction.
            _custom_parameters.delta_threshold = 1;     // By default, the minimum change magnitude to report is 1.

            return true;
        }

        ~EncoderModule() override = default;

    private:
        /// Stores custom addressable runtime parameters of the module.
        struct CustomRuntimeParameters
        {
                bool report_CCW          = true;  ///< Determines whether to report changes in the CCW direction.
                bool report_CW           = true;  ///< Determines whether to report changes in the CW direction.
                uint32_t delta_threshold = 1;  ///< Sets the minimum pulse count change (delta) for reporting changes.
        } __attribute__((packed)) _custom_parameters;

        /// The encoder class that abstracts low-level access to the Encoder pins and provides an easy API to retrieve
        /// the automatically incremented encoder pulse vector. The vector can be reset by setting it to 0, and the
        /// class relies on hardware interrupt functionality to maintain the desired precision.
        Encoder _encoder = Encoder(kPinA, kPinB);  // HAS to be initialized statically or the runtime crashes!

        /// The multiplier is used to optionally invert the pulse counter sign to virtually flip the direction of
        /// encoder readings. This is helpful if the encoder is mounted and wired in a way where CW rotation of the
        /// tracked object produces CCW readout in the encoder. In other words, this is used to virtually align the
        /// encoder readout direction to the tracked object rotation direction.
        static constexpr int32_t kMultiplier = kInvertDirection ? -1 : 1;  // NOLINT(*-dynamic-static-initializers)

        /// This variable is used to accumulate insignificant encoder readouts to be reused during further calls.
        /// This allows filtering the tracker object jitter while still accurately tracking small, incremental
        /// movements.
        int32_t _overflow = 0;

        /// Reads and resets the current encoder pulse counter and sends the result to the PC if it is significantly
        /// different from previous readout. Note, the result will be transformed into an absolute value and its
        /// direction will be codified as the event code of the message transmitted to PC.
        void ReadEncoder()
        {
            // Retrieves and, if necessary, flips the value of the encoder. The value tracks the number of pulses
            // relative to the previous reset command or the initialization of the encoder. Combines the pulse count
            // read from the encoder with the pulses already stored in the '_overflow' variable and resets the encoder
            // to 0 at each readout.
            _overflow += _encoder.readAndReset() * kMultiplier;

            // If encoder has not moved since the last call to this method, returns without further processing.
            if (_overflow == 0)
            {
                CompleteCommand();
                return;
            }

            // Converts the pulse count delta to an absolute value for the threshold checking below.
            auto delta = static_cast<uint32_t>(abs(_overflow));

            // If the value is negative, this is interpreted as rotation in the Clockwise direction.
            // If reporting CW rotation is allowed, sends the pulse count to the PC as an absolute value, using the
            // transmitted event status code to indicate the direction of movement. Note, the delta is only sent if
            // it is greater than or equal to the readout threshold value.
            if (_overflow < 0 && _custom_parameters.report_CW && delta >= _custom_parameters.delta_threshold)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kRotatedCW),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    delta
                );
                _overflow = 0;  // Resets the overflow, as all tracked pulses have been 'consumed' and sent to the PC.
            }

            // If the value is positive, this is interpreted as the CCW movement direction.
            // Same as above, if reporting the CCW movement is allowed and the delta is greater than or equal to
            // the readout threshold, sends the data to the PC.
            else if (_custom_parameters.report_CCW && delta >= _custom_parameters.delta_threshold)
            {
                SendData(
                    static_cast<uint8_t>(kCustomStatusCodes::kRotatedCCW),
                    communication_assets::kPrototypes::kOneUnsignedLong,
                    delta
                );
                _overflow = 0;  // Resets the overflow, as all tracked pulses have been 'consumed' and sent to the PC.
            }

            // Completes the command execution
            CompleteCommand();
        }

        /// Resets the encoder pulse counter back to 0. This can be used to reset the encoder without reading its data.
        void ResetEncoder()
        {
            _encoder.write(0);  // Resets the encoder tracker back to 0 pulses.
            CompleteCommand();
        }

        /// Estimates the Pulse-Per-Revolution (PPR) of the encoder by using the index pin to precisely measure the
        /// number of pulses per encoder rotation. Measures up to 11 full rotations and averages the pulse counts per
        /// each rotation to improve the accuracy of the computed PPR value.
        void GetPPR()
        {
            // First, establishes the measurement baseline. Since the algorithm does not know the current position of
            // the encoder, waits until the index pin is triggered. This is used to establish the baseline for tracking
            // the pulses per rotation.
            while (!digitalReadFast(kPinX))
                ;
            _encoder.write(0);  // Resets the pulse tracker to 0

            // Measures 10 full rotations (indicated by index pin pulses). Resets the pulse tracker to 0 at each
            // measurement and does not care about the rotation direction.
            uint32_t pprs = 0;
            for (uint8_t i = 0; i < 10; ++i)
            {
                // Delays for 100 milliseconds to ensure the object spins past the range of index pin trigger
                delay(100);

                // Blocks until the index pin is triggered.
                while (!digitalReadFast(kPinX))
                    ;

                // Accumulates the pulse counter into the summed value and reset the encoder each call.
                pprs += abs(_encoder.readAndReset());
            }

            // Computes the average PPR by using half-up rounding to get a whole number.
            // Currently, it is very unlikely to see a ppr > 10000, so casts the ppr down to uint16_t
            const auto average_ppr = static_cast<uint16_t>((pprs + 10 / 2) / 10);

            // Sends the average PPR count to the PC.
            SendData(
                static_cast<uint8_t>(kCustomStatusCodes::kPPR),
                communication_assets::kPrototypes::kOneUnsignedShort,
                average_ppr
            );

            // Completes the command execution
            CompleteCommand();
        }
};

#endif  //AXMC_ENCODER_MODULE_H
