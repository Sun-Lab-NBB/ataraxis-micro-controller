/**
 * @file
 *
 * @brief Provides the Kernel class used to manage the runtime of custom hardware modules and
 * integrate them with the companion host-computer (PC) control interface.
 *
 * Manages PC-microcontroller communication and schedules and executes commands addressed to custom hardware
 * modules. Due to the static API exposed by the (base) Module class, from which all custom module instances should
 * inherit, Kernel seamlessly integrates custom hardware modules with the centralized interface running on the
 * host-computer (PC).
 *
 * @note A single instance of this class should be created in the main.cpp file and used to manage the runtime. See
 * the example .cpp files included with the distribution of the library or
 * the https://github.com/Sun-Lab-NBB/ataraxis-micro-controller repository for details.
 */

#ifndef AXMC_KERNEL_H
#define AXMC_KERNEL_H

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "axmc_shared_assets.h"
#include "communication.h"
#include "module.h"

// The digitalWriteFast library aliases digitalWriteFast and digitalReadFast to the standard Arduino functions on every
// non-AVR target. Teensy boards ship always-inline register-level implementations of both, so removing the aliases
// exposes them. The pinModeFast alias is kept, as Teensy provides no fast counterpart for it.
#if defined(CORE_TEENSY)
#undef digitalWriteFast
#undef digitalReadFast
#endif

using namespace axmc_shared_assets;

/**
 * @brief Manages the runtime of one or more custom hardware module instances.
 *
 * The Kernel integrates all custom hardware module instances with the centralized control interface running on the
 * companion host-computer (PC). It handles the majority of the microcontroller-PC interactions.
 *
 * @warning After initialization, call the instance's Setup() method in the main setup() function and the RuntimeCycle()
 * method in the main loop() function of the main.cpp / main.ino file.
 *
 * @note During initialization, this class should be provided with an array of hardware module instances
 * that inherit from the Module class.
 */
class Kernel
{
    public:
        /// Defines the codes used by the Kernel class to communicate its runtime state to the PC.
        enum class kKernelStatusCodes : uint8_t
        {
            kStandby                = 0,   ///< Currently not used. Statically reserves 0 to NOT be a valid code.
            kSetupComplete          = 1,   ///< Setup() method runtime succeeded.
            kModuleSetupError       = 2,   ///< Setup() method runtime failed due to a module setup error.
            kReceptionError         = 3,   ///< Encountered a communication error when receiving data from the PC.
            kTransmissionError      = 4,   ///< Encountered a communication error when sending data to the PC.
            kInvalidMessageProtocol = 5,   ///< Received a message that uses an unsupported (unknown) protocol.
            kModuleParametersSet    = 6,   ///< Received and applied the parameters addressed to the module instance.
            kModuleParametersError  = 7,   ///< Unable to apply the received parameters to the module instance.
            kCommandNotRecognized   = 8,   ///< Received an unsupported (unknown) Kernel command.
            kTargetModuleNotFound   = 9,   ///< Unable to find the module with the requested combined type and ID code.
            kKeepAliveTimeout       = 10,  ///< The Kernel did not receive a keepalive message within the expected time.
        };

        /// Defines the codes for the supported Kernel commands.
        enum class kKernelCommands : uint8_t
        {
            kStandby            = 0,  ///< The standby code used during class initialization.
            kReceiveData        = 1,  ///< Checks and, if possible, receives PC-sent data. Not externally addressable.
            kResetController    = 2,  ///< Resets the software and hardware state of all managed assets.
            kIdentifyController = 3,  ///< Sends the ID of the controller to the PC.
            kIdentifyModules    = 4,  ///< Sequentially sends each managed module's combined Type+ID code to the PC.
            kKeepAlive          = 5,  ///< Resets the keepalive watchdog timer, starting a new keepalive cycle.
        };

        /**
         * @brief Initializes the necessary assets used to manage the runtime of the input hardware module instances.
         *
         * @param controller_id The unique identifier of the microcontroller that uses this Kernel instance. This
         * ID code has to be unique for all microcontrollers used at the same time. Valid values range from 1 to 255.
         * The value 0 is reserved.
         * @param communication The shared Communication instance used to bidirectionally communicate with the PC
         * during runtime.
         * @param module_array The array of pointers to custom hardware module instances. Each instance must inherit
         * from the base Module class, and the array must contain at least one instance.
         * @param keepalive_interval The interval, in milliseconds, used to derive the keepalive timeout. The Kernel
         * doubles this value to tolerate brief communication lapses, so emergency shutdown occurs after about twice
         * the supplied interval without a keepalive command from the PC. Setting this parameter to 0 disables the
         * keepalive mechanism.
         */
        template <const size_t kModuleNumber>
        Kernel(
            const uint8_t controller_id,
            Communication& communication,
            Module* (&module_array)[kModuleNumber],
            const uint32_t keepalive_interval = 0
        ) :
            _modules(module_array),
            _module_count(kModuleNumber),
            _controller_id(controller_id),
            _keepalive_interval(ResolveKeepaliveTimeout(keepalive_interval)),
            _communication(communication)
        {
            // While compiling an empty array should not be possible, ensures there is always at least one module to
            // manage.
            static_assert(
                kModuleNumber > 0,
                "At least one valid Module-derived class instance must be provided during Kernel class initialization."
            );
        }

        /// Returns the currently active Kernel command code.
        [[nodiscard]]
        uint8_t get_kernel_command() const
        {
            return _kernel_command;
        }

        /// Sets the currently active Kernel command code.
        void set_kernel_command(const uint8_t command)
        {
            _kernel_command = command;
        }

        /**
         * @brief Configures the hardware and software assets used by the Kernel and all managed hardware modules.
         *
         * @warning This method deactivates the built-in LED of the controller board. Seeing the LED constantly ON
         * (HIGH) after this method's runtime means the controller experienced a communication error when it tried
         * sending data to the PC. Seeing the LED blink on and off at ~2-second intervals indicates that the Kernel
         * failed the setup sequence.
         *
         * @note This method has to be called as part of the main setup() function.
         */
        void Setup()
        {
            _kernel_command = static_cast<uint8_t>(kKernelCommands::kResetController);

            // Ensures that the setup tracker is inactivated before running the rest of the setup code. This is needed
            // to support correct cycling through Setup() calls on Teensy boards that do not reset on USB connection
            // cycling and to properly handle PC-sent 'reset' commands. As a safety feature, this bricks the
            // controller if any managed module reports a failure to setup.
            _setup_complete = false;

            // Configures the built-in LED before anything that can fail, as the module loop below reports its failures
            // to the PC and falls back to the LED indicator when that report cannot be delivered. Leaving the pin
            // unconfigured until SetupKernel() would silence the indicator in exactly that case.
            ConfigureIndicatorLed();

            // Aborts the active and queued commands of every module before any module hardware is configured. Running
            // this as a separate pass ensures that a setup failure part-way through the loop below cannot leave a
            // later module holding an active command, as the suppressed runtime would never be able to finish it.
            // This step cannot fail.
            for (size_t index = 0; index < _module_count; index++)
            {
                _modules[index]->ResetExecutionParameters();
            }

            // Loops over each module and calls its SetupModule() virtual method. Note, expects that setup methods
            // generally cannot fail, but supports non-success return codes.
            for (size_t index = 0; index < _module_count; index++)
            {
                if (!_modules[index]->SetupModule())
                {
                    // If the setup fails, sends an error message to notify the PC of the setup failure.
                    const uint8_t error_object[2] = {
                        _modules[index]->get_module_type(),
                        _modules[index]->get_module_id(),
                    };

                    SendData(static_cast<uint8_t>(kKernelStatusCodes::kModuleSetupError), error_object);

                    // Returns without completing the setup. This 'bricks' the controller requiring firmware reset
                    // before it can re-attempt the setup process and receive data from the PC. The caller detects this
                    // outcome through the inactivated setup tracker and suspends the rest of the runtime.
                    return;
                }
            }

            // Sets up the hardware managed by the Kernel. This is done last to, if necessary, override any
            // module-derived modifications of the hardware reserved by the Kernel. This method cannot fail.
            SetupKernel();

            _setup_complete = true;

            // Informs the PC that the setup process has been completed.
            SendData(static_cast<uint8_t>(kKernelStatusCodes::kSetupComplete));
        }

        /**
         * @brief Carries out a single runtime cycle.
         *
         * During each runtime cycle, the instance first receives and processes all messages sent from the PC. All
         * messages other than commands addressed to the managed hardware modules are processed and handled immediately.
         * For example, Kernel-addressed commands are executed as soon as they are received. Module-addressed commands
         * are queued for execution and are executed after all available data is received and parsed.
         *
         * Once all data is received, the method loops over managed modules and executes one command execution stage
         * for each module.
         *
         * Finally, when keepalive tracking is active and no keepalive command arrives within the timeout, the method
         * reports the timeout to the PC and reruns Setup(), resetting all managed modules and aborting active commands.
         *
         * @note This method has to be repeatedly called as part of the main loop() function.
         */
        void RuntimeCycle()
        {
            // Tracks whether Kernel hardware setup was completed. This only needs to run once, since if the Kernel ever
            // fails setup, it bricks the controller until the firmware is reset.
            static bool once = true;

            // If the Setup method was not called, sets up the built-in LED control via the Kernel-specific setup
            // sequence known to be fail-prone. This is only done once.
            if (!_setup_complete && once)
            {
                SetupKernel();
                once = false;
            }

            // If the method is called before the Setup() method, instead of normal runtime continuously blinks the
            // LED to visually communicate setup error to the user.
            if (!_setup_complete)
            {
                if (_setup_error_blink_timer >= kSetupErrorBlinkDelay)
                {
                    _setup_error_led_state = !_setup_error_led_state;
                    digitalWriteFast(LED_BUILTIN, _setup_error_led_state);
                    _setup_error_blink_timer = 0;
                }
                return;  // Ends cycle. A firmware reset is needed to get out of this loop.
            }

            // Continuously parses the data received from the PC until all data is processed.
            _kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);
            while (true)
            {
                const uint8_t protocol = ReceiveData();
                bool break_loop = false;  // A flag used to break the while loop once all available data is received.
                int16_t target_module;    // Stores the index of the module targeted by a Module-addressed command.
                uint8_t return_code;

                // Uses the message protocol of the returned message to execute appropriate logic to handle the message.
                switch (static_cast<kProtocols>(protocol))
                {
                    // Returned protocol 0 indicates that the data was not received. This may be due to no data to
                    // receive or due to a reception pipeline error. In either case, this ends the reception loop.
                    case kProtocols::kUndefined: break_loop = true; break;

                    case kProtocols::kModuleParameters:
                        {
                            const ModuleParameters& header = _communication.get_module_parameters_header();
                            return_code                    = header.return_code;
                            if (return_code) SendReceptionCode(return_code);

                            // For module-addressed commands, attempts to resolve (discover) the addressed module. If
                            // this method succeeds, it returns an index (>=0) of the target module class inside the
                            // _modules array
                            target_module = ResolveTargetModule(header.module_type, header.module_id);

                            // Aborts early if the target module is not found, as indicated by the returned code being
                            // a negative number (-1).
                            if (target_module < 0) break;

                            Module* const target = _modules[static_cast<size_t>(target_module)];
                            if (!target->SetCustomParameters())
                            {
                                // If the module fails to process the parameters, as indicated by the API method
                                // returning 'false', sends an error message to the PC to communicate the error.
                                const uint8_t error_object[2] = {
                                    target->get_module_type(),
                                    target->get_module_id(),
                                };
                                SendData(
                                    static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersError),
                                    error_object
                                );
                            }
                            else
                            {
                                // If the parameters were set correctly, notifies the PC.
                                SendData(static_cast<uint8_t>(kKernelStatusCodes::kModuleParametersSet));
                            }
                            break;
                        }

                    case kProtocols::kKernelCommand:
                        return_code = _communication.get_kernel_command().return_code;
                        if (return_code) SendReceptionCode(return_code);

                        // This method resolves and executes the command logic. It automatically extracts the command
                        // code from the received message stored in the Communication class attribute.
                        RunKernelCommand();

                        // The reset command runs Setup(), which inactivates the setup tracker when a managed module
                        // fails to set up. Ends the reception loop in that case, as the controller is bricked and must
                        // not process any further PC messages until its firmware is reset.
                        if (!_setup_complete) break_loop = true;

                        // Restores the command tracker, so that every message the rest of this reception cycle emits
                        // reports the data reception command rather than the kernel command that has just finished.
                        _kernel_command = static_cast<uint8_t>(kKernelCommands::kReceiveData);
                        break;

                    case kProtocols::kDequeueModuleCommand:
                        {
                            const DequeueModuleCommand& message = _communication.get_module_dequeue();
                            return_code                         = message.return_code;
                            if (return_code) SendReceptionCode(return_code);
                            target_module = ResolveTargetModule(message.module_type, message.module_id);

                            // Aborts early if the target module is not found, as indicated by the returned code being
                            // a negative number (-1).
                            if (target_module < 0) break;

                            // Resets the queue of the target module. Note, this does not abort already running
                            // commands: they are allowed to finish gracefully.
                            _modules[static_cast<size_t>(target_module)]->ResetCommandQueue();
                            break;
                        }

                    case kProtocols::kOneOffModuleCommand:
                        {
                            const OneOffModuleCommand& message = _communication.get_one_off_module_command();
                            return_code                        = message.return_code;
                            if (return_code) SendReceptionCode(return_code);

                            target_module = ResolveTargetModule(message.module_type, message.module_id);

                            if (target_module < 0) break;

                            Module* const target = _modules[static_cast<size_t>(target_module)];

                            // The command queue reserves the code 0 to mark the absence of a command, so no module can
                            // execute it. Rejects it the way modules reject every other code they do not support,
                            // since queueing it would discard the command without any reply reaching the PC.
                            if (message.command == 0)
                            {
                                target->SendCommandRejection(message.command);
                                break;
                            }

                            // Uses an overloaded QueueCommand method that always sets the input command as
                            // non-recurrent.
                            target->QueueCommand(message.command, message.noblock);
                            break;
                        }

                    case kProtocols::kRepeatedModuleCommand:
                        {
                            const RepeatedModuleCommand& message = _communication.get_repeated_module_command();
                            return_code                          = message.return_code;
                            if (return_code) SendReceptionCode(return_code);

                            target_module = ResolveTargetModule(message.module_type, message.module_id);

                            if (target_module < 0) break;

                            Module* const target = _modules[static_cast<size_t>(target_module)];

                            // The command queue reserves the code 0 to mark the absence of a command, so no module can
                            // execute it. Rejects it the way modules reject every other code they do not support,
                            // since queueing it would discard the command without any reply reaching the PC.
                            if (message.command == 0)
                            {
                                target->SendCommandRejection(message.command);
                                break;
                            }

                            // Uses the non-overloaded QueueCommand method that always sets the input command to execute
                            // recurrently.
                            target->QueueCommand(message.command, message.noblock, message.cycle_delay);
                            break;
                        }

                    default:
                        // Every protocol code ReceiveData() can return is addressed by a dedicated arm above, and
                        // messages that use an unsupported protocol are reported by ReceiveData() itself. This arm
                        // therefore only satisfies the compiler's enumeration coverage requirement, and it ends the
                        // reception loop, as reaching it would mean the reception pipeline produced a code this
                        // switch cannot address.
                        break_loop = true;
                        break;
                }

                // If necessary, breaks the reception loop.
                if (break_loop) break;
            }

            // Once the loop above escapes due to running out of data to receive or a reception error, triggers a method
            // that sequentially executes Module commands in the blocking or non-blocking manner. Skips the execution
            // if a managed module failed to set up during this cycle, as the controller is bricked at that point and
            // must not drive any module hardware.
            if (_setup_complete) RunModuleCommands();

            // Keepalive status resolution. If the Kernel is configured to require keepalive messages, and it does not
            // receive a keepalive message within the configured interval, sends an error message to the PC and triggers
            // an emergency reset.
            if (_keepalive_enabled && (_since_previous_keepalive > _keepalive_interval))
            {
                SendData(static_cast<uint8_t>(kKernelStatusCodes::kKeepAliveTimeout), _keepalive_interval);

                // Resets the microcontroller runtime to default parameters, effectively clearing all command buffers
                // and hardware states.
                Setup();
            }
        }

    private:
        /// The delay, in milliseconds, between consecutive built-in LED toggles when signaling a setup error.
        static constexpr uint32_t kSetupErrorBlinkDelay = 2000;

        /// The multiplier applied to the requested keepalive interval to tolerate brief communication lapses.
        static constexpr uint32_t kKeepaliveIntervalMultiplier = 2;

        /// Tracks the currently active Kernel command. Used to send data and error messages to the PC.
        uint8_t _kernel_command = static_cast<uint8_t>(kKernelCommands::kStandby);

        /// Stores the managed custom hardware module classes.
        Module** _modules;

        /// Stores the size of the _modules array.
        const size_t _module_count;

        /// Stores the unique identifier code of the microcontroller that uses the Kernel instance.
        const uint8_t _controller_id;

        /// Stores the effective keepalive timeout, in milliseconds. The constructor sets this to twice the supplied
        /// interval to tolerate brief communication lapses between consecutive keepalive messages from the PC.
        const uint32_t _keepalive_interval;

        /// Tracks the time elapsed since receiving the last keepalive message.
        elapsedMillis _since_previous_keepalive;

        /// Determines whether the keepalive tracking is enabled.
        bool _keepalive_enabled = false;

        /// Stores the Communication instance used to bidirectionally communicate with the PC interface.
        Communication& _communication;

        /// Determines whether the Setup() method has been called to ensure that the instance is properly configured for
        /// runtime.
        bool _setup_complete = false;

        /// Tracks the time elapsed since the last built-in LED toggle of the setup error blink pattern.
        elapsedMillis _setup_error_blink_timer;

        /// Determines whether the built-in LED is currently lit as part of the setup error blink pattern.
        bool _setup_error_led_state = false;

        /**
         * @brief If a message sent from the PC is available for reception, decodes it into the Communication's
         * reception buffer.
         *
         * @returns The protocol code of the received message or 0 to indicate that no valid message was received.
         */
        [[nodiscard]]
        uint8_t ReceiveData() const
        {
            // Attempts to receive a message sent by the PC. If reception succeeds, returns the protocol code of the
            // received message
            if (_communication.ReceiveMessage()) return _communication.get_protocol_code();

            const uint8_t communication_status = _communication.get_communication_status();

            // A message that uses an unsupported protocol gets its own status code, which carries the rejected
            // protocol code to the PC. This is resolved here rather than in the RuntimeCycle protocol switch, as
            // ReceiveMessage() only surfaces the protocol code for the protocols it knows how to parse, which leaves
            // that switch unable to observe the rejected code.
            if (communication_status == static_cast<uint8_t>(kCommunicationStatusCodes::kInvalidProtocol))
            {
                SendData(
                    static_cast<uint8_t>(kKernelStatusCodes::kInvalidMessageProtocol),
                    _communication.get_protocol_code()
                );
            }

            // Data reception can also fail for two broad reasons. The first reason is that one of the classes involved
            // in data reception encounters an error. If this happens, the error needs to be reported to the PC.
            // However, it is also not uncommon for the reception method to 'fail' as there is no data to receive. This
            // is not an error and should be handled as a valid 'no need to do anything' case.
            else if (communication_status != static_cast<uint8_t>(kCommunicationStatusCodes::kNoBytesToReceive))
            {
                // For legitimately failed runtimes, sends an error message to the PC.
                _communication.SendCommunicationErrorMessage(
                    static_cast<uint8_t>(kKernelCommands::kReceiveData),
                    static_cast<uint8_t>(kKernelStatusCodes::kReceptionError)
                );
            }

            // Regardless of the source of communication failure, uses 'kUndefined' protocol (0) code return to indicate
            // that no valid data to process was received.
            return static_cast<uint8_t>(kProtocols::kUndefined);
        }

        /**
         * @brief Packages and sends the provided event_code and data object to the PC.
         *
         * @warning If sending the data fails for any reason, this method automatically emits an error message. Since
         * that error message may itself fail to be sent, the method also statically activates the built-in LED of the
         * board to visually communicate the encountered runtime error. Do not use the LED-connected pin or LED when
         * using this method to avoid interference!
         *
         * @note If the message is intended to communicate only the event code, do not provide the data object.
         * SendData() has an overloaded version specialized for sending event codes that is more efficient than the
         * data-containing version.
         *
         * @tparam ObjectType The type of the data object to be sent along with the message.
         * @param event_code The event that triggered the data transmission.
         * @param object The data object to be sent along with the message.
         */
        template <typename ObjectType>
        void SendData(const uint8_t event_code, const ObjectType& object) const
        {
            if (_communication.SendDataMessage(_kernel_command, event_code, object)) return;

            // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                _kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /**
         * @brief Packages and sends the provided event code to the PC.
         *
         * Overloads the SendData() method to optimize transmitting messages that only need to communicate the event.
         *
         * @param event_code The code of the event that triggered the data transmission.
         */
        void SendData(const uint8_t event_code) const
        {
            if (_communication.SendStateMessage(_kernel_command, event_code)) return;

            // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                _kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /// Sends the unique identifier code of the microcontroller that uses this Kernel instance to the PC.
        void SendControllerID() const
        {
            // Sends the identification message to the PC. If the message was sent, ends the runtime.
            if (_communication.SendServiceMessage<kProtocols::kControllerIdentification>(_controller_id)) return;

            // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                _kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /// Sequentially sends the combined type and ID code for each hardware module instance managed by this Kernel
        /// instance to the PC.
        void SendModuleTypeIDs() const
        {
            for (size_t index = 0; index < _module_count; ++index)
            {
                const uint16_t module_type_id = _modules[index]->get_module_type_id();

                // Sends the identification message to the PC. If the message was sent, moves on to the next module.
                if (_communication.SendServiceMessage<kProtocols::kModuleIdentification>(module_type_id)) continue;

                // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
                _communication.SendCommunicationErrorMessage(
                    _kernel_command,
                    static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
                );
            }
        }

        /**
         * @brief Sends the input reception code to the PC.
         *
         * @param reception_code The reception code received as part of an incoming message sent from the PC.
         */
        void SendReceptionCode(const uint8_t reception_code) const
        {
            if (_communication.SendServiceMessage<kProtocols::kReceptionCode>(reception_code)) return;

            // Otherwise, attempts to send a communication error to the PC and activates the LED indicator.
            _communication.SendCommunicationErrorMessage(
                _kernel_command,
                static_cast<uint8_t>(kKernelStatusCodes::kTransmissionError)
            );
        }

        /// Configures the built-in LED for output. Kept separate from SetupKernel(), as the LED backs the fallback
        /// error channel and therefore has to be usable before the rest of the setup sequence runs.
        static void ConfigureIndicatorLed()
        {
            pinModeFast(LED_BUILTIN, OUTPUT);
        }

        /**
         * @brief Doubles the requested keepalive interval to derive the effective keepalive timeout.
         *
         * The result saturates at the largest value the interval tracker can hold. Doubling an interval above half of
         * that maximum would otherwise wrap around, which silently disables the keepalive watchdog or shortens its
         * timeout by orders of magnitude.
         *
         * @param keepalive_interval The requested keepalive interval, in milliseconds.
         *
         * @returns The effective keepalive timeout, in milliseconds.
         */
        [[nodiscard]]
        static constexpr uint32_t ResolveKeepaliveTimeout(const uint32_t keepalive_interval)
        {
            // Compares against the halved maximum instead of evaluating the product, as the overflow this guards
            // against would happen inside the comparison itself.
            if (keepalive_interval > UINT32_MAX / kKeepaliveIntervalMultiplier) return UINT32_MAX;

            return keepalive_interval * kKeepaliveIntervalMultiplier;
        }

        /// Sets up the hardware and software assets managed by the Kernel class.
        void SetupKernel()
        {
            // Configures and deactivates the built-in LED. Currently, this is the only hardware system directly
            // managed by the Kernel.
            ConfigureIndicatorLed();
            digitalWriteFast(LED_BUILTIN, LOW);

            _keepalive_enabled = false;
        }

        /// Resolves and calls the method associated with the currently active Kernel command.
        void RunKernelCommand()
        {
            _kernel_command = _communication.get_kernel_command().command;
            switch (static_cast<kKernelCommands>(_kernel_command))
            {
                case kKernelCommands::kResetController: Setup(); return;

                case kKernelCommands::kIdentifyController: SendControllerID(); return;

                case kKernelCommands::kIdentifyModules: SendModuleTypeIDs(); return;

                case kKernelCommands::kKeepAlive:
                    // If necessary, activates the keepalive tracking. Prevents enabling the keepalive tracking if the
                    // interval is set to 0.
                    if (!_keepalive_enabled && _keepalive_interval > 0) _keepalive_enabled = true;

                    // Resets the keepalive interval tracker in-place
                    _since_previous_keepalive = 0;
                    return;

                default:
                    // If the command code was not matched with any valid code, sends an error message.
                    SendData(static_cast<uint8_t>(kKernelStatusCodes::kCommandNotRecognized));
            }
        }

        /**
         * @brief Finds the managed hardware module instance addressed by the input type and id codes.
         *
         * @note If this method is unable to resolve the target module, it automatically sends an error message to the
         * PC in addition to returning the '-1' error code.
         *
         * @param target_type The type (family) identifier of the addressed module.
         * @param target_id The unique identifier of the addressed module.
         *
         * @returns A non-negative integer representing the index of the module in the array of managed modules if the
         * addressed module is found. A '-1' value if the target module was not found.
         */
        [[nodiscard]]
        int16_t ResolveTargetModule(const uint8_t target_type, const uint8_t target_id) const
        {
            for (size_t index = 0; index < _module_count; index++)
            {
                if (_modules[index]->get_module_type() == target_type && _modules[index]->get_module_id() == target_id)
                {
                    return static_cast<int16_t>(index);
                }
            }

            // Otherwise, sends an error message to the PC and returns -1 to indicate that the target module was not
            // found.
            const uint8_t errors[2] = {target_type, target_id};
            SendData(static_cast<uint8_t>(kKernelStatusCodes::kTargetModuleNotFound), errors);
            return -1;
        }

        /// Resolves and, if necessary, executes the active command for each managed hardware module.
        void RunModuleCommands() const
        {
            for (size_t index = 0; index < _module_count; index++)
            {
                // First, determines which command to run, if any. This relies on the following choice hierarchy:
                // finish already active commands > execute a newly queued command > repeat a cyclic command.
                // If this method is able to resolve (activate) a command, it returns 'true'. Otherwise, there is no
                // command to run. If the module has no active command, aborts the iteration early to conserve CPU
                // cycles and speed up looping through modules.
                if (!_modules[index]->ResolveActiveCommand()) continue;

                // If RunActiveCommand is implemented properly, it returns 'true' if it matches the active command
                // code to the method to execute and 'false' otherwise. If the method returns 'false', the Kernel calls
                // an API method to send a predetermined error message to the PC and then discards the command. The
                // discard is required because an unrecognized command has no runtime that could complete itself, so
                // leaving it active would wedge the module and re-emit the same error on every runtime cycle.
                if (!_modules[index]->RunActiveCommand())
                {
                    _modules[index]->SendCommandActivationError();
                    _modules[index]->DiscardActiveCommand();
                }
            }
        }
};

#endif  // AXMC_KERNEL_H
