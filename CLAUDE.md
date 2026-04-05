# Claude Code Instructions

## Session start behavior

At the beginning of each coding session, before making any code changes, you should build a comprehensive
understanding of the codebase by invoking the `/explore-codebase` skill.

This ensures you:
- Understand the project architecture before modifying code
- Follow existing patterns and conventions
- Do not introduce inconsistencies or break integrations

## Style guide compliance

You MUST invoke the appropriate style skill before performing ANY of the following tasks:

| Task                                   | Skill to invoke    |
|----------------------------------------|--------------------|
| Writing or modifying C++ code          | `/cpp-style`       |
| Writing or modifying README files      | `/readme-style`    |
| Writing or modifying Sphinx docs files | `/api-docs`        |
| Writing git commit messages            | `/commit`          |
| Writing or modifying skill files       | `/skill-design`    |

Each skill contains a verification checklist that you MUST complete before submitting any work. Failure to invoke the
appropriate skill results in style violations.

## Cross-referenced library verification

This project depends on the following Sun Lab library:

| Library                     | Import header            | Role                                                 |
|-----------------------------|--------------------------|------------------------------------------------------|
| ataraxis-transport-layer-mc | `axtlmc_shared_assets.h` | Bidirectional serial communication with CRC and COBS |

**Before writing code that interacts with a cross-referenced library, you MUST:**

1. **Check for local version**: Look for the library in the parent directory (e.g., `../ataraxis-transport-layer-mc/`).

2. **Compare versions**: If a local copy exists, compare its version against the latest release or main branch on
   GitHub:
   - Read the local `library.json` to get the current version
   - Use `gh api repos/Sun-Lab-NBB/{repo-name}/releases/latest` to check the latest release

3. **Handle version mismatches**: If the local version differs from the latest release or main branch, notify the user
   with the following options:
   - **Use online version**: Fetch documentation and API details from the GitHub repository
   - **Update local copy**: The user will pull the latest changes locally before proceeding

4. **Proceed with correct source**: Use whichever version the user selects as the authoritative reference for API
   usage, patterns, and documentation.

**Why this matters**: Skills and documentation may reference outdated APIs. Always verify against the actual library
state to prevent integration errors.

## Companion library synchronization

This library (`ataraxis-micro-controller`) and its Python counterpart (`ataraxis-communication-interface`) implement
the same hardware module framework on opposite ends of the connection. Any change to the message protocols, status
codes, prototype resolution, module addressing, or kernel command handling in this library MUST be synchronized with
the corresponding change in `ataraxis-communication-interface`, and vice versa.

**Before modifying any protocol-level behavior, you MUST:**

1. **Identify the companion repository**: Check for a local copy at `../ataraxis-communication-interface/`. If
   unavailable, use `gh api repos/Sun-Lab-NBB/ataraxis-communication-interface` to access the remote repository.

2. **Review the corresponding implementation**: Read the Python source that implements the same functionality you are
   modifying. Verify that the current MC and PC implementations are in sync before making changes.

3. **Plan synchronized changes**: Document what must change in both libraries. Notify the user of the required
   companion changes so they can be applied together.

4. **Never modify protocol behavior unilaterally**: A change applied to only one side of the connection will cause
   communication failures. Both libraries must agree on all synchronized elements listed below.

**What requires synchronization:**
- `kProtocols` enum values and meanings (13 message types)
- `kPrototypes` enum values and lookup table (252 type-count prototype codes)
- Message structure layouts (`RepeatedModuleCommand`, `OneOffModuleCommand`, `DequeueModuleCommand`, `KernelCommand`,
  `ModuleParameters`, `ModuleData`, `ModuleState`, `KernelData`, `KernelState`)
- `kCommunicationStatusCodes` values and meanings
- Kernel status codes and kernel command codes
- Module core status codes (0-3) and user-defined code range (51-250)
- Module addressing scheme (`module_type`, `module_id`)
- Prototype resolution logic (type index mapping and count-based lookup)

**What does NOT require synchronization:**
- Module subclass implementations (hardware-specific logic)
- Kernel runtime scheduling (keepalive intervals, LED error indication, command queue priority)
- Platform-specific buffer sizes and serial baud rates
- Test infrastructure, build system, and PlatformIO configuration
- Arduino/Teensy-specific timing and `elapsedMicros` usage

## Available skills

| Skill                    | Description                                                               |
|--------------------------|---------------------------------------------------------------------------|
| `/explore-codebase`      | Perform in-depth codebase exploration at session start                    |
| `/firmware-module`       | Guide creation of custom hardware Module subclasses                       |
| `/cpp-style`             | Apply Sun Lab C++ coding conventions (REQUIRED for all C++ changes)       |
| `/readme-style`          | Apply Sun Lab README conventions (REQUIRED for README changes)            |
| `/api-docs`              | Apply Sun Lab API documentation conventions                               |
| `/commit`                | Draft Sun Lab style-compliant git commit messages                         |
| `/skill-design`          | Generate and verify skill files and CLAUDE.md project instructions        |

## Project context

This is **ataraxis-micro-controller**, a C++17 PlatformIO library that provides a framework for integrating custom
hardware modules with a centralized PC control interface (`ataraxis-communication-interface`). It abstracts
communication, command scheduling, and runtime management, allowing developers to focus on hardware-specific module
logic. The library targets Arduino and Teensy microcontrollers within the
[Ataraxis](https://github.com/Sun-Lab-NBB/ataraxis) framework.

### Key areas

| Directory    | Purpose                                                          |
|--------------|------------------------------------------------------------------|
| `src/`       | Library source code (4 headers + main.cpp development entry)     |
| `test/`      | Unity test suite for Communication class                         |
| `examples/`  | TestModule implementation and main.cpp integration example       |
| `docs/`      | Sphinx + Breathe documentation source (consumes Doxygen XML)     |

### Architecture

- **Kernel** (`kernel.h`): Main runtime manager that integrates custom module instances with the PC interface.
  Constructor takes `controller_id` (1-255), `Communication` reference, `Module` pointer array, and optional
  `keepalive_interval` (milliseconds, 0 = disabled). `Setup()` initializes hardware and all modules; `RuntimeCycle()`
  runs one iteration of the main loop. Internally routes received messages by protocol code: kernel commands are
  handled directly, module commands and parameters are dispatched to the target module via `(module_type, module_id)`
  lookup. Implements keepalive monitoring (two consecutive missed windows trigger emergency reset) and LED-based error
  indication (constant HIGH = transmission error, 2-second blink = setup error).
- **Communication** (`communication.h`): Bidirectional message interface built on `TransportLayer` from
  `ataraxis-transport-layer-mc` with CRC-16 (polynomial 0x1021, initial 0xFFFF). Handles 13 message protocols
  (`kProtocols` enum) and 252 data prototypes (`kPrototypes` enum). Provides `SendDataMessage()` (event code + typed
  data payload), `SendStateMessage()` (event code only), `SendCommunicationErrorMessage()`, `SendServiceMessage()`
  (reception codes, controller/module identification), and `ReceiveMessage()` (parses incoming protocol-specific
  structures). `ExtractModuleParameters()` unpacks parameter payloads into user-defined structures.
- **Module** (`module.h`): Abstract base class for custom hardware modules. Constructor takes `module_type`,
  `module_id`, and `Communication` reference. Three pure virtual methods: `SetupModule()` (hardware initialization),
  `SetCustomParameters()` (unpack PC parameters), and `RunActiveCommand()` (execute active command). Manages command
  execution state via `ExecutionControlParameters` struct tracking active command, stage, non-blocking mode, queued
  command, and recurrent execution with configurable cycle delay. Protected utility methods: `QueueCommand()`,
  `ResolveActiveCommand()`, `CompleteCommand()`, `AbortCommand()`, `AdvanceCommandStage()`, `WaitForMicros()`,
  `SendData()`, `ExtractParameters()`, `AnalogRead()`, `DigitalRead()`.
- **Shared assets** (`axmc_shared_assets.h`): Namespace `axmc_shared_assets` containing `kCommunicationStatusCodes`
  enum, `kProtocols` enum (13 message types), `kPrototypes` enum (252 prototype codes), all `PACKED_STRUCT` message
  structures, the `kPrototypeLookup` compile-time 2D lookup table, `ResolvePrototype<T>()` template function, and
  reimplemented type traits for Arduino Mega compatibility (`is_array`, `array_extent`, `remove_extent`,
  `PrototypeTypeIndex`).

### Core components

| Component                    | File                   | Purpose                                                     |
|------------------------------|------------------------|-------------------------------------------------------------|
| `Kernel`                     | `kernel.h`             | Runtime management, command routing, keepalive monitoring   |
| `Communication`              | `communication.h`      | Bidirectional PC-microcontroller message handling           |
| `Module`                     | `module.h`             | Abstract base class for custom hardware modules             |
| `kProtocols`                 | `axmc_shared_assets.h` | 13 message protocol codes for PC-microcontroller exchange   |
| `kPrototypes`                | `axmc_shared_assets.h` | 252 data prototype codes for type-safe serialization        |
| `kPrototypeLookup`           | `axmc_shared_assets.h` | Compile-time 2D table mapping (type, count) to prototypes   |
| `ResolvePrototype<T>()`      | `axmc_shared_assets.h` | Compile-time prototype resolution from C++ types            |
| `ExecutionControlParameters` | `module.h`             | Command execution state tracking (stage, queue, recurrence) |

### Key patterns

- **Header-only library**: All source lives in `.h` files under `src/`. The `main.cpp` is excluded from library
  export and serves as a development entry point.
- **Stage-based command execution**: Commands execute across multiple `RuntimeCycle()` iterations in discrete stages.
  `AdvanceCommandStage()` transitions between stages; `WaitForMicros()` provides non-blocking delays. This enables
  cooperative multitasking across modules without threads or interrupts.
- **Command queue priority**: Active command runs to completion first, then newly queued commands, then recurrent
  commands (if cycle delay has elapsed). `ResolveActiveCommand()` implements this priority chain.
- **Compile-time prototype resolution**: `ResolvePrototype<T>()` maps C++ types to wire protocol codes at compile
  time via `kPrototypeLookup`. Supports 11 scalar types and C-style arrays at specific element counts. Static
  assertions catch unsupported type/count combinations during compilation.
- **PACKED_STRUCT serialization**: All message structures use `PACKED_STRUCT` for byte-level serialization with no
  compiler padding, ensuring binary compatibility with the companion Python library.
- **Status code returns**: All operations return enum status codes rather than throwing exceptions, consistent with
  embedded C++ patterns.
- **Platform-conditional compilation**: `elapsedMillis` dependency is included only for non-Teensy boards; Teensy
  provides native `elapsedMicros`. Buffer sizes vary by platform (Teensy: 248 bytes, Due: 244 bytes, Mega: 52 bytes).
- **`using namespace axmc_shared_assets;`** in source files is intentional for readability in the embedded context.
- **LED error indication**: `LED_BUILTIN` is used as a fallback error channel when serial communication has failed.
- **Module core status codes 0-3 are reserved** by the base `Module` class. Custom module status codes must use the
  range 51-250, unique within each module class.

### Dependencies

| Library                      | Purpose                                              | Platforms                |
|------------------------------|------------------------------------------------------|--------------------------|
| Arduino.h                    | Core Arduino framework (Serial, Stream, types)       | All                      |
| elapsedMillis                | Non-blocking timers on non-Teensy boards             | atmelsam, atmelavr       |
| digitalWriteFast             | Fast GPIO read/write operations                      | All                      |
| ataraxis-transport-layer-mc  | CRC-16 checksummed serial communication with COBS    | All                      |

### Build system

This is a PlatformIO library project. The `platformio.ini` defines three board environments:

| Environment | Board          | Platform   | Monitor speed |
|-------------|----------------|------------|---------------|
| `teensy41`  | Teensy 4.1     | teensy     | 115200        |
| `due`       | Arduino Due    | atmelsam   | 5250000       |
| `mega`      | Arduino Mega   | atmelavr   | 1000000       |

All environments use the Arduino framework, Unity test framework, and `-std=c++17` build flag.

### Development commands

```bash
pio run -e teensy41              # Build for Teensy 4.1
pio run -e due                   # Build for Arduino Due
pio run -e mega                  # Build for Arduino Mega
pio test -e teensy41             # Run Unity tests on Teensy 4.1
pio check -e teensy41            # Run static analysis
tox -e docs                      # Build Sphinx API documentation (Doxygen + Breathe)
```

### Workflow guidance

**Modifying Kernel:**

1. Review `src/kernel.h` for the current implementation
2. Understand the message routing architecture: `ReceiveMessage()` parses protocol, `RunKernelCommand()` handles
   kernel commands, `RunModuleCommands()` dispatches to modules via `ResolveTargetModule()`
3. All status codes return via `SendServiceMessage()` or `SendDataMessage()` — do not introduce exception handling
4. Keepalive logic uses a two-window miss threshold; modifying this affects safety guarantees
5. LED error indication is a fallback — it must remain functional even when serial communication has failed

**Modifying Communication:**

1. Review `src/communication.h` for the current implementation
2. Message structures are defined in `src/axmc_shared_assets.h` — changes to structure layout require companion
   library synchronization
3. `TransportLayer` from `ataraxis-transport-layer-mc` handles CRC and COBS — do not reimplement these
4. All 13 protocol codes must remain synchronized with `ataraxis-communication-interface`
5. `ExtractModuleParameters()` uses template parameter deduction — the storage object type determines extraction

**Implementing custom Module subclasses:**

1. Invoke the `/firmware-module` skill for guided module creation
2. Review `examples/example_module.h` for the TestModule reference implementation
3. Override `SetupModule()`, `SetCustomParameters()`, and `RunActiveCommand()` — all three are required
4. Use stage-based execution with `AdvanceCommandStage()` and `WaitForMicros()` for non-blocking operations
5. Custom parameters must use `PACKED_STRUCT` for serialization and match the PC-side parameter structure exactly
6. Custom status codes must be in the range 51-250 and unique within the module class

**Modifying prototype resolution:**

1. Review the `kPrototypes` enum and `kPrototypeLookup` table in `src/axmc_shared_assets.h`
2. `PrototypeTypeIndex<T>()` maps scalar types to row indices (11 types: bool through double)
3. `ResolvePrototype<T>()` handles both scalar types and C-style arrays via `is_array` trait detection
4. Adding new prototype entries requires updating both the enum, the lookup table, and the companion Python library
5. Maximum payload size is constrained by `TransportLayer` buffer sizes (platform-dependent)

**Important considerations:**

- Maximum payload size varies by platform: Teensy (248 bytes), Due (244 bytes), Mega (52 bytes)
- The reimplemented type traits in `axmc_shared_assets.h` exist because Arduino Mega lacks `<type_traits>`
- `library.json` controls what gets exported to the PlatformIO registry — `main.cpp` is explicitly excluded
- Controller ID 0 is reserved; valid range is 1-255
- Module type and module ID are both `uint8_t` — the `(type, id)` pair must be unique across all modules managed by
  a single Kernel instance
