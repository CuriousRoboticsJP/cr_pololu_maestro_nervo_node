#ifndef __POLOLU_MAESTRO_H
#define __POLOLU_MAESTRO_H

#include "usb_device.h"

class MaestroProtocol
{
public:
    // These are the values to put in to bRequest when making a setup packet
    // for a control transfer to the Maestro.
    enum Request
    {
        REQUEST_GET_PARAMETER = 0x81,
        REQUEST_SET_PARAMETER = 0x82,
        REQUEST_GET_VARIABLES = 0x83,
        REQUEST_SET_SERVO_VARIABLE = 0x84, // (also clears the serial timeout timer)
        REQUEST_SET_TARGET = 0x85,         // (also clears the serial timeout timer)
        REQUEST_CLEAR_ERRORS = 0x86,       // (also clears the serial timeout timer)
        REQUEST_REINITIALIZE = 0x90,
        REQUEST_ERASE_SCRIPT = 0xA0,
        REQUEST_WRITE_SCRIPT = 0xA1,
        REQUEST_SET_SCRIPT_DONE = 0xA2, // value.low.b is 0 for go, 1 for stop, 2 for single-step
        REQUEST_RESTART_SCRIPT_AT_SUBROUTINE = 0xA3,
        REQUEST_RESTART_SCRIPT_AT_SUBROUTINE_WITH_PARAMETER = 0xA4,
        REQUEST_RESTART_SCRIPT = 0xA5,
        REQUEST_START_BOOTLOADER = 0xFF
    };


    /// Represents the non-channel-specific variables that can be read from
    /// a Micro Maestro using REQUEST_GET_VARIABLES.
    struct MaestroVariables
    {
        /// The number of values on the data stack (0-32).  A value of 0 means the stack is empty.
        uint8_t stackPointer;

        /// The number of return locations on the call stack (0-10).  A value of 0 means the stack is empty.
        uint8_t callStackPointer;

        /// The error register.  Each bit stands for a different error (see uscError).
        /// If the bit is one, then it means that error occurred some time since the last
        /// GET_ERRORS serial command or CLEAR_ERRORS USB command.
        uint16_t errors;

        /// The address (in bytes) of the next bytecode instruction that will be executed.
        uint16_t programCounter;

        // /// Meaningless bytes to protect the program from stack underflows.
        // /// <remarks>This is public to avoid mono warning CS0169.</remarks>
        // int16_t buffer[3];

        // /// The data stack used by the script.  The values in locations 0 through stackPointer-1
        // /// are on the stack.
        // int16_t stack[32];

        // /// The call stack used by the script.  The addresses in locations 0 through
        // /// callStackPointer-1 are on the call stack.  The next return will make the
        // /// program counter go to callStack[callStackPointer-1].
        // uint16_t callStack[10];

        /// 0 = script is running.
        /// 1 = script is done.
        /// 2 = script will be done as soon as it executes one more instruction
        ///     (used to implement step-through debugging features)
        uint8_t scriptDone;

        /// Meaningless byte to protect the program from call stack overflows.
        /// <remarks>This is public to avoid mono warning CS0169.</remarks>
        uint8_t buffer2;

        // NOTE: after these variables, 6 copies of servoSetting follow on the Micro Maestro.
    };


    /// Represents the current status of a channel.
    struct ServoStatus
    {
        uint16_t position;
        uint16_t target;
        uint16_t speed;
        uint8_t acceleration;
    };

};

class MaestroDevice : public USB_Device
{
public:
    static const uint16_t VendorID = 0x1FFB;     // Pololu Corporation
    static const uint16_t ProductID_6ch = 0x89;  // Maestro 6-channel
    static const uint16_t ProductID_12ch = 0x8A; // Maestro 12-channel
    static const uint16_t ProductID_18ch = 0x8B; // Maestro 18-channel

    static MaestroDevice *Open(uint16_t productID = MaestroDevice::ProductID_6ch, const char *serial = NULL);

    int servoCount;

    bool SetPosition(uint8_t servo, uint16_t position);

    bool GetServoStatus(MaestroProtocol::ServoStatus *servoStatus);

    bool ResetErrors();

protected:
    MaestroDevice();

};

#endif