#pragma once
#include <string>
#include <stdexcept>
#include <libusb-1.0/libusb.h>

/**
 * Thrown when no valid Maestro device is found
 */
class NoMaestroFoundError : public std::runtime_error {
public:
    NoMaestroFoundError() : std::runtime_error("NoMaestroFoundError") { }
};

/**
 * Thrown when a valid device is found but can not be opened or otherwise initialized.
 */
class MaestroInitializationError : public std::runtime_error {
public:
    MaestroInitializationError(std::string what) : std::runtime_error(what) { }
    MaestroInitializationError(char* what) : std::runtime_error(what) { }
};

/**
 * Thrown when a control command to an already open device fails.
 */
class MaestroControlError : public std::runtime_error {
public:
    MaestroControlError(std::string what) : std::runtime_error(what) { }
    MaestroControlError(char* what) : std::runtime_error(what) { }
};

/**
 * Wrapper class for a Pololu Micro Maestro.
 * Uses libusb-1.0 to establish a connection and abtracts the basic control.
 */
class Maestro {
private:

    /**
     * vendor id for all maestro devices
     */
    static const uint16_t vendorId;
    
    /**
     * number of product ids
     */
    static const size_t nProductIds;
    
    /**
     * array of all valid product ids for maestro devices
     */
    static const uint16_t productIds[];

    /**
     * Given a device descriptor, determine if it's a Maestro device.
     */
    static bool isMaestroDevice(libusb_device_descriptor& desc);

    /**
     * product id for the device this instance has a handle to
     */
    uint16_t productId;

    libusb_context* ctx;
    libusb_device_handle* deviceHandle;

    /**
     * Helper method, calls controlTransfer on the underlying device handle.
     * @see libusb_control_transfer
     */
    int controlTransfer(uint8_t type, uint8_t request, uint16_t value, uint16_t index);

    /**
     * Throw if the passed return code is an error (< 0), otherwise just pass it through
     * @param code return code
     * @tparam T error type, needs a constructor with the signature T(std::string)
     */
    template <class T>
    static int throwIfError(int code);

    /**
     * Return error message associated with Maestro return codes.
     */
    static std::string errorDescription(int error);

public:
    /**
     * Basic constructor, will open the first valid device it finds.
     */
    Maestro();

    /**
     * Close underlying libusb handle, clean up.
     */
    ~Maestro();

    /**
     * Set the target pulse width for a particular servo. The PWM signal will gradually approach the target value based on the curent speed setting.
     * @param servo 0-indexed servo number.
     * @param value Pulse width in 0.25µs increments (e.g. for 1000µs you should pass 4000)
     */
    void setTarget(uint8_t servo, uint16_t value);

    /**
     * Set the maximum travel speed. This controls how quickly the PWM signal will move to the target value.
     * @param servo 0-indexed servo number.
     * @param value Maximun pulse width change rate, in 0.25µs/10ms
     */
    void setSpeed(uint8_t servo, uint16_t value);

    /**
     * Set the maximum acceleration.
     * @param servo 0-indexed servo number.
     * @param value Has a value in range (0..255) with units 0.25µs/10ms/80ms
     */
    void setAcceleration(uint8_t servo, uint16_t value);
};
