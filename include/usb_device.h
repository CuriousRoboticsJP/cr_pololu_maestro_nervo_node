#ifndef __USB_DEVICE_H
#define __USB_DEVICE_H

#include <libusb-1.0/libusb.h>

#include <stdio.h>
#include <string>
#include <cstring>


class USB_Device
{
public:
    USB_Device();

    ~USB_Device();

    bool Open(uint16_t vendorID, uint16_t productID, const char *serial = NULL);

    void Close();

    libusb_device *Find(uint16_t vendorID, uint16_t productID, const char *serial = NULL);

    inline const char *GetSerial() const { return mSerial.c_str(); }

    inline const char *GetVendor() const { return mVendor.c_str(); }

    inline const char *GetProduct() const { return mProduct.c_str(); }

    bool ControlTransfer(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, void *data, uint32_t size);

    bool ControlTransfer(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index);

    bool DataTransfer(uint8_t requestType, uint8_t request, void *data, uint32_t size);

protected:
    libusb_device *mDevice;
    libusb_device_handle *mDeviceContext;
    libusb_context *mUSBContext;

    std::string mVendor;
    std::string mProduct;
    std::string mSerial;

    uint16_t mVendorID;
    uint16_t mProductID;
};

#endif