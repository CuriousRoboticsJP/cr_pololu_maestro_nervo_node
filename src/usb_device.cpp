#include "usb_device.h"

USB_Device::USB_Device()
{
    this->mDevice = NULL;
    this->mDeviceContext = NULL;
    this->mUSBContext = NULL;

    this->mVendorID = 0;
    this->mProductID = 0;
}

USB_Device::~USB_Device()
{
    this->Close();
}

bool USB_Device::Open(uint16_t vendorID, uint16_t productID, const char *serial)
{
    this->mDevice = this->Find(vendorID, productID, serial);

    if (!this->mDevice)
        return false;

    if (libusb_open(this->mDevice, &this->mDeviceContext) != 0)
    {
        printf("[usb] failed to open USB device vendor=%#04x product=%#04x serial=%s\n", vendorID, productID, serial);
        return false;
    }

    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(this->mDevice, &desc) != 0)
        return false;

    this->mVendorID = desc.idVendor;
    this->mProductID = desc.idProduct;

    // get serial number
    char buffer[512];
    memset(buffer, 0, sizeof(buffer));

    if (libusb_get_string_descriptor_ascii(this->mDeviceContext, desc.iSerialNumber, (uint8_t *)buffer, sizeof(buffer)) > 0)
        this->mSerial = buffer;

    if (libusb_get_string_descriptor_ascii(this->mDeviceContext, desc.iManufacturer, (uint8_t *)buffer, sizeof(buffer)) > 0)
        this->mVendor = buffer;

    if (libusb_get_string_descriptor_ascii(this->mDeviceContext, desc.iProduct, (uint8_t *)buffer, sizeof(buffer)) > 0)
        this->mProduct = buffer;

    printf("[usb] opened '%s %s' (vendor=%#04x product=%#04x serial=%s)\n", this->mVendor.c_str(), this->mProduct.c_str(), this->mVendorID, this->mProductID, this->mSerial.c_str());
    return true;
}

void USB_Device::Close()
{
    if (this->mDeviceContext != NULL)
    {
        printf("[usb] closing device '%s %s' (vendor=%#04x product=%#04x serial=%s)\n", this->mVendor.c_str(), this->mProduct.c_str(), this->mVendorID, this->mProductID, this->mSerial.c_str());
        libusb_close(this->mDeviceContext);
        this->mDeviceContext = NULL;
    }

    if (this->mUSBContext != NULL)
    {
        libusb_exit(this->mUSBContext);
        this->mUSBContext = NULL;
    }
}

libusb_device *USB_Device::Find(uint16_t vendorID, uint16_t productID, const char *serial)
{
    // open USB context
    if (!this->mUSBContext && libusb_init(&this->mUSBContext) != 0)
    {
        printf("[usb] failed to create libusb context\n");
        return NULL;
    }

    libusb_device **devList;
    const uint32_t numDevices = libusb_get_device_list(this->mUSBContext, &devList);
    printf("[usb] %u devices\n\n", numDevices);
    for (uint32_t n = 0; n < numDevices; n++)
    {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devList[n], &desc) != 0)
            continue;

        printf("    [%02u] \tvendor %#04x\tproduct %#04x\tserial %d\n", n, desc.idVendor, desc.idProduct, desc.iSerialNumber);
        if (vendorID != 0 && vendorID != desc.idVendor)
            continue;
        if (productID != 0 && productID != desc.idProduct)
            continue;
        if (serial != NULL)
        {
            libusb_device_handle *deviceCtx;
            if (libusb_open(devList[n], &deviceCtx) != 0)
            {
                printf("[usb] failed to open USB device for reading serial (vendor=%#04x product %#04x)\n", desc.idVendor, desc.idProduct);
                continue;
            }

            // get serial number
            char buffer[512];
            memset(buffer, 0, sizeof(buffer));
            libusb_get_string_descriptor_ascii(deviceCtx, desc.iSerialNumber, (uint8_t *)buffer, sizeof(buffer));
            libusb_close(deviceCtx);
            if (strcmp(serial, buffer) == 0)
                break;
        }

        return devList[n];
    }

    printf("[usb] couldn't find device with vendor=%#04x product=%#04x serial=%s\n", vendorID, productID, serial);
    return NULL;
}

bool USB_Device::ControlTransfer(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, void *data, uint32_t size)
{
    const int res = libusb_control_transfer(this->mDeviceContext, requestType, request, value, index, (uint8_t *)data, size, 5000);
    if (res != (int)size)
    {
        printf("[usb] usb control transfer failed (size=%i result=%i) (vendor='%s' product='%s' serial=%s\n", size, res, this->mVendor.c_str(), this->mProduct.c_str(), this->mSerial.c_str());
        return false;
    }

    return true;
}

bool USB_Device::ControlTransfer(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index)
{
    return this->ControlTransfer(requestType, request, value, index, 0, 0);
}

bool USB_Device::DataTransfer(uint8_t requestType, uint8_t request, void *data, uint32_t size)
{
    return this->ControlTransfer(requestType, request, 0, 0, data, size);
}