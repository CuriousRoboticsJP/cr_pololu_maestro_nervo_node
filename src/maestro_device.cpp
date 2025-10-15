#include "maestro_device.h"

MaestroDevice::MaestroDevice() : USB_Device()
{
}

MaestroDevice *MaestroDevice::Open(uint16_t productID, const char *serial)
{
    // check for supported product
    MaestroDevice *maestroDevice = new MaestroDevice();
    if (productID == MaestroDevice::ProductID_6ch)
    {
        maestroDevice->servoCount = 6;
    }
    else
    {
        printf("[Maestro] Only the 6ch Micro Maestro is supported by this library!\n");
        delete maestroDevice;
        return NULL;
    }

    // open the connection
    USB_Device *usbDevice = (USB_Device *)maestroDevice;
    if (!usbDevice->Open(MaestroDevice::VendorID, productID, serial))
    {
        delete maestroDevice;
        return NULL;
    }

    // initialize the device
    maestroDevice->InitializeServos();

    return maestroDevice;
}

bool MaestroDevice::InitializeServos()
{
    printf("[Maestro] Reinitializing device ...\n");
    return this->ControlTransfer(0x40, MaestroProtocol::REQUEST_REINITIALIZE, 0, 0);
}


bool MaestroDevice::SetPosition(uint8_t servo, uint16_t position)
{
    printf("[Maestro] set servo=%u position=%u\n", (uint32_t)servo, (uint32_t)position);
    return this->ControlTransfer(0x40, MaestroProtocol::REQUEST_SET_TARGET, position * 4, servo);
}

bool MaestroDevice::GetServoStatus(MaestroProtocol::ServoStatus *servoStatus)
{
    const int servoDataSize = sizeof(MaestroProtocol::MaestroVariables) + (this->servoCount * sizeof(MaestroProtocol::ServoStatus));
    uint8_t *data = new uint8_t[servoDataSize];
    bool ret = this->DataTransfer(0xC0, MaestroProtocol::REQUEST_GET_VARIABLES, data, servoDataSize);
    if (ret)
    {
        for (int i = 0; i < this->servoCount; i++)
        {
            servoStatus[i] = *(MaestroProtocol::ServoStatus*)(data + sizeof(MaestroProtocol::MaestroVariables) + sizeof(MaestroProtocol::ServoStatus) + i);
        }
    }

    return ret;
}

bool MaestroDevice::ResetErrors()
{
    printf("[Maestro] Resetting errors ...\n");
    if (!this->ControlTransfer(0x40, MaestroProtocol::REQUEST_CLEAR_ERRORS, 0, 0))
        return false;
    return true;
}