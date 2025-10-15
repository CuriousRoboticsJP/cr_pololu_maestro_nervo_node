#include "maestro_device.h"

#include <chrono>
#include <thread>

MaestroDevice::MaestroDevice() : USB_Device()
{
}

MaestroDevice *MaestroDevice::Open(MaestroDevice::ProductID productID, const char *serial)
{
    // check for supported product
    MaestroDevice *maestroDevice = new MaestroDevice();
    if (MaestroDevice::ProductID::Maestro_6ch == productID)
    {
        maestroDevice->productID = productID;
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
    if (!usbDevice->Open(MaestroDevice::VendorID, (u_int16_t)productID, serial))
    {
        delete maestroDevice;
        return NULL;
    }

    // load parameters
    maestroDevice->servoMin = 4000;
    maestroDevice->servoMax = 8000;

    // initialize the device
    maestroDevice->InitializeServos();

    return maestroDevice;
}

bool MaestroDevice::InitializeServos()
{
    printf("[Maestro] Reinitializing device ...\n");
    return this->ControlTransfer(0x40, MaestroProtocol::REQUEST_REINITIALIZE, 0, 0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

bool MaestroDevice::SetPosition(uint8_t servo, uint16_t position)
{
    printf("[Maestro] set servo=%u position=%u\n", servo, position);
    return this->ControlTransfer(0x40, MaestroProtocol::REQUEST_SET_TARGET, position, servo);
}

bool MaestroDevice::GetServoStatus(MaestroProtocol::ServoStatus *servoStatus)
{
    const int servoDataSize = sizeof(MaestroProtocol::MaestroVariables) + (this->GetServoCount() * sizeof(MaestroProtocol::ServoStatus));
    uint8_t *data = new uint8_t[servoDataSize];
    bool ret = this->DataTransfer(0xC0, MaestroProtocol::REQUEST_GET_VARIABLES, data, servoDataSize);
    if (ret)
    {
        for (int i = 0; i < this->GetServoCount(); i++)
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

double MaestroDevice::ConvertToJointPosition(uint16_t position)
{
    return ((position / 4) - this->servoMin) / (this->servoMax - this->servoMin);
}

uint16_t MaestroDevice::ConvertToPWMPosition(double position)
{
    return 4 * (position * (this->servoMax - this->servoMin) + this->servoMin);
}
