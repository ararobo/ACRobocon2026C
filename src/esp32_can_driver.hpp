#include <CAN.h>
#include <gn10_can/drivers/driver_interface.hpp>

class ESP32CANDriver : public gn10_can::drivers::ICanDriver
{
public:
    void setpin(uint8_t rx, uint8_t tx) {
        CAN.setPins(rx, tx);
    }
    bool begin(long baudRate)
    {
        return CAN.begin(baudRate);
    }
    bool send(const gn10_can::CANFrame& frame) override
    {
        // CANフレームをArduinoのCANライブラリの形式に変換して送信
        CAN.beginPacket(frame.id);
        CAN.write(frame.data.data(), frame.dlc);
        return CAN.endPacket();
    }
    bool receive(gn10_can::CANFrame& out_frame) override
    {
        // CANフレームを受信してArduinoのCANライブラリの形式から変換
        int packetSize = CAN.parsePacket();
        if (packetSize) {
            out_frame.id = CAN.packetId();
            out_frame.dlc = packetSize;
            CAN.readBytes(out_frame.data.data(), packetSize);
            return true;
        }
        return false;
    }
};