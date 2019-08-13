#pragma once

#include <string>
#include <stdint.h>

namespace Metabot
{
    class Packet
    {
        public:
            enum {
                ACK=0,
                MONITOR=1,
                STORE=2,
                PROGRAM=3,
                PRINT=4,
                RESET=6
            } PacketType;

            Packet(int type = 0);

            Packet& appendByte(uint8_t b);
            Packet& appendFloat(float f);
            Packet& appendSmallFloat(float f);
            Packet& appendInt(int i);
            Packet& appendShort(int16_t s);

            uint8_t available();
            uint8_t readByte();
            float readFloat();
            float readSmallFloat();
            int readInt();
            int16_t readShort();

            int type;

            void setPayload(std::string payload);
            int getSize();
            std::string toRaw();
        
        protected:
            std::string payload;
            size_t pos;
            uint8_t checksum;
    };
}
