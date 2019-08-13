#include <iostream>
#include "Packet.h"

namespace Metabot
{
    Packet::Packet(int type)
        : type(type), pos(0), checksum(0)
    {
    }

    Packet& Packet::appendByte(uint8_t b)
    {
        payload += b;
        checksum += b;
        
        return *this;
    }

    Packet& Packet::appendInt(int i)
    {
        appendByte((i>>24)&0xff);
        appendByte((i>>16)&0xff);
        appendByte((i>>8)&0xff);
        appendByte((i>>0)&0xff);
        
        return *this;
    }
    
    Packet& Packet::appendShort(int16_t s)
    {
        appendByte((s>>8)&0xff);
        appendByte((s>>0)&0xff);
        
        return *this;
    }
    
    Packet& Packet::appendFloat(float f)
    {
        appendInt(f*1000.0);

        return *this;
    }
    
    Packet& Packet::appendSmallFloat(float f)
    {
        appendShort(f*10.0);

        return *this;
    }
    
    uint8_t Packet::available()
    {
        return payload.size()-pos;
    }

    uint8_t Packet::readByte()
    {
        if (pos >= payload.size()) {
            std::cerr << "Error: trying to read too much data (" << pos << "/" << payload.size() << ")" << std::endl;
            exit(0);
            return 0;
        }
        return payload[pos++];
    }

    float Packet::readFloat()
    {
        return readInt()/1000.0;
    }
    
    float Packet::readSmallFloat()
    {
        return readShort()/10.0;
    }

    int Packet::readInt()
    {
        int i = 0;
        i |= readByte()<<24;
        i |= readByte()<<16;
        i |= readByte()<<8;
        i |= readByte()<<0;

        return i;
    }

    int16_t Packet::readShort()
    {
        uint16_t i = 0;
        i |= readByte()<<8;
        i |= readByte()<<0;

        return (int16_t)i;
    }
        
    void Packet::setPayload(std::string payload_)
    {
        pos = 0;
        payload = payload_;
    }
            
    std::string Packet::toRaw()
    {
        std::string raw;

        raw += (char)0xFF;
        raw += (char)0xAA;
        raw += (char)type;
        raw += (char)(payload.size());
        raw += payload;
        raw += (char)checksum;

        return raw;
    }

    int Packet::getSize()
    {
        return payload.size();
    }
}
