#include <map>
#include <iostream>
#include "MetabotV2.h"

namespace Metabot
{
    MetabotV2::MetabotV2(std::string port, int baud)
        : Robot(port, baud), motors(12, 0), leds(12, 0)
    {
    }

    void MetabotV2::receive(Packet &packet)
    {
        if (packet.type == METABOT_MONITOR && packet.getSize() == 40) {
            for (int k=0; k<12; k++) {
                motors[k] = packet.readSmallFloat();
            }
            
            yaw = packet.readSmallFloat();
            pitch = packet.readSmallFloat();
            roll = packet.readSmallFloat();

            for (int k=0; k<12; k+=2) {
                uint8_t b = packet.readByte();
                leds[k] = (b>>4)&0xf;
                leds[k+1] = b&0xf;
            }

            distance = packet.readSmallFloat();
            voltage = packet.readSmallFloat();
            mutex.unlock();
        }
    }

    void MetabotV2::waitUpdate()
    {
        mutex.lock();
    }

    Packet MetabotV2::command(uint8_t instruction)
    {
        return Packet(METABOT_COMMAND).appendByte(instruction);
    }

    void MetabotV2::start()
    {
        Packet packet = command(START);
        send(packet);
    }
    
    void MetabotV2::stop()
    {
        Packet packet = command(STOP);
        send(packet);
    }

    void MetabotV2::reset()
    {
        Packet packet = command(RESET);
        send(packet);
    }

    void MetabotV2::control(float x, float y, float turn)
    {
        Packet packet = command(CONTROL);
        packet.appendSmallFloat(x);
        packet.appendSmallFloat(y);
        packet.appendSmallFloat(turn);
        send(packet);
    }

    void MetabotV2::beep(uint16_t freq, uint16_t duration)
    {
        Packet packet = command(BEEP);
        packet.appendShort(freq);
        packet.appendShort(duration);
        send(packet);
    }

    void MetabotV2::setLeds(uint8_t value)
    {
        Packet packet = command(LEDS);
        packet.appendByte(value);
        send(packet);
    }
            
    void MetabotV2::setLeds(std::string color)
    {
        std::map<std::string, uint8_t> mapping = {
            {"red", LED_R},
            {"blue", LED_B},
            {"green", LED_G},

            {"yellow", LED_R|LED_G},
            {"cyan", LED_G|LED_B},
            {"magenta", LED_R|LED_B},

            {"off", 0},
            {"white", LED_R|LED_G|LED_B},
        };

        if (mapping.count(color)) {
            setLeds(mapping[color]);
        } else {
            std::cerr << "Unknown color: " << color << std::endl;
        }
    }
            
    void MetabotV2::bhv(uint8_t b)
    {
        Packet packet = command(BHV);
        packet.appendByte(b);
        send(packet);
    }

    /*
    void MetabotV2::setMotors(float motors_target[12])
    {
        Packet packet = command(MOTOR);
        for(int i =0; i<12; i++)
            packet.appendFloat(motors_target[i]);
        send(packet);

    }
    */
    
    
    void MetabotV2::setMotors(float l1, float l2, float l3, float l4, float l5, float l6, float l7, float l8, float l9, float l10, float l11, float l12)
    {
        Packet packet = command(MOTOR);
        packet.appendFloat(l1);
        packet.appendFloat(l2);
        packet.appendFloat(l3);
        packet.appendFloat(l4);
        packet.appendFloat(l5);
        packet.appendFloat(l6);
        packet.appendFloat(l7);
        packet.appendFloat(l8);
        packet.appendFloat(l9);
        packet.appendFloat(l10);
        packet.appendFloat(l11);
        packet.appendFloat(l12);
        send(packet);

    }
    

}
