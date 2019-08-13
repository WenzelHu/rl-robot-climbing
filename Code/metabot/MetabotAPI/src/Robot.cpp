#include <iostream>
#include "Robot.h"
#include <unistd.h>
#include <stdio.h>

namespace Metabot
{
    Robot::Robot(std::string name, int baud)
        : port(name, baud, serial::Timeout::simpleTimeout(1000)), over(false)
    {
        mutex.lock();
        thread = new std::thread([this]() {
            this->process();
        });
        mutex.lock();
    }

    Robot::~Robot()
    {
        if (thread) {
            over = true;
            thread->join();
            delete thread;
            thread = NULL;
        }
	port.close();
    }

    void Robot::send(Packet &packet)
    {
        std::string raw = packet.toRaw();
        port.write(raw);
    }

    void Robot::receive(Packet &packet)
    {
    }

    void Robot::monitor(int frequency)
    {
        // Enabling monitoring
        send(Packet(Packet::MONITOR).appendInt(frequency));
    }

    void Robot::rhock_mode() {
      printf("- requesting rhock mode\n");
      port.write("rhock\nrhock\nrhock\n");    
    }

    void Robot::process()
    {
        // Ensuring the robot is in rhock mode
        rhock_mode();
	
        // Reseting monitor
        monitor(0);
        mutex.unlock();

	printf("- listening to robot\n");
        int state = 0;
        int type = 0;
        int size = 0;
        int pos = 0;
        uint8_t checksum = 0;
        std::string payload;
        while (!over) {
            if (port.waitReadable(0.1)) {
                size_t n = port.available();
                uint8_t *data = new uint8_t[n];
                port.read(data, n);

                for (size_t k=0; k<n; k++) {
                    uint8_t c = data[k];
                    switch (state) {
                        case 0:
                            if (c == 0xFF) {
                                state++;
                            }
                            break;
                        case 1:
                            if (c == 0xAA) {
                                state++;
                            } else {
                                state = 0;
                            }
                            break;
                        case 2:
                            type = c;
                            state++;
                            break;
                        case 3:
                            size = c;
                            state++;
                            payload = "";
                            checksum = 0;
                            pos = 0;
                            if (!size) {
                                state ++;
                            }
                            break;
                        case 4:
                            pos++;
                            payload += (char)c;
                            checksum += c;
                            if (pos >= size) {
                                state ++;
                            }
                            break;
                        case 5:
                            if (c == checksum) {
                                Packet packet(type);
                                packet.setPayload(payload);
                                receive(packet);
                            }
                            state = 0;
                            break;
                    }
                }

                delete[] data;
            }
        }
    }
}
