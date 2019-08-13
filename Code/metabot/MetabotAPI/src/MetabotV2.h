#pragma once

#include "Robot.h"

namespace Metabot
{
    class MetabotV2 : public Robot
    {
        public:
            enum {
                METABOT_MONITOR=5,
                METABOT_COMMAND=50
            };

            enum {
                START=0,
                STOP=1,
                ROTATE_CALIBRATION=2,
                SET_CONTROL=3,
                CONTROL=4,
                BEEP=5,
                LEDS=6,
                RESET=7,
                BHV=8,
                MOTOR=9
            };

            enum {
                LED_B=1,
                LED_G=2,
                LED_R=4
            };

            MetabotV2(std::string port, int baud=1000000);

            void receive(Packet &packet);
            Packet command(uint8_t instruction);

            /**
             * Wait for the robot to receive an update
             */
            void waitUpdate();

            /**
             * Enable the motors (this take several seconds since the torque is
             * slowly increased in the robot)
             */
            void start();

            /**
             * Disables the motors
             */
            void stop();

            /**
             * Reset (stops all motion, buzzer, restore leds etc.)
             */
            void reset();

            /**
             * Controls the robot speed (in mm/s and °/s)
             */
            void control(float x, float y, float turn);

            /**
             * Beeps (freq is Hz, duration is ms)
             */
            void beep(uint16_t freq, uint16_t duration);

            /**
             * LEDs
             */
            void setLeds(uint8_t value);

            /**
             * LEDs (by color name)
             */
            void setLeds(std::string color);

            /**
             * Enable given behavior
             */
            void bhv(uint8_t b);

            /**
            * Set motors position done by Siqi Hu
            */
            //void setMotors(float motors_target[12]);
            void setMotors(float l1, float l2, float l3, float l4, float l5, float l6, float l7, float l8, float l9, float l10, float l11, float l12);
            // Motors position
            std::vector<float> motors;

            // Imu
            float yaw, pitch, roll;

            // Leds
            std::vector<int> leds;
            
            // Distance sensor
            float distance;

            // Voltage
            float voltage;
    };
}
