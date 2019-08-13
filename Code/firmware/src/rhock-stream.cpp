#include <stdio.h>
#include <rhock/event.h>
#include <rhock/stream.h>
#include "rhock-stream.h"
#include "leds.h"
#include "mapping.h"
#include "motion.h"
#include "buzzer.h"
#include "motors.h"
#include "imu.h"
#include "leds.h"
#include "behavior.h"

#define RHOCK_STREAM_METABOT    50

short rhock_controls[16] = {0};

char rhock_on_packet(uint8_t type)
{
    if (type == RHOCK_STREAM_METABOT) {
        if (rhock_stream_available() >= 1) {
            uint8_t command = rhock_stream_read();
            switch (command) {
                case 0: // Starting the robot
                    motors_enable();
                    return 1;
                    break;
                case 1: // Stop
                    motors_disable();
                    return 1;
                    break;
                case 2: // Rotate calibration
#ifndef __EMSCRIPTEN__
                    imu_calib_rotate();
#endif
                    return 1;
                    break;
                case 3: // Set control
                    if (rhock_stream_available() == 3) {
                        uint8_t control = rhock_stream_read();
                        if (control < RHOCK_CONTROLS) {
                            uint16_t value = rhock_stream_read_short();
                            rhock_controls[control] = (short)value;
                        }
                    }
                    return 1;
                    break;
                case 4: // Speed
                    if (rhock_stream_available() == 6) {
                        motion_set_x_speed(((int16_t)rhock_stream_read_short())/10.0);
                        motion_set_y_speed(((int16_t)rhock_stream_read_short())/10.0);
                        motion_set_turn_speed(((int16_t)rhock_stream_read_short())/10.0);
                    }
                    return 1;
                    break;
                case 5: // Beep
                    if (rhock_stream_available() == 4) {
                        uint16_t freq = rhock_stream_read_short();
                        uint16_t duration = rhock_stream_read_short();
                        buzzer_beep(freq, duration);
                    }
                    return 1;
                    break;
                case 6: // Leds
                    if (rhock_stream_available() == 1) {
                        led_set_all(rhock_stream_read(), true);
                    }
                    return 1;
                    break;
                case 7: // Reset
                    buzzer_stop();
                    leds_decustom();
                    motion_set_x_speed(0.0);
                    motion_set_y_speed(0.0);
                    motion_set_turn_speed(0.0);
                    motion_reset();
                    return 1;
                    break;
                case 8: // Behavior
                    if (rhock_stream_available() == 1) {
                        behavior_set(rhock_stream_read());
                    }
                    break;
                case 9:
                    if (rhock_stream_available() == 48){

                        float l[12];
                        for(int i =0; i<12; i++)
                        {
                            l[i] = ((int32_t)rhock_stream_read_int())/1000.0;
                        }

                        l1[0] = l[0];
                        l1[1] = l[1];
                        l1[2] = l[2];
                        l1[3] = l[3];

                        l2[0] = l[4];
                        l2[1] = l[5];
                        l2[2] = l[6];
                        l2[3] = l[7];

                        l3[0] = l[8];
                        l3[1] = l[9];
                        l3[2] = l[10];
                        l3[3] = l[11];


                    }
                    
                    /*
                    if (rhock_stream_available() == 48){
                        float l[12];
                        for(int i =0; i<12;i++)
                        {
                            l[i] = ((int32_t)rhock_stream_read_int())/1000.0;
                        }
                        dxl_set_position(mapping[0], l[0]);
                        dxl_set_position(mapping[3], l[1]);
                        dxl_set_position(mapping[6], l[2]);
                        dxl_set_position(mapping[9], l[3]);

                        dxl_set_position(mapping[1], l[4]);
                        dxl_set_position(mapping[4], l[5]);
                        dxl_set_position(mapping[7], l[6]);
                        dxl_set_position(mapping[10], l[7]);

                        dxl_set_position(mapping[2], l[8]);
                        dxl_set_position(mapping[5], l[9]);
                        dxl_set_position(mapping[8], l[10]);
                        dxl_set_position(mapping[11], l[11]);

                    }
                    */
            }
        }
    }
    return 0;
}
