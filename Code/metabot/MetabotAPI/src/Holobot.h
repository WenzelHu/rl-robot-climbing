#pragma once
#include <vector>
#include "Robot.h"

#define METABOT_MONITOR 5
#define HOLOBOT_COMMAND 80

#define OPTICS_NB 7

namespace Metabot
{
    class Holobot : public Robot
    {
            bool output_state;
	    short sent_dx, sent_dy, sent_turn;
	    float yaw0;
        public:
	    float current_time; 
	    float distance;
	    float optics[OPTICS_NB];
	    float wheel_speeds[3];
	    float gyro_yaw;
	    float acc_x, acc_y, acc_z;

            Holobot(std::string port, int baud=1000000);
	    ~Holobot();
            void receive(Packet &packet);
            Packet command(uint8_t instruction);
	    void waitUpdate();
	    
	    /* the time in seconds from the starting of the robot system */
	    float get_time();
	    /* distance sensor (cm) */
	    float get_dist(); 
	    /* optical sensors, in [0,1] */
	    float get_opt(int i); /* sensors are identified from 0 to 4, from right to left, viewed from up */
            std::vector<float> get_opts();
	    /* yaw, relative to the start in degree, in trigonometric way from the upper view */
	    float get_yaw(); /* the yaw computed from the gyro, more precise than the magnetic yaw, but drift (several degree/mn) */
	    /* set the current yaw to be the yaw 0.0 deg (as yaw is computed from the gyro, it is relative */
	    void reset_yaw();		
	    /* the speeds of the wheels (deg/s). The wheels ids are as follows:
	     * 0: the wheel at the left (from up view) of the usb plug
	     * 1: the wheel at the right of the usb plug
	     * 2: the wheel at the opposite of the usb plug 
	     * the way (positive value) make the robot turn in the trigonometric way (from up view) */
	    float get_wheel_speeds(int id);

	    /* TODO: unités ? */
	    void control(float dx, float dy, float turn);
	    /* speed: mm/s ; direction : deg, anti trigo from the direction of optic sensors */
	    void move_toward(float speed, float direction);
	    /* rot_speed : deg / sec */
	    void turn(float rot_speed);
	    void stop_all();

            /* Trigger opticals calibration */
            void calibrate_opticals(bool black);
	    /* Calibrate the magneto by rotating the robot */
	    void calibrate_magneto();
	    void set_board_led(uint8_t state);
	    void beep(short freq, short duration);
	    void play(short id);
	    void print_state();
	    void debug_state(uint8_t on_or_off);

            /* Leds control */
            void set_leds(uint8_t red, uint8_t blue, uint8_t green);
            void leds_breath();
    };
}
