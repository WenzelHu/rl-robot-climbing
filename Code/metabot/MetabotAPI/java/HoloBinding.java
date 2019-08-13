
class HoloBinding {

    public native boolean init(String port, int baudrate);
    public native boolean close();
    public native void monitor(boolean activate);

    /* the time in seconds from the starting of the robot system */
    public native float get_time();
    /* distance sensors (cm), the sensors id are as follows:
     * 0 : the distance sensor at the bottom of the usb plug
     * 1 : the distance sensor of the switch (at the right of the robot, viewed from up) 
     * 2 : the left one */
    public native float get_dist(int i); 
    
    /* optical sensors, in [0,1] */
    public native float get_opt(int i); /* sensors are identified from 0 to 4, from right to left, viewed from up */

    /* yaw, relative to the start in degree, in trigonometric way from the upper view */
    public native float get_yaw(); /* the yaw computed from the gyro, more precise than the magnetic yaw, but drift (several degree/mn) */
    /* set the current yaw to be the yaw 0.0 deg (as yaw is computed from the gyro, it is relative */
    public native void reset_yaw();		

    /* the speeds of the wheels (deg/s). The wheels ids are as follows:
     * 0: the wheel at the left (from up view) of the usb plug
     * 1: the wheel at the right of the usb plug
     * 2: the wheel at the opposite of the usb plug 
     * the way (positive value) make the robot turn in the trigonometric way (from up view) */
    public native float get_wheel_speeds(int id);

    /* play a beep (freq : frequency of the sound, duration : duration of the sound (ms)) */
    public native void beep(int freq, int duration);


    /* TODO: unités */
    public native void control(float dx, float dy, float turn);
    /* speed: mm/s ; direction : deg, anti trigo from the direction of optic sensors */
    public native void move_toward(float speed, float direction);
    /* rot_speed : deg / sec */
    public native void turn(float rot_speed);
    public native void stop_all();

    /* print the global state of the robot */
    public native void print_state();
    /* TODO: doc */
    public native void set_board_led(int state);
    
    HoloBinding(String port, int baudrate) {
	this.init("/dev/tty.holo-DevB", 115200);
	Runtime.getRuntime().addShutdownHook(new Thread() {
		public void run() {
		    close();
		    System.out.println("Bye bye baby !");
		}
	    });
    }

    static float normalize_yaw(float yaw) {
	while (yaw < -180.0) yaw += 360;
	while (yaw > 180.0) yaw -= 360;
	return yaw;
    }
    
    static {
        System.loadLibrary("holojava");
    }

}

