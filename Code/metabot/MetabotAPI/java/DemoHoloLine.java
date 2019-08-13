/* Lauching test : java DemoHoloLine.java
 * type ctrl-C to stop the process
 */
class DemoHoloLine {
    public static void main(String[] args)
    {
	HoloBinding holo = new HoloBinding("/dev/tty.holo-DevB", 115200);
	holo.reset_yaw(); /* reset the yaw to 0.0 deg */
	holo.beep(880, 200);

	while (holo.get_opt(2) < 0.30) {
	    holo.move_toward(50,0);
	    sleep(0.050);
	}

	holo.move_toward(-30,0);
	sleep(0.10);
	
	while (holo.get_opt(2) < 0.30) { 
	    holo.move_toward(-10,0);
	    sleep(0.050);
	}
	
	holo.stop_all();
	sleep(0.10);
	
	holo.beep(440, 200);
	holo.close();
    }

    public static void sleep(double sec) {
	try { Thread.sleep((int) (sec*1000)); } catch (Exception e) {}
    }

}
