/* Lauching test : java DemoHoloMotion.java
 * type ctrl-C to stop the process
 */
class DemoHoloMotion {
    public static void main(String[] args)
    {
	HoloBinding holo = new HoloBinding("/dev/tty.holo-DevB", 115200);
	holo.reset_yaw(); /* reset the yaw to 0.0 deg */
	holo.beep(880, 200);

	// 4 directions
	float [] angles = { 0.0f, 90.0f, 180.0f, -90.0f };
	for (int i=0; i<angles.length; i++) { 
	    holo.move_toward(40, angles[i]);
	    sleep(1.0);
	    holo.stop_all();
	    sleep(0.25);
	    holo.move_toward(-40, angles[i]);
	    sleep(1.0);
	    holo.stop_all();
	    sleep(0.25);
	}

	holo.beep(440, 200);

	// Rotation
        holo.control(0,0,40);
	sleep(1);
	holo.stop_all();
	sleep(0.250);
	holo.control(0,0,-40);
	sleep(1);
	
	// Cercle
	holo.stop_all();
	double t = 0.0;
	double dt = 0.050;
	while (t < 4.0) {
	    float x = (float) Math.sin(2*Math.PI * (t/4) - Math.PI/2);
	    float y = (float) Math.sin(2*Math.PI * (t/4));
	    holo.control(50*x,50*y,0);
	    t += dt;    
	    sleep(dt); // 20Hz
	}
	holo.stop_all();
	sleep(1.0);

	holo.close();
    }

    public static void sleep(double sec) {
	try { Thread.sleep((int) (sec*1000)); } catch (Exception e) {}
    }

}
