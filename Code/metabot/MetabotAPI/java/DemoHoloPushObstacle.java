/* Lauching test : java DemoHoloPushObstacle.java
 * type ctrl-C to stop the process
 */
class DemoHoloPushObstacle {

    static HoloBinding holo;
    
    public static void main(String[] args)
    {
	holo = new HoloBinding("/dev/tty.holo-DevB", 115200);
	System.out.println("");
	holo.reset_yaw(); /* reset the yaw to 0.0 deg */
	holo.beep(880, 200);

	System.out.println("- exploration");
	float min_dist = -1;
	int min_idx = -1;
	float min_yaw = 0.0f;
	holo.turn(-20.0f);
	while (holo.get_yaw() < 90.0) {
	    sleep(0.050);
	    for (int i=0; i<3; i++) {
		float d = holo.get_dist(i);
		if (min_dist < 0 || d < min_dist) {
		    min_idx = i;
		    min_dist = d;
		    min_yaw = holo.get_yaw();
		}
	    }
	}

	System.out.format("- minimum distance : %1.2f detected on sensor %d at yaw %1.1f\n",
			  min_dist, min_idx, min_yaw);

	go_to_azimut(min_yaw + 30);
	holo.move_toward(-40, -min_idx * 120);
	sleep(1.0);
	
	holo.stop_all();
	holo.close();
    }

    public static void go_to_azimut(float az) {
	az = HoloBinding.normalize_yaw(az);
	System.out.format("- going to yaw : %1.1f ... ", az);
	holo.stop_all();
	while (holo.get_yaw() - az < 1) 
	    holo.turn(-30);
	while (holo.get_yaw() - az > -1) 
	    holo.turn(30);
	holo.stop_all();
	System.out.format("real yaw reached : %1.1f\n", holo.get_yaw());
    }
    
    public static void sleep(double sec) {
	try { Thread.sleep((int) (sec*1000)); } catch (Exception e) {}
    }

}
