/* Lauching test : java DemoHoloState.java
 * type ctrl-C to stop the process
 */
class DemoHoloState {
    public static void main(String[] args)
    {
	HoloBinding holo = new HoloBinding("/dev/tty.holo-DevB", 115200);
	holo.reset_yaw(); /* reset the yaw to 0.0 deg */
	
	try {
	    while (true) {
		System.out.println("--------------------------------------------------------------------------------");
		System.out.println("Robot state : ");
		System.out.format("- on board time : %1.1f sec\n", holo.get_time());
		System.out.format("- distance sensor (cm) : %1.1f %1.1f %1.1f\n", holo.get_dist(0), holo.get_dist(1), holo.get_dist(2));
		System.out.format("- optical sensor (%%) : %d %d %d %d %d\n",
				  (int) (100 * holo.get_opt(0)),
				  (int) (100 * holo.get_opt(1)),
				  (int) (100 * holo.get_opt(2)),
				  (int) (100 * holo.get_opt(3)),
				  (int) (100 * holo.get_opt(4)));
		System.out.format("- yaw : %3.1f deg (caution: subject to drift)\n", holo.get_yaw());
		System.out.format("- wheels speed (deg/s) : %3.2f %3.2f %3.2f\n",
				  holo.get_wheel_speeds(0), holo.get_wheel_speeds(1), holo.get_wheel_speeds(2));

		Thread.sleep(50);
	    }
	} catch (Exception e) {}

	
	//try { Thread.sleep(1000); } catch (Exception e) {}		
	holo.close();
    }
}
