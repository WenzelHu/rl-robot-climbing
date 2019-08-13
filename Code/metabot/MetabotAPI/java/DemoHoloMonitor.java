/* Lauching test : java DemoHoloMonitor
 * type ctrl-C to stop the process
 */

class DemoHoloMonitor {
    public static void main(String[] args)
    {
	HoloBinding holo = new HoloBinding("/dev/tty.holo-DevB", 115200);
	holo.monitor(true);
		
	try {
	    while (true)
		Thread.sleep(1000);
	} catch (Exception e) {}
    }
}
