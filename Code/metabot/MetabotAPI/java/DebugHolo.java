/* Lauching test : java DebugHolo
 * type ctrl-C to stop the process
 */

class DebugHolo {
    public static void main(String[] args)
    {
	HoloBinding hb = new HoloBinding("/dev/tty.holo-DevB", 115200);

	try {
	    while (true) {
		System.out.format("distance sensor : %f\n", hb.get_time());
		Thread.sleep(50);
	    }
	} catch (Exception e) {}

	
	//try { Thread.sleep(1000); } catch (Exception e) {}		
	hb.close();
    }
}
