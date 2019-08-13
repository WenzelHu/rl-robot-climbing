/* Lauching test : java HoloBindingTest */

class HoloBindingTest {
    public static void main(String[] args)
    {
	HoloBinding hb = new HoloBinding("/dev/tty.holo-DevB", 115200);
	hb.beep(880, 200);
	// try { Thread.sleep(100); } catch (Exception e) {}
	hb.close();
    }
}
