 package RoboSegwaz;
 
import lejos.nxt.LCD;
import lejos.nxt.Button;
import lejos.nxt.comm.LCPBTResponder;


public class Segway {

	public static void main(String[] args) {
		LCD.drawString("Segway programm: Press Any Button for start", 0, 0);
		Button.waitForAnyPress();
		Balance b=new Balance();
		b.balance();
		LCPBTResponder lcpThread = new LCPBTResponder();
		lcpThread.setDaemon(true);
		lcpThread.start();
		
	}

}
