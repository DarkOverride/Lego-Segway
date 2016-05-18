package RoboSegwaz;

import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.util.LogColumn;
import lejos.util.NXTDataLogger;

public class Logging {
	static NXTDataLogger logger = new NXTDataLogger();
	static LogColumn gyroValue = new LogColumn("angular Vilocity", LogColumn.DT_DOUBLE);
	static LogColumn ki= new LogColumn("ki", LogColumn.DT_DOUBLE,2);
	static LogColumn kd= new LogColumn("kd", LogColumn.DT_DOUBLE,3);
	static LogColumn out= new LogColumn("out", LogColumn.DT_DOUBLE,4);
	static LogColumn[] columnDefs = new LogColumn[] { gyroValue, ki, kd, out}; 

	public static void logging() {
		LCD.clear();
		LCD.drawString("Waiting for ", 0, 2);
		LCD.drawString("bluetooth con to", 0, 3);
		LCD.drawString("PC to log data.", 0, 4);
		LCD.drawString("Launch NXT Chart", 0, 5);
		LCD.drawString("Logger & click", 0, 6);
		LCD.drawString("the Connect btn.", 0, 7);

		// From a command line:
		// [leJOS Install Location]\bin\nxjchartinglogger.bat
		// Enter the NXT's name or address.  Be sure you have write permission
		// to the folder chosen by the utility.  If not, change the folder.
		// Click the Connect button

		NXTConnection connection = Bluetooth.waitForConnection();
		try {
			logger.startRealtimeLog(connection);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		logger.setColumns(columnDefs);  // must be after startRealtimeLog()

		LCD.clear();
		Sound.beep();
		LCD.drawString("Press orange btn", 0, 4);
		LCD.drawString("to start.", 0, 5);
		Button.ENTER.waitForPressAndRelease();
		LCD.clear();

		LCD.drawString("Press and hold", 0, 5);
		LCD.drawString("dark gray ESCAPE", 0, 6);
		LCD.drawString("button to stop.", 0, 7);

	}

}
