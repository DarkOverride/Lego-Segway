package RoboSegwaz;

import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;

public class AngleFilter {
		private double angle = 0.0;
		private static double OFFSET = 270;
		private AccelHTSensor acc;
		private double accY;

		public double getAccY() {
			return accY;
		}

		// h is the sample-time
		public AngleFilter() {
			acc = new AccelHTSensor(SensorPort.S1);
		}

		public double compFilt(double angleV, double dt) {

			double angularVelocity = angleV;
			double  gyroAngle = (float) (angularVelocity * dt);
			
			double accX = acc.getXAccel();
			double accZ = acc.getZAccel();

			accY = Math.toDegrees((Math.atan2(accX, accZ) + Math.PI))- OFFSET;
			angle = (0.9 * (angle + (gyroAngle)) + 0.1 * accY);

			return angle;
		}
}
