package RoboSegwaz;

import lejos.nxt.SensorPort;
import lejos.nxt.addon.AccelHTSensor;

public class AngleFilter {
		private AccelHTSensor acc;
		private double angleY;
		private double angle;

		public double getAngleY() {
			return angleY;
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
			
			angleY=Math.toDegrees(Math.atan2(accX, accZ))-90;

			angle = (0.9 * (gyroAngle) + 0.1 * angleY);

			return angle;
		}
}
