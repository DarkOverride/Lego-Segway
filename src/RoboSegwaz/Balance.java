package RoboSegwaz;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.EncoderMotor;
import lejos.nxt.addon.GyroSensor;
import lejos.nxt.LCD;
import lejos.util.*;
import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;

public class Balance {
	
	static final int Tp = 20;
	static final double MOVING_AVG_PERCENTAGE_GYRO=0.9;
	static final int max=1000;
	static final int min=-max;
	private PIDcontroller angleV;
	
	double offset;
	double angle=0;

	EncoderMotor mr; 
	EncoderMotor ml;
	GyroSensor gyro;

	public Balance(){
		this.gyro=new GyroSensor(SensorPort.S2);
		this.mr =new NXTMotor(MotorPort.B); 
		this.ml =new NXTMotor(MotorPort.C);
		this.angleV=new PIDcontroller();
		setOffset();
	}

	public void setOffset(){
		double os=0;
		LCD.clear();
		System.out.print("calibrating, please lay down the robot");

		//calibrating the gyro
		this.gyro.recalibrateOffset();
		for(int i=0;i<=100;i++){
			setMotors(0);
			os=os+gyro.getAngularVelocity();
		}
		this.offset=(os/100+1);
		LCD.drawString("the robot is calibratet", 0, 1);
		LCD.drawInt((int)this.offset, 0, 2);
		Button.waitForAnyPress();
	}
	//set the Motor power to both Motors same
	public void setMotors(double power) {
		setMotors(power, power);
	}
	//set the Motor power different for each
	public void setMotors(double r, double l) {
		mr.setPower(calcPower(r));
		if (r < 0) {
			mr.backward();
		} else if (r > 0) {
			mr.forward();
		}

		ml.setPower(calcPower(l));
		if (l < 0) {
			ml.backward();
		} else if (l > 0) {
			ml.forward();
		}
	}
	//calculate the Power
	public int calcPower(double out){
		
		return (int)Math.abs(out);//(55 + (Math.abs(out) * 45) / 100); //
	}

	//balancing the Robot with a PID controller
	public void balance(){

		//logging the gyro angle velocity an sending it to the Pc
		Logging.logging();

		double dt=25;
		double lastTime=0;
		boolean setLastTime=true;
		double now=0;
		double gyroV=0;
		double gyroA=0;
		double out=0;

		LCD.clear();
		LCD.drawString("Starting balancing in 5sec, please put up the robot", 0, 0);
		for (int i=4;i>0;i--){
			Delay.msDelay(1000);
			LCD.drawString(i+"sec", 0, 5-i);
		}
		Delay.msDelay(1000);
		Sound.beep();
		LCD.drawString("Starting", 0, 5);


		lastTime=System.currentTimeMillis();
		while(Button.ESCAPE.isUp()){
			LCD.clear();
			now=System.currentTimeMillis();

			gyroV = gyroV * MOVING_AVG_PERCENTAGE_GYRO +
				  (gyro.getAngularVelocity() - offset) * (1 - MOVING_AVG_PERCENTAGE_GYRO);//////////////////////////////?
			
			//When the Segway falls down ->break
			if(Math.abs(gyroV)>90){
				System.out.println("Segway is fallen down! Pls Restart");
				Delay.msDelay(2000);
				break;
			}
			//Time setting
			if(setLastTime){
				dt=now-lastTime;
				setLastTime=false;
			}

			//gyroA = gyroA + (gyroV * ((double) dt / 1000));
			out =angleV.calcOutput(gyroV, offset, dt);

			if(out>max){
				out=max;
			}
			if(out<min){
				out=min;
			}
			setMotors(out);



			Logging.logger.writeLog(gyroV);
			Logging.logger.writeLog(angleV.getKi()*angleV.getIntegral());
			Logging.logger.writeLog(angleV.getKd()*angleV.getDerivative());
			Logging.logger.writeLog(out);
			Logging.logger.finishLine();
			//}

			
			lastTime=now;
			Delay.msDelay(Tp); //Waiting Tp time
		} //! done with loop, go back and do it again.
		setMotors(0);
		Logging.logger.stopLogging();
	}



}
