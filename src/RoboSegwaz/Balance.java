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
	static final double Kp = 10;	//340                             
	static final double Ki = 0.5;	//50                              
	static final double Kd = 1;	//300 
	static final int Tp = 20;
	static final double MOVING_AVG_PERCENTAGE_GYRO=0.9;
	static final int max=1000;
	static final int min=-max;
	static final int intMax=5000;
	static final int intMin=-intMax;
	double offset;
	double angle=0;

	EncoderMotor mr; 
	EncoderMotor ml;
	GyroSensor gyro;

	public Balance(){
		this.gyro=new GyroSensor(SensorPort.S2);
		this.mr =new NXTMotor(MotorPort.B); 
		this.ml =new NXTMotor(MotorPort.C);
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
		/*setMotors(mrls + 18*Math.sin(System.currentTimeMillis()/(double)50),
				mrls + 18*Math.sin(System.currentTimeMillis()/(double)50 + Math.PI));*/
		setMotors(power, power);
	}
	//set the Motor power different for each
	public void setMotors(double r, double l) {
		mr.setPower((int) Math.abs(r));
		if (r < 0) {
			mr.backward();
		} else if (r > 0) {
			mr.forward();
		}

		ml.setPower((int) Math.abs(l));
		if (l < 0) {
			ml.backward();
		} else if (l > 0) {
			ml.forward();
		}
	}

	//balancing the Robot with a PID controller
	public void balance(){

		//logging the gyro angle velocity an sending it to the Pc
		Logging.logging();

		double lastError = 0; 

		double integral = 0;
		double derivative = 0;  // ! the place where we will store the derivative
		double dt=25;
		double lastTime=0;
		boolean setLastTime=true;
		double now=0;
		double gyroV=0;
		double gyroA=0;
		double out=0;
		double error =  0;

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
			error=gyroV-offset;//////////////////////////////////???????????????????????
			integral = integral + error*dt;        
			derivative = (error - lastError)/dt;  

			//f(error<2 && error>-2){
			//	error=0;
			//}
			//if(error!=0){
			if(integral>intMax){
				integral=intMax;
			}
			if(integral<intMin){
				integral=intMin;
			}
			//reset Integral
			if(Math.signum(error)!=Math.signum(lastError))
			{
				integral=0;
			}
			out =Kp*error + Ki*integral + Kd*derivative;

			if(out>max){
				out=max;
			}
			if(out<min){
				out=min;
			}
			setMotors(out);



			Logging.logger.writeLog(gyroV);
			Logging.logger.writeLog((double)Ki*integral);
			Logging.logger.writeLog((double)Kd*derivative);
			Logging.logger.writeLog(out);
			Logging.logger.finishLine();
			//}

			lastError = error ;
			lastTime=now;
			Delay.msDelay(Tp); //Waiting Tp time
		} //! done with loop, go back and do it again.
		setMotors(0);
		Logging.logger.stopLogging();
	}



}
