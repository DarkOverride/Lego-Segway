package RoboSegwaz;

public class PIDcontroller {
	private double Kp;	                          
	private double Ki;	                            
	private double Kd;
	private double error=0;
	private double lastError=0;
	private double integral=0;
	private double derivative=0;
	private final int intMax=5000;
	private final int intMin=-intMax;
	

	public PIDcontroller(){
		this.Kp = 1;	//340                             
		this.Ki = 0.01;	//50                              
		this.Kd = 0.1;	//300 
		
	}
	public double calcOutput(double value,double offset, double dt){
		error=value-offset;//////////////////////////////////???????????????????????
		integral = integral + error*dt;        
		derivative = (error - lastError)/dt;
		integralTest();
		
		double out =Kp*error + Ki*integral + Kd*derivative;
		lastError = error ;
		return out;
		
	}

	public void integralTest(){
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
	}
	public double getKp() {
		return Kp;
	}
	public void setKp(double kp) {
		Kp = kp;
	}
	public double getKi() {
		return Ki;
	}
	public void setKi(double ki) {
		Ki = ki;
	}
	public double getKd() {
		return Kd;
	}
	public void setKd(double kd) {
		Kd = kd;
	}
	public double getError() {
		return error;
	}
	public void setError(double error) {
		this.error = error;
	}
	public double getLastError() {
		return lastError;
	}
	public void setLastError(double lastError) {
		this.lastError = lastError;
	}
	public double getIntegral() {
		return integral;
	}
	public void setIntegral(double integral) {
		this.integral = integral;
	}
	public double getDerivative() {
		return derivative;
	}
	public void setDerivative(double derivative) {
		this.derivative = derivative;
	}
}
