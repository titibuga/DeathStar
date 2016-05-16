import lejos.nxt.ColorSensor;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.CompassHTSensor;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Behavior;
import lejos.robotics.subsumption.Arbitrator;
import lejos.util.Delay;

public class DeathStar {
	// Constant Definitions
	static final double speed = 20.0; // cm per sec;
	static final double rotateSpeed = 50.0; // degrees per sec;
	static final int distanceUp = 22;
	static final int distanceDown = 6;

	static OdometryPoseProvider position;
	static Navigator navigator;
	static CompassHTSensor compass;
	static boolean isInLine = false; //  Flag variable

	static UltrasonicSensor sonicDown, sonicUp;
	static ColorSensor color;
	static boolean gotIt, supressed;
	static DifferentialPilot pilot;
	static NXTRegulatedMotor leftMotor, rightMotor;

	public static void main(String[] args) {
		Button.waitForAnyPress();
		leftMotor = Motor.B;
		rightMotor = Motor.A;
		sonicDown = new UltrasonicSensor(SensorPort.S3);
		color = new ColorSensor(SensorPort.S2);
		sonicUp = new UltrasonicSensor(SensorPort.S1);
		gotIt = false;

		pilot = new DifferentialPilot( 5.6f, 11.2f, leftMotor, rightMotor, false);
		position = new OdometryPoseProvider(pilot);
		navigator = new Navigator(pilot, position);
		compass = new CompassHTSensor(SensorPort.S4);
		compass.resetCartesianZero();
		
		pilot.setTravelSpeed(speed);
		pilot.setRotateSpeed(rotateSpeed);
//		 	pilot.setAcceleration(20);

		Delay.msDelay(1000);

		Behavior b1 = new GoToLine();
		Behavior b4 = new ScoreGoal();
		Behavior b3 = new DriveForward();
		Behavior b2 = new DetectBlock();
		Behavior b5 = new DeadReckoning();
		
		Behavior[] behaviorList = { b1, b3, b2, b4, b5 };
		Arbitrator arby = new Arbitrator(behaviorList);
		arby.start();
	} 
}


class ScoreGoal implements Behavior {
	private boolean suppressed = false;

	public ScoreGoal() {
		pilot = DeathStar.pilot;
	}

	public boolean takeControl() {
		return DeathStar.gotIt;  // this behavior always wants control.
	}

	public void suppress() {
		suppressed = true;// standard practice for suppress methods
	}

	public void action() {
		LCD.clear();
		LCD.drawString("ScoreGoal",0,1);
		suppressed = false;
		pilot.rotate(-90);
		pilot.travel(40);
		pilot.rotate(90);
		pilot.travel(10000);
	}

	private DifferentialPilot pilot;
}

class DetectBlock implements Behavior {
	private UltrasonicSensor sonar;
	private boolean gotIt, supressed;
	private DifferentialPilot pilot;

	public DetectBlock() {
		gotIt = false;
		pilot = DeathStar.pilot;
		sonar = DeathStar.sonicDown;
	}

	public boolean takeControl() {
		sonar.ping();
		return sonar.getDistance() <= DeathStar.distanceDown 
				&&!DeathStar.gotIt 
				&& DeathStar.isInLine;
	}

	public void suppress() {
		supressed = true;
	}

	public void action() {
		LCD.clear();
		LCD.drawString("DetectBlock",0,1);
		supressed = false;

		if(DeathStar.color.getColorID() != ColorSensor.Color.YELLOW) dodgeCube();
		else{
			DeathStar.gotIt = true;
			return;		
		}

		/*
	else {
	    pilot.forward(); //Faz gol
	}

    if(!DeathStar.gotIt && DeathStar.color.getColorID() == lejos.robotics.Color.RED){
    	DeathStar.gotIt = true;
    }
    else{
    	dodgeCube();
    }*/

	}

	private void dodgeCube() {
		pilot.travel(-(2*DeathStar.distanceDown));
		pilot.rotate(-90);
		pilot.travel(20);
		pilot.rotate(90);
		pilot.travel(45);
		pilot.rotate(90);
		pilot.travel(20);
		pilot.rotate(-90);
	}
}

class DriveForward implements Behavior {
	private UltrasonicSensor sonar;
	private boolean gotIt, supressed;
	private DifferentialPilot pilot;

	public DriveForward() {
		gotIt = false;
		pilot = DeathStar.pilot;
		sonar = DeathStar.sonicDown;
	}

	public boolean takeControl() {
		// sonar.ping();
		// return DeathStar.sonicDown.getDistance() <= DeathStar.distanceDown && !gotIt;
		return DeathStar.isInLine;
	}

	public void suppress() {
		supressed = true;
	}

	public void action() {
		LCD.clear();
		LCD.drawString("DriveForward",0,1);
		supressed = false;
		
		pilot.forward();
		while (!supressed) {
			Thread.yield();
		}
		pilot.stop();
	}
}

class GoToLine implements Behavior {
	private DifferentialPilot pilot;
	private UltrasonicSensor sonar;
	private boolean gotIt, supressed;

	public GoToLine() {
		sonar = new UltrasonicSensor(SensorPort.S1);
		pilot = DeathStar.pilot;
	}

	public boolean takeControl() {
		return true; 
	}

	public void suppress() {
		supressed = true;
	}

	public void action() {
		LCD.clear();
		LCD.drawString("GoToLine",0,1);
		supressed = false;
		int distCenterToLine = 40;
		
		// If "esta no centro" vira pra direita
		pilot.rotate(90); 
		pilot.travel(distCenterToLine);
		pilot.rotate(-90);

		DeathStar.isInLine = true;
	}
}

class DeadReckoning implements Behavior {
	private boolean supressed;
	private OdometryPoseProvider position;
	private Navigator navigator;
	private CompassHTSensor compass;
	private final float e = 15; // erro mínimo

	public DeadReckoning() {
		navigator = DeathStar.navigator;
		compass = DeathStar.compass;
		position = DeathStar.position;
	}

	public boolean takeControl() {
		return (Math.abs(position.getPose().getHeading() - correctAngle(compass.getDegreesCartesian())) >= e);
	}

	public void suppress() {
		supressed = true;
	}

	public void action() {
		System.out.println("DeadReckoning");
		System.out.println("Pose: " + position.getPose().getHeading());
		System.out.println("Compass: " + compass.getDegreesCartesian());
		position.setPose(new Pose(position.getPose().getX(), position.getPose().getY(), correctAngle(compass.getDegreesCartesian())));
		System.out.println("NewPose: " + position.getPose().getHeading());
//		Button.waitForAnyPress();
		supressed = false;
	}
	
	private float correctAngle(float angle) {
		if (angle > 180) {
			return angle - 360;
		}
		return angle;
	}
}