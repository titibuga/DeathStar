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
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.subsumption.Behavior;
import lejos.robotics.subsumption.Arbitrator;
import lejos.util.Delay;

public class DeathStar {
	// Constant Definitions
	static final double speed = 20.0; // cm per sec;
	static final double rotateSpeed = 50.0; // degrees per sec;
	static final int distanceUp = 22;
	static final int distanceDown = 6;

	static char lastLine;

	static OdometryPoseProvider position;
	static Navigator navigator;
	static CompassHTSensor compass;

	static boolean isInLine = false; //  Flag variable

	static UltrasonicSensor sonicDown, sonicUp;
	static ColorSensor color;
	static boolean gotIt, supressed;
	static DifferentialPilot pilot;

	static NXTRegulatedMotor leftMotor, rightMotor;
	// Using this points considering the left lower corner as 0,0
	static Waypoint pRLineB, pRLineE, pLLineB, pLLineE, pMyCenter, pECenter, pGoal;

	public static void main(String[] args) {
		Button.waitForAnyPress();
		leftMotor = Motor.B;
		rightMotor = Motor.A;
		sonicDown = new UltrasonicSensor(SensorPort.S3);
		color = new ColorSensor(SensorPort.S2);
		sonicUp = new UltrasonicSensor(SensorPort.S1);
		gotIt = false;
		
		// Points of the map
		lastLine = 'L';
		pRLineB = new Waypoint(105, 37, 90);
		
		pRLineE = new Waypoint(105, 113, 90);
		pLLineE = new Waypoint(25, 37, 90);
		pLLineB = new Waypoint(25, 113, 90);
		
		pMyCenter = new Waypoint(65, 37, 90);
		
		pECenter = new Waypoint(65, 113, 90);
		pGoal = new Waypoint(65, 141, 90);
		

		pilot = new DifferentialPilot( 5.6f, 11.2f, leftMotor, rightMotor, false);
		position = new OdometryPoseProvider(pilot);
		navigator = new Navigator(pilot, position);
		compass = new CompassHTSensor(SensorPort.S4);
		compass.resetCartesianZero();
		
		position.setPose(new Pose(pMyCenter.x, pMyCenter.y, 90 ));
		
		
		pilot.setTravelSpeed(speed);
		pilot.setRotateSpeed(rotateSpeed);
		// 	pilot.setAcceleration(20);

		
		Delay.msDelay(1000);
	        
		Behavior b1 = new GoToLine();
		Behavior b4 = new ScoreGoal();
		Behavior b3 = new DriveForward();
		Behavior b2 = new DetectBlock();
		Behavior b5 = new DeadReckoning();
		
//		Behavior[] behaviorList = { b1, b3, b2, b4 };
		Behavior[] behaviorList = { b1, b3, b2, b4, b5 };
		Arbitrator arby = new Arbitrator(behaviorList);
		arby.start();
	} 
}

class ScoreGoal implements Behavior {
	private boolean suppressed = false;
	private DifferentialPilot pilot;
	private Navigator navigator;

	public ScoreGoal() {
		pilot = DeathStar.pilot;
		navigator = DeathStar.navigator;
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
		navigator.goTo(DeathStar.pECenter);
		navigator.goTo(DeathStar.pGoal);
		navigator.goTo(DeathStar.pECenter);
	}	
}

class DetectBlock implements Behavior {
	private UltrasonicSensor sonar;
	private boolean gotIt, supressed;
	private DifferentialPilot pilot;
	private Navigator navigator;

	public DetectBlock() {
		gotIt = false;
		pilot = DeathStar.pilot;
		navigator = DeathStar.navigator;
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
		pilot.rotate(90);
		pilot.travel(20);
		pilot.rotate(-90);
		pilot.travel(45);
		pilot.rotate(-90);
		pilot.travel(20);
		pilot.rotate(90);
	}
}

class DriveForward implements Behavior {
	private UltrasonicSensor sonar;
	private boolean gotIt, supressed;
	private DifferentialPilot pilot;
	private Navigator navigator;

	public DriveForward() {
		gotIt = false;
		pilot = DeathStar.pilot;
		sonar = DeathStar.sonicDown;
		navigator = DeathStar.navigator;
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
		Waypoint destination;
		supressed = false;
		
		if(DeathStar.lastLine == 'R'){ destination = DeathStar.pRLineE;System.out.println("Going to Right End");}
		else {destination = DeathStar.pLLineE;System.out.println("Going to Left End");}		
		navigator.goTo(destination.x, destination.y);
		while (!supressed && navigator.isMoving()) {
		//	Pose p = nav.getPoseProvider().getPose();
		//	System.out.println((int) p.getX()+"  ||  "+ (int )p.getY());
			Thread.yield();
		}
		navigator.stop();
		if(!supressed) DeathStar.isInLine = false;
	}
}

class GoToLine implements Behavior {
	private DifferentialPilot pilot;
	private Navigator navigator = DeathStar.navigator;
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
		LCD.drawString("GoToLine = " + navigator.getPoseProvider().getPose(),0,1);
		supressed = false;
		int distCenterToLine = 40;
		Waypoint wp;
		
		if(DeathStar.lastLine == 'L')
		{
			DeathStar.lastLine = 'R';
			wp = DeathStar.pRLineB;
			System.out.println("Going to Right Point");
		}
		else
		{
			DeathStar.lastLine = 'L';
			wp = DeathStar.pLLineB;
			System.out.println("Going to Left Point");
		}
		
		// If "esta no centro" vira pra direita
		navigator.goTo(wp.x,wp.y);
		while(!supressed && navigator.isMoving())
		{
			//Pose p = nav.getPoseProvider().getPose();
			//System.out.println((int) p.getX()+"  ||  "+ (int )p.getY());
			Thread.yield();
		}
		
		if(supressed) DeathStar.lastLine = DeathStar.lastLine == 'R'? 'L': 'R';	
		else DeathStar.isInLine = true;
		//Button.waitForAnyPress();
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
		return (diffAngle(position.getPose().getHeading(), correctAngle(compass.getDegreesCartesian())) >= e);
	}

	public void suppress() {
		supressed = true;
	}

	public void action() {
//		float oldHeading = position.getPose().getHeading();
		System.out.println("DeadReckoning");
		System.out.println("Pose: " + position.getPose().getHeading());
		System.out.println("Compass: " + compass.getDegreesCartesian());
		float compassHeading = correctAngle(compass.getDegreesCartesian());
		Pose pose = new Pose(position.getPose().getX(), position.getPose().getY(), compassHeading);
		
		
//		DeathStar.pilot.rotate(anglesToRotate(oldHeading, compassHeading));
		position.setPose(pose);
		System.out.println("NewPose: " + position.getPose().getHeading());
//		System.out.println("Rotate:"+anglesToRotate(oldHeading, compassHeading));
//		Button.waitForAnyPress();
		supressed = false;
		
	}
	
	private float anglesToRotate(float oldH, float newH)
	{
		if(oldH < 0) oldH += 360;
		if(newH < 0) newH += 360;
		float diff = newH - oldH;
		if(diff > 180)
			return 180 - diff;
		else return diff;
		
	}
	
	private float correctAngle(float angle) {
		if (angle > 180) {
			return angle - 360;
		}
		return angle;
	}
	
	private float diffAngle(float a, float b) {
//		boolean flaga = false, flagb = false;
//		if (a < 0) {
//			a *= -1;
//			flaga = true;
//		}
//		if (b < 0) {
//			b *= -1;
//			flagb = true;
//		}
//		if (flaga == flagb) {
//			return Math.abs(a - b);
//		}
//		else {
//			return 360 - a - b;
//		}
		float diff = a > b ? (a - b) : (b - a);
		return (diff > 180) ? 360 - diff: diff;
	}
}