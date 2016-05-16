import lejos.nxt.ColorSensor;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
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

	static int position; // Placeholder variable to represent position
	static boolean isInLine = false; //  Flag variable

	static UltrasonicSensor sonicDown, sonicUp;
	static ColorSensor color;
	static boolean gotIt, supressed;
	static DifferentialPilot pilot;
	static Navigator nav;
	static NXTRegulatedMotor leftMotor, rightMotor;
	// Using this points considering the left lower corner as 0,0
	static Waypoint pRLineB, pRLineE, pLLineB, pLLineE, pMyCenter, pECenter, pGoal;

	public static void main(String[] args) {
		
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
		pLLineE = new Waypoint(25, 37, 270);
		pLLineB = new Waypoint(25, 113, 270);
		
		pMyCenter = new Waypoint(65, 37, 90);
		
		
		
		pECenter = new Waypoint(65, 113, 90);
		pGoal = new Waypoint(65, 141, 90);
		

		pilot = new DifferentialPilot( 5.6f, 11.2f, leftMotor, rightMotor, false); 
		OdometryPoseProvider position = new OdometryPoseProvider(pilot);
		Pose p = new Pose(pMyCenter.x,pMyCenter.y, pMyCenter.angle());
		position.setPose(new Pose(pMyCenter.x, pMyCenter.y, 90 ));
		nav = new Navigator(pilot, position);
		
		
		pilot.setTravelSpeed(speed);
		pilot.setRotateSpeed(rotateSpeed);
		// 	pilot.setAcceleration(20);

		Button.waitForAnyPress();
		Delay.msDelay(1000);
		
		
		
		
		

		Behavior b1 = new GoToLine(); 
		Behavior b4 = new ScoreGoal();
		Behavior b3 = new DriveForward();
		Behavior b2 = new DetectBlock();

		Behavior[] behaviorList = { b1, b3, b2, b4 };
		Arbitrator arby = new Arbitrator(behaviorList);
		arby.start();
	} 
}


class ScoreGoal implements Behavior {
	private boolean suppressed = false;
	private DifferentialPilot pilot;
	private Navigator nav;

	public ScoreGoal() {
		pilot = DeathStar.pilot;
		nav = DeathStar.nav;
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
		nav.goTo(DeathStar.pECenter);
		nav.goTo(DeathStar.pGoal);
		nav.goTo(DeathStar.pECenter);
	}

	
}

class DetectBlock implements Behavior {
	private UltrasonicSensor sonar;
	private boolean gotIt, supressed;
	private DifferentialPilot pilot;
	private Navigator nav;

	public DetectBlock() {
		gotIt = false;
		pilot = DeathStar.pilot;
		nav = DeathStar.nav;
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
	private Navigator nav;

	public DriveForward() {
		gotIt = false;
		pilot = DeathStar.pilot;
		sonar = DeathStar.sonicDown;
		nav = DeathStar.nav;
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
		nav.goTo(destination);
		while (!supressed && nav.isMoving()) {
		//	Pose p = nav.getPoseProvider().getPose();
		//	System.out.println((int) p.getX()+"  ||  "+ (int )p.getY());
			Thread.yield();
		}
		nav.stop();
		if(!supressed) DeathStar.isInLine = false;
	}
}

class GoToLine implements Behavior {
	private DifferentialPilot pilot;
	private Navigator nav = DeathStar.nav;
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
		LCD.drawString("GoToLine = " + nav.getPoseProvider().getPose(),0,1);
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
		nav.goTo(wp);
		while(!supressed && nav.isMoving())
		{
			//Pose p = nav.getPoseProvider().getPose();
			//System.out.println((int) p.getX()+"  ||  "+ (int )p.getY());
			Thread.yield();
		}
		
		if(supressed) DeathStar.lastLine = DeathStar.lastLine == 'R'? 'L': 'R';	
		else DeathStar.isInLine = true;
	}
}