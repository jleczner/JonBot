package leczner.jon.JonBot;

import robocode.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.Random;

import static robocode.util.Utils.normalRelativeAngleDegrees;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * JonBot - a robot by (your name here)
 */
public class JonBot extends TeamRobot
{
	private ArrayList<String> friendlies = new ArrayList<>();
	private ArrayList enemies = new ArrayList<>();
	private byte scanDirection = 1;
	String targetName;
	boolean movingForward = true;

	/**
	 * run: JonBot's default behavior
	 */
	public void run() {
		friendlies.add("Champion");
		friendlies.add("RocketDoll");
		friendlies.add("StaticTinTin");
		friendlies.add("Jelly");
		friendlies.add("McKiller");

		setAllColors(Color.BLACK);
		setBulletColor(Color.BLUE);
		setAdjustRadarForGunTurn(true);
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForRobotTurn(true);

		Random r = new Random();
		int randomDirection;
		int counter = 0;

		System.out.println("init complete");
		// Robot main loop
		while(true) {
			setTurnRadarRight(360);
			randomDirection = r.nextInt(180);
			if (counter % 5 == 0)
				setTurnRight(getHeading() - randomDirection);
			if (counter % 6 == 0)
				setTurnLeft(getHeading() - randomDirection);
			waitFor(new TurnCompleteCondition(this));
			setAhead(1000);
			execute();
			counter++;
		}
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		if (targetName != null && !e.getName().equals(targetName)) {
			return;
		}

		if (targetName == null) {
			targetName = e.getName();
			System.out.println("Tracking " + targetName);
		}

		double absoluteBearing = getHeading() + e.getBearing();
		double bearingFromGun = normalRelativeAngleDegrees(absoluteBearing - getGunHeading());

		// If it's close enough, fire!
		if (Math.abs(bearingFromGun) <= 3) {
			turnGunRight(bearingFromGun);
			// We check gun heat here, because calling fire()
			// uses a turn, which could cause us to lose track
			// of the other robot.
			if (getGunHeat() == 0) {
				fire(Math.min(3 - Math.abs(bearingFromGun), getEnergy() - .1));
			}
		} // otherwise just set the gun to turn.
		// Note:  This will have no effect until we call scan()
		else {
			turnGunRight(bearingFromGun);
		}
		// Generates another scan event if we see a robot.
		// We only need to call this if the gun (and therefore radar)
		// are not turning.  Otherwise, scan is called automatically.
		if (bearingFromGun == 0) {
			scan();
		}

//		//** based on http://mark.random-article.com/robocode/basic_scanning.html
//		setTurnRadarRight(getHeading() - getRadarHeading() + e.getBearing());
//		scanDirection *= -1; // changes value from 1 to -1
//		setTurnRadarRight(360 * scanDirection);
////		execute();
//		// ********
//		Robot target = null;
//		if (isTeammate(e.getName())) {
//			setEventPriority("friendly", 0);
//		} else {
//			//** based on https://sourceforge.net/p/robocode/discussion/116459/thread/df3a124f/
//			enemies.add(e);
//			System.out.println("enemy found: " + e.getName());
//			double weakestEnergy = e.getEnergy();
//			for (int i = 0; i < enemies.size(); i++) {
//				Robot current = (Robot)enemies.get(i);
//				double otherEnemyEnergy = current.getEnergy();
//				if (otherEnemyEnergy < weakestEnergy) {
//					weakestEnergy = otherEnemyEnergy;
//					target = current;
//				}
//			}
//			if (target.equals(e)) {
//				System.out.println("target acquired");
//				double absoluteBearing = getHeading() + e.getBearing();
//				setTurnGunRight(robocode.util.Utils.normalRelativeAngle(absoluteBearing - getGunHeading()));
//				if (getGunHeat() == 0 && Math.abs(getGunTurnRemaining()) < 10)
//					setFire(Math.min(400 / e.getDistance(), 3));
//			}
//		}
//		//********
//		scan();
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// Replace the next line with any behavior you would like
		setTurnLeft(90);
		setTurnGunRight(e.getBearing());
		fire(2);
		setAhead(30);
		scan();
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
//		setTurnRight(e.getBearing());
//		setTurnGunLeft(e.getBearing());
//		setBack(1000);
		if (movingForward) {
			setBack(40000);
			movingForward = false;
		} else {
			setAhead(40000);
			movingForward = true;
		}
		execute();
	}	
}
