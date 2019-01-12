/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeExecuter;
import frc.robot.Autonomous.Modes.BasicMode;
import frc.robot.Autonomous.Modes.NewMode;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Loops.RobotStateEstimator;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

import java.util.ArrayList;

public class Robot extends CustomRobot {
	private Controllers robotControllers;

	private Looper mLooper;


	private DriveBaseSubsystem driveBaseSubsystem;
	private RobotStateEstimator robotStateEstimator;

	private ThreadRateControl threadRateControl = new ThreadRateControl();

	private AutoModeExecuter autoModeExecuter;

	private CustomJoystick driveJoystickThrottle;


	@Override
	public void robotInit() {
		robotControllers = Controllers.getInstance();
		mLooper = new Looper();

		driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		driveBaseSubsystem.init();
		driveBaseSubsystem.registerEnabledLoops(mLooper);
		robotStateEstimator = RobotStateEstimator.getInstance();
		mLooper.register(robotStateEstimator);

	}

	@Override
	public void autonomous() {
		mLooper.start(true);
		driveBaseSubsystem.setBrakeMode(true);
		autoModeExecuter = new AutoModeExecuter();


		AutoModeBase autoMode = new BasicMode();


		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;

		autoModeExecuter.start();

		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {threadRateControl.doRateControl(100);}
	}

	@Override
	public void operatorControl() {
		exitAuto();
		mLooper.start(false);

		while (isOperatorControl() && isEnabled()) {
			double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
			double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

			driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y - x, 1)));
			threadRateControl.doRateControl(20);

			System.out.println(driveBaseSubsystem.getLeftDistanceInches());
		}
	}

	@Override
	public void disabled() {
		exitAuto();

		mLooper.stop();

		threadRateControl.start(true);

		while (isDisabled()) {
			driveBaseSubsystem.setBrakeMode(false);
			threadRateControl.doRateControl(100);
		}
	}

	private void exitAuto() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			autoModeExecuter = null;
		} catch (Throwable t) {

		}
	}

	@Override
	public void test() {
	}
}
