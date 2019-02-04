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
import frc.robot.Subsystems.HatchSubsystem;
import frc.robot.Subsystems.CargoSubsystem;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Loops.RobotStateEstimator;
//import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

//import java.util.ArrayList;

public class Robot extends CustomRobot {
	private Looper mLooper;
	private OI oI;

	private DriveBaseSubsystem driveBaseSubsystem;
	private HatchSubsystem hatchSubsystem;
	private CargoSubsystem cargoSubsystem;

	private RobotStateEstimator robotStateEstimator;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private AutoModeExecuter autoModeExecuter;

	@Override
	public void robotInit() {
		mLooper = new Looper();
		oI = OI.getInstance();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		driveBaseSubsystem.init();
		driveBaseSubsystem.registerEnabledLoops(mLooper);

		hatchSubsystem = HatchSubsystem.getInstance();
		hatchSubsystem.init();
		hatchSubsystem.registerEnabledLoops(mLooper);

		cargoSubsystem = CargoSubsystem.getInstance();
		cargoSubsystem.init();
		cargoSubsystem.registerEnabledLoops(mLooper);

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
		threadRateControl.start(true);

		Controllers.getInstance().getCompressor().start();
		Controllers.getInstance().getCompressor().setClosedLoopControl(true);
		System.out.println(Controllers.getInstance().getCompressor().enabled());

		while (isOperatorControl() && isEnabled()) {
			oI.run();
			threadRateControl.doRateControl(20);
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
