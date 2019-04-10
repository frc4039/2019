/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;

import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeExecuter;
import frc.robot.Autonomous.Modes.TestMode;
import frc.robot.Autonomous.Modes.CLVisionFrontSide;
import frc.robot.Autonomous.Modes.LVisionLeftLeft;
import frc.robot.Autonomous.Modes.CRampReverseFrontSide;
import frc.robot.Autonomous.Modes.LRampDoubleSide;
import frc.robot.Autonomous.Modes.DriverControlMode;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Subsystems.HatchSubsystem;
import frc.robot.Subsystems.CargoSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Loops.RobotStateEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

//import java.util.ArrayList;

public class Robot extends CustomRobot {
	private Looper mLooper;
	private OI oI;

	private DriveBaseSubsystem driveBaseSubsystem;
	private HatchSubsystem hatchSubsystem;
	private CargoSubsystem cargoSubsystem;
	private ClimberSubsystem climberSubsystem;

	private RobotStateEstimator robotStateEstimator;
	private ThreadRateControl threadRateControl = new ThreadRateControl();

	AutoModeBase autoMode;
    SendableChooser<String> autoChooser;
	private AutoModeExecuter autoModeExecuter;
	Boolean noAuto;

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

		climberSubsystem = ClimberSubsystem.getInstance();
		climberSubsystem.init();
		climberSubsystem.registerEnabledLoops(mLooper);

		robotStateEstimator = RobotStateEstimator.getInstance();
		mLooper.register(robotStateEstimator);

		CameraServer.getInstance().startAutomaticCapture(0);

		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault("Driver Control", "DriverControl");
		
		autoChooser.addObject("Center Reverse Front to Side", "Center Reverse Front to Side");
		autoChooser.addObject("Left Ramp Double Side","LRampDoubleSide");
		
		autoChooser.addObject("Vision Left","LVisionLeftLeft");
		autoChooser.addObject("Vision Center (left)","CLVisionFrontSide");

        SmartDashboard.putData("Autonomous Chooser", autoChooser);
		
	}

	@Override
	public void autonomous() {
		mLooper.start(true);
		driveBaseSubsystem.setBrakeMode(true);
		autoModeExecuter = new AutoModeExecuter();

		String selectedAuto = (String) autoChooser.getSelected();
        System.out.println(selectedAuto);
        switch (selectedAuto) {
		case "Center Reverse Front to Side":
			autoMode = new CRampReverseFrontSide();
			noAuto = false;
			break;
		case "LRampDoubleSide":
			autoMode = new LRampDoubleSide();
			noAuto = false;
			break;
		case "LVisionLeftLeft":
			autoMode = new LVisionLeftLeft();
			noAuto = false;
			break;
		case "CLVisionFrontSide":
			autoMode = new CLVisionFrontSide();
			noAuto = false;
			break;
        default:
			autoMode = new DriverControlMode();
			noAuto = true;
            break;
		}

		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;
	
		autoModeExecuter.start();
		threadRateControl.start(true);

		noAuto = true;

		while (isAutonomous() && isEnabled()) {
			if (noAuto == true){
				threadRateControl.doRateControl(20);
				oI.run();
			} else {
				threadRateControl.doRateControl(100);
				noAuto = oI.autoCancel();
			}
		}
	}

	@Override
	public void operatorControl() {
		exitAuto();
		mLooper.start(false);
		threadRateControl.start(true);

		Controllers.getInstance().getCompressor().start();
		Controllers.getInstance().getCompressor().setClosedLoopControl(true);
		
		

		while (isOperatorControl() && isEnabled()) {

			if ((climberSubsystem.getClimberSystemState() == "HOLDING") || (climberSubsystem.getClimberSystemState() == "READYING")){
				Controllers.getInstance().getCompressor().setClosedLoopControl(true);
			} else {
				Controllers.getInstance().getCompressor().setClosedLoopControl(false);
			}

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

		while (isTest() &&  isEnabled()) {
			oI.test();
		}
	}
}
