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
import frc.robot.Autonomous.Modes.CargoShipLeftMode;
import frc.robot.Autonomous.Modes.CargoShipFrontMode;
import frc.robot.Autonomous.Modes.CargoShipBlueRightMode;
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
        autoChooser.addObject("Cargoship Left", "CargoShipLeftMode");
		autoChooser.addObject("Cargoship Front", "CargoShipFrontMode");
		autoChooser.addObject("Cargoship Right", "CargoShipBlueRightMode");
		autoChooser.addObject("Test", "TestMode");

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
        case "CargoShipLeftMode":
            autoMode = new CargoShipLeftMode();
            break;
        case "CargoShipFrontMode":
            autoMode = new CargoShipFrontMode();
			break;
		case "CargoShipBlueRightMode":
			autoMode = new CargoShipBlueRightMode();
			break;
		case "TestMode":
			autoMode = new TestMode();
			break;
        default:
            autoMode = new DriverControlMode();
            break;
		}
		
		AutoModeBase autoMode = new CargoShipLeftMode();

		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;
	
		autoModeExecuter.start();
		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {
			threadRateControl.doRateControl(100);
			//threadRateControl.doRateControl(20);
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
