package frc.robot;

//import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.Actions.AutomatedActions;
//import frc.robot.Actions.TurnToHeadingAction;
import frc.robot.Subsystems.HatchSubsystem;
import frc.robot.Subsystems.HatchSubsystem.HatchWantedState;
import frc.robot.Subsystems.CargoSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.CargoSubsystem.CargoWantedState;
import frc.robot.Subsystems.ClimberSubsystem.ClimberWantedState;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

import edu.wpi.first.networktables.*;

public class OI implements Runnable {
	private static OI instance = null;

	private DriveBaseSubsystem driveBaseSubsystem;
	private HatchSubsystem hatchSubsystem;
	private CargoSubsystem cargoSubsystem;
	private ClimberSubsystem climberSubsystem;
	// private DriverStation ds;
	private CustomJoystick driveJoystickThrottle;
	private CustomJoystick operatorJoystick;
	// private DriverHelper driveHelper;

	private NetworkTable table;

	private OI() throws Exception {
		// ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();
		driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();
		operatorJoystick = robotControllers.getOperatorJoystick();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		hatchSubsystem = HatchSubsystem.getInstance();
		cargoSubsystem = CargoSubsystem.getInstance();
		climberSubsystem = ClimberSubsystem.getInstance();

		table = NetworkTableInstance.getDefault().getTable("limelight");
		
		//driveHelper = new DriveHelper();
	}

	public static OI getInstance() {
		if(instance == null) {
			try {
				instance = new OI();
			} catch (Exception ex) {}
		}
		return instance;
	}

	@Override
	public void run() {
		///////////////////////////////
		//Hatch Control
		if (operatorJoystick.getRisingEdgeButton(Constants.HATCH_PICKUP)) {
			if (hatchSubsystem.getHatchSystemState() == "HOLDING") {
				hatchSubsystem.setHatchWantedState(HatchWantedState.ACQUIRE);
			} else if (hatchSubsystem.getHatchSystemState() == "ACQUIRING") {
				hatchSubsystem.setHatchWantedState(HatchWantedState.HOLD);
			} 
		} else if (operatorJoystick.getRisingEdgeButton(Constants.HATCH_ZERO)) {
			hatchSubsystem.setHatchWantedState(HatchWantedState.HOME);
		}
		///////////////////////////////

		///////////////////////////////
		//Cargo Control
		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_INTAKE)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.INTAKE);
			} else if (cargoSubsystem.getCargoSystemState() == "INTAKING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
			}
		}

		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_WINDUP)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.WINDUP);
			} else if (cargoSubsystem.getCargoSystemState() == "WINDINGUP") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.SHOOT);
			} else if (cargoSubsystem.getCargoSystemState() == "SHOOTING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
			}
		}

		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_HOLD)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
		}
		///////////////////////////////	

		///////////////////////////////
		//Climber control
		if (operatorJoystick.getRisingEdgeButton(Constants.CLIMBER_INITIATE)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.INITIATE);
			System.out.println("Initiate");
		}

		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMBER_EXTEND)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.EXTEND);
			System.out.println("Extend");
		}

		if (driveJoystickThrottle.getRisingEdgeButton(Constants.PRINT_LIMIT_SWITCH)) {
			System.out.println(climberSubsystem.getClimberLimitSwitchBottom());
			System.out.println(climberSubsystem.getClimberLimitSwitchTop());
		}

		///////////////////////////////
		//Drivebase control
		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

		if (driveJoystickThrottle.getRisingEdgeButton(Constants.VISION_ASSIST)) {
			System.out.println("led on");
		} 
		if (driveJoystickThrottle.getFallingEdgeButton(Constants.VISION_ASSIST)) {
			System.out.println("led off");
		}

		if (driveJoystickThrottle.getRawButton(Constants.VISION_ASSIST)){
			table.getEntry("ledMode").setNumber(3); //Turns LED's on
			table.getEntry("camMode").setNumber(0); //Set camera to vision mode
			driveBaseSubsystem.setVisionAssist(new DriveMotorValues(y, x));
		} else {
			table.getEntry("ledMode").setNumber(1); //Turns LED's off
			table.getEntry("camMode").setNumber(1); //Set camera to camera mode
			driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(y, x));
		}
		///////////////////////////////

		//TODO: Enable for new drivers maybe
		//CheesyDrive for new drivers
		//driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
		//driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 650, (y - x) * 650));
	}
}
