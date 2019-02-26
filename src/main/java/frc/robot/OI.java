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
	private CustomJoystick testJoystick;
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
		//////////////////////////////////////////////////////////////
		//Hatch Control
		if (operatorJoystick.getRisingEdgeButton(Constants.HATCH_PICKUP)) {
			hatchSubsystem.setHatchWantedState(HatchWantedState.SUPERHOLD);
		}
		if (operatorJoystick.getRisingEdgeButton(Constants.HATCH_SCORE)) {
			hatchSubsystem.setHatchWantedState(HatchWantedState.ACQUIRE);
		}
		if (operatorJoystick.getRisingEdgeButton(Constants.HATCH_ZERO)) {
			hatchSubsystem.setHatchWantedState(HatchWantedState.HOME);
		}
		//////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////
		//Cargo Control

		// Intake
		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_INTAKE)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.INTAKE);
			} else if (cargoSubsystem.getCargoSystemState() == "INTAKING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
			}
		}

		// Shoot
		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_SHOOTER)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.WINDUP);
			} else if (cargoSubsystem.getCargoSystemState() == "WINDINGUP") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.SHOOT);
			} else if (cargoSubsystem.getCargoSystemState() == "SHOOTING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
			}
		}

		//Push
		/*if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_PUSH)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.PUSH);
			} else if (cargoSubsystem.getCargoSystemState() == "PUSHING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.INTAKE);
			}
		}*/

		// Hold
		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_HOLD)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
		}
		//////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////
		//Climber control

		//operator
		if (operatorJoystick.getRisingEdgeButton(Constants.CLIMBER_INITIATE)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.INITIATE);
		}
		if (operatorJoystick.getRisingEdgeButton(Constants.RESET_ENCODER)) {
			climberSubsystem.subsystemHome();
		}

		//driver
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMBER_EXTEND)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.EXTEND);

		}
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMBER_RETRACT)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.RETRACT);
		}
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMB_RESET)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.RESET);
		} else if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMB_HOLD)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.HOLD);
		}
		//////////////////////////////////////////////////////////////

		
		//////////////////////////////////////////////////////////////
		//Camera control
		if (operatorJoystick.getPOV()==Constants.PIPELINE_0) {
			table.getEntry("pipeline").setNumber(0);
		} else if (operatorJoystick.getPOV()==Constants.PIPELINE_1) {
			table.getEntry("pipeline").setNumber(1);
		} else if (operatorJoystick.getPOV()==Constants.PIPELINE_2) {
			table.getEntry("pipeline").setNumber(2);
		} else if (operatorJoystick.getPOV()==Constants.PIPELINE_3) {
			table.getEntry("pipeline").setNumber(3);
		} 
		///////////////////////////////


		//////////////////////////////////////////////////////////////
		//Drivebase control
		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

		//Vision
		if (driveJoystickThrottle.getRawButton(Constants.VISION_ASSIST)){
			table.getEntry("ledMode").setNumber(3); //Turns LED's on
			table.getEntry("camMode").setNumber(0); //Set camera to vision mode
			driveBaseSubsystem.setVisionAssist(new DriveMotorValues(y, x));
		} else if (climberSubsystem.getClimberSystemState() == "DRIVING" || climberSubsystem.getClimberSystemState() == "RETRACTING") {
			driveBaseSubsystem.setDriveClimb();
		} else {
			table.getEntry("ledMode").setNumber(1); //Turns LED's off
			table.getEntry("camMode").setNumber(1); //Set camera to camera mode
			driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(y, x));
		}

		//Score cargo/hatch
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.DRIVER_SCORE)) {
			if (cargoSubsystem.getCargoSystemState() == "WINDINGUP") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.SHOOT);
			}
			if (hatchSubsystem.getHatchSystemState() == "HOLDING") {
				hatchSubsystem.setHatchWantedState(HatchWantedState.ACQUIRE);
			}
		}

		//Intake
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.DRIVER_INTAKE)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.INTAKE);
			} else if (cargoSubsystem.getCargoSystemState() == "INTAKING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
			}
		}
		//////////////////////////////////////////////////////////////

		//TODO: Enable for new drivers maybe
		//CheesyDrive for new drivers
		//driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
		//driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 650, (y - x) * 650));
	}

	public void test() {
		//////////////////////////////////////////////////////////////
		//Hatch Control
		if (testJoystick.getRisingEdgeButton(Constants.HATCH_OPEN)) {
			hatchSubsystem.setHatchWantedState(HatchWantedState.HOLD);
		}
		if (testJoystick.getRisingEdgeButton(Constants.HATCH_CLOSE)) {
			hatchSubsystem.setHatchWantedState(HatchWantedState.ACQUIRE);
		}
		//////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////
		//Cargo Control
		// Intake
		if (testJoystick.getRisingEdgeButton(Constants.INTAKE_OUT)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.INTAKE);
		}
		if (testJoystick.getRisingEdgeButton(Constants.INTAKE_IN)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
		}

		// Shoot
		if (testJoystick.getRisingEdgeButton(Constants.WINDUP)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.WINDUP);
		}
		if (testJoystick.getRisingEdgeButton(Constants.SHOOTING)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.SHOOT);
		}

		//Push
		/*if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_PUSH)) {
			if (cargoSubsystem.getCargoSystemState() == "HOLDING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.PUSH);
			} else if (cargoSubsystem.getCargoSystemState() == "PUSHING") {
				cargoSubsystem.setCargoWantedState(CargoWantedState.INTAKE);
			}
		}*/

		// Hold
		if (operatorJoystick.getRisingEdgeButton(Constants.CARGO_HOLD)) {
			cargoSubsystem.setCargoWantedState(CargoWantedState.HOLD);
		}
		//////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////
		//Climber control
		//operator
		if (testJoystick.getRisingEdgeButton(Constants.CLIMB_INITIATE)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.INITIATE);
		}
		if (testJoystick.getRisingEdgeButton(Constants.RESET_ENCODERS)) {
			climberSubsystem.subsystemHome();
		}

		//driver
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMBER_EXTEND)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.EXTEND);
		}
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMB_RETRACT)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.RETRACT);
		}
		if (driveJoystickThrottle.getRisingEdgeButton(Constants.CLIMB_RESET)) {
			climberSubsystem.setClimberWantedState(ClimberWantedState.RESET);
		}
		//////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////
		//Drivebase control
		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X), Constants.kJoystickDeadband);
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y), Constants.kJoystickDeadband);
		//////////////////////////////////////////////////////////////
	}
}
