package frc.robot;

//import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.Actions.AutomatedActions;
//import frc.robot.Actions.TurnToHeadingAction;
import frc.robot.Subsystems.HatchSubsystem;
import frc.robot.Subsystems.HatchSubsystem.HatchWantedState;
import frc.robot.Subsystems.CargoSubsystem;
import frc.robot.Subsystems.CargoSubsystem.CargoWantedState;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

public class OI implements Runnable {
	private static OI instance = null;

	private DriveBaseSubsystem driveBaseSubsystem;
	private HatchSubsystem hatchSubsystem;
	private CargoSubsystem cargoSubsystem;
	// private DriverStation ds;
	private CustomJoystick driveJoystickThrottle;
	private CustomJoystick operatorJoystick;
	// private DriverHelper driveHelper;

	private OI() throws Exception {
		// ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();
		driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();
		operatorJoystick = robotControllers.getOperatorJoystick();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		hatchSubsystem = HatchSubsystem.getInstance();
		cargoSubsystem = CargoSubsystem.getInstance();

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
		//Drivebase control
		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

		x = Math.abs(x)*x;
        y = Math.abs(y)*y;

        double left = y + x;
        double right = y - x;   

		if (driveJoystickThrottle.getRawButton(Constants.TARGETING)){
            driveBaseSubsystem.setDriveTargetingHelp(new DriveMotorValues(Util.limit(left, 1), Util.limit(right, 1)));
        } else {
			driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(left, 1), Util.limit(right, 1)));
		}
		///////////////////////////////

		//TODO: Enable for new drivers maybe
		//CheesyDrive for new drivers
		//driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
		//driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 650, (y - x) * 650));
	}
}