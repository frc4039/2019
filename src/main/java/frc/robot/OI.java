package frc.robot;

//import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.Actions.AutomatedActions;
//import frc.robot.Actions.TurnToHeadingAction;
import frc.robot.Subsystems.HatchSubsystem;
import frc.robot.Subsystems.HatchSubsystem.WantedState;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

public class OI implements Runnable {
	private static OI instance = null;

	private DriveBaseSubsystem driveBaseSubsystem;
	private HatchSubsystem hatchSubsystem;
	//private DriverStation ds;
	private CustomJoystick driveJoystickThrottle;
	private CustomJoystick operatorJoystick;
	//private DriveHelper driveHelper;

	private OI() throws Exception {
		//ds = DriverStation.getInstance();

		Controllers robotControllers = Controllers.getInstance();
		driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();
		operatorJoystick = robotControllers.getOperatorJoystick();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		hatchSubsystem = HatchSubsystem.getInstance();

		//driveHelper = new DriveHelper();
	}

	public static OI getInstance() {
		if(instance == null) {
			try {
				instance = new OI();
			} catch (Exception ex) {
				
			}
		}

		return instance;
	}

	@Override
	public void run() {

		///////////////////////////////
		//Hatch Control
		if (operatorJoystick.getRawButton(Constants.HATCH_SCORE)) {
			hatchSubsystem.setWantedState(WantedState.ACQUIRE);
		} else if (operatorJoystick.getRawButton(Constants.HATCH_PICKUP)) {
			hatchSubsystem.setWantedState(WantedState.HOLD);
		} //else {
		//	hatchSubsystem.setWantedState(WantedState.HOLD);
		//}
		///////////////////////////////

	

		///////////////////////////////
		//Drivebase control
		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband);
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband);

		driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y - x, 1)));
		///////////////////////////////

		//TODO: Enable for new drivers maybe
		//CheesyDrive for new drivers
//		driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));



//		driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
//		driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 650, (y - x) * 650));
	}

}

