package frc.robot.Utilities;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.CustomTalonSRX;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.Drivers.NavX;

public class Controllers {
	private static Controllers instance = null;

	public static Controllers getInstance() {
		if (instance == null)
			instance = new Controllers();

		return instance;
	}


	private Controllers() {

		driveJoystickThrottle = new CustomJoystick(0);
		operatorJoystick = new CustomJoystick(1);

		leftDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(Constants.kLeftDriveMasterId, Constants.kLeftDriveMasterPDPChannel);
		leftDrive2 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kLeftDriveSlaveId, leftDrive1);
		leftDrive3 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kLeftDriveSlaveId2, leftDrive1);

		rightDrive1 = CANSpeedControllerBuilder.createFastMasterTalonSRX(Constants.kRightDriveMasterId, Constants.kRightDriveMasterPDPChannel);
		rightDrive2 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kRightDriverSlaveId, rightDrive1);
		rightDrive3 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kRightDriverSlaveId2, rightDrive1);

		hatchMotor = CANSpeedControllerBuilder.createDefaultTalonSRX(Constants.kHatchMotorId, Constants.kHatchMotorPDPChannel);

		try {
			navX = new NavX(SPI.Port.kMXP);
		} catch (Exception ex) {

		}

		canifierLED = new CANifier(Constants.kCANifierLEDId);
		
	}


	private CustomTalonSRX leftDrive1;
	private BaseMotorController leftDrive2;
	private BaseMotorController leftDrive3;
	private CustomTalonSRX rightDrive1;
	private BaseMotorController rightDrive2;
	private BaseMotorController rightDrive3;

	private CustomTalonSRX hatchMotor;

	private CustomJoystick driveJoystickThrottle;
	private CustomJoystick operatorJoystick;

	private NavX navX;
	private CANifier canifierLED;


	////Subsystem Motors

	public CustomTalonSRX getHatchMotor() {
		return hatchMotor;
	}


	////Drive Motors

	public CustomTalonSRX getLeftDrive1() {
		return leftDrive1;
	}

	public CustomTalonSRX getRightDrive1() {
		return rightDrive1;
	}

	public BaseMotorController getLeftDrive2() {
		return leftDrive2;
	}

	public BaseMotorController getLeftDrive3() {
		return leftDrive3;
	}

	public BaseMotorController getRightDrive2() {
		return rightDrive2;
	}

	public BaseMotorController getRightDrive3() {
		return rightDrive3;
	}


	////Joystics and stuff

	public CustomJoystick getDriveJoystickThrottle() {
		return driveJoystickThrottle;
	}

	public CustomJoystick getOperatorJoystick() {
		return operatorJoystick;
	}


	////Sensors and stuff

	public NavX	getNavX() {
		return navX;
	}

	public CANifier getCANifierLED() {
		return canifierLED;
	}



}
