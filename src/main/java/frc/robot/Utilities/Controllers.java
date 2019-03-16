package frc.robot.Utilities;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.*;

import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.CustomTalonSRX;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.Drivers.NavX;
import edu.wpi.first.wpilibj.Compressor;

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
		rightDrive2 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kRightDriveSlaveId, rightDrive1);
		rightDrive3 = CANSpeedControllerBuilder.createPermanentVictorSlaveToTalonSRX(Constants.kRightDriveSlaveId2, rightDrive1);

		hatchMotor = CANSpeedControllerBuilder.createDefaultTalonSRX(Constants.kHatchMotorId, Constants.kHatchMotorPDPChannel);
		hatchSolenoid = new DoubleSolenoid(Constants.kHatchSolenoidOut, Constants.kHatchSolenoidIn);

		cargoIntakeMotor = new VictorSPX(Constants.kCargoIntakeMotorId);
		CBcargoIntakeMotor = new VictorSP(Constants.kBCCargoIntakeMotorId); //TODO: remove once practice bot matches compbot
		//cargoShooterMotor = new CustomTalonSRX(Constants.kCargoShooterMotorId, Constants.kCargoShooterMotorPDPChannel);
		cargoIntakeSolenoid = new DoubleSolenoid(Constants.kCargoIntakeSolenoidOut, Constants.kCargoIntakeSolenoidIn);

		leftClimberMotor = new CANSparkMax(Constants.kLeftClimberMotorId, MotorType.kBrushless);
		rightClimberMotor = new CANSparkMax(Constants.kRightClimberMotorId, MotorType.kBrushless);
		climberDriveMotor = new VictorSPX(Constants.kClimberDriveMotorId);

		compressor = new Compressor();

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
	private VictorSPX cargoIntakeMotor;
	private VictorSP CBcargoIntakeMotor; //TODO: remove once practice bot matches compbot
	//private CustomTalonSRX cargoShooterMotor;

	private DoubleSolenoid cargoIntakeSolenoid;
	private DoubleSolenoid hatchSolenoid;

	private CANSparkMax leftClimberMotor;
	private CANSparkMax rightClimberMotor;
	private VictorSPX climberDriveMotor;

	private CustomJoystick driveJoystickThrottle;
	private CustomJoystick operatorJoystick;

	private NavX navX;
	private CANifier canifierLED;

	private Compressor compressor;


////////// Subsystem Motors & Stuff

	public CustomTalonSRX getHatchMotor() {
		return hatchMotor;
	}

	public VictorSPX getCargoIntakeMotor() {
		return cargoIntakeMotor;
	}

	//TODO: remove once practice bot matches compbot
	public VictorSP getCBCargoIntakeMotor() {
		return CBcargoIntakeMotor;
	}

	//public CustomTalonSRX getCargoShooterMotor() {
	//	return cargoShooterMotor;
	//}

	public DoubleSolenoid getCargoIntakeSolenoid() {
		return cargoIntakeSolenoid;
	}

	public DoubleSolenoid getHatchSolenoid() {
		return hatchSolenoid;
	}

	public CANSparkMax getLeftClimberMotor() {
		return leftClimberMotor;
	}

	public CANSparkMax getRightClimberMotor() {
		return rightClimberMotor;
	}

	public VictorSPX getClimberDriveMotor() {
		return climberDriveMotor;
	}
  
////////// Drive Motors

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


////////// Joysticks

	public CustomJoystick getDriveJoystickThrottle() {
		return driveJoystickThrottle;
	}

	public CustomJoystick getOperatorJoystick() {
		return operatorJoystick;
	}


////////// Sensors & Stuff

	public Compressor getCompressor() {
		return compressor;
	}
	
	public NavX	getNavX() {
		return navX;
	}

	public CANifier getCANifierLED() {
		return canifierLED;
	}
}
