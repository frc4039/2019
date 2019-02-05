package frc.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import frc.robot.Utilities.*;
import frc.robot.Utilities.Drivers.CustomTalonSRX;
import frc.robot.Utilities.Drivers.NavX;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.TrajectoryFollowingMotion.*;

import java.util.concurrent.locks.ReentrantLock;

public class DriveBaseSubsystem implements CustomSubsystem {
	private static DriveBaseSubsystem instance = null;

	private CustomTalonSRX mLeftMaster, mRightMaster;
	private BaseMotorController leftDriveSlave1, leftDriveSlave2, rightDriveSlave1, rightDriveSlave2;

	private NavX mNavXBoard;

	private DriveControlState mControlMode;

	private static ReentrantLock _subsystemMutex = new ReentrantLock();

	private boolean mPrevBrakeModeVal;

	private Path mCurrentPath = null;
	private PathFollower mPathFollower;

	private PathFollowerRobotState mRobotState = PathFollowerRobotState.getInstance();

	public static DriveBaseSubsystem getInstance() {
		if (instance == null)
			instance = new DriveBaseSubsystem();

		return instance;
	}

	private DriveBaseSubsystem() {

		Controllers robotControllers = Controllers.getInstance();
		mLeftMaster = robotControllers.getLeftDrive1();
		leftDriveSlave1 = robotControllers.getLeftDrive2();
		leftDriveSlave2 = robotControllers.getLeftDrive3();
		mRightMaster = robotControllers.getRightDrive1();
		rightDriveSlave1 = robotControllers.getRightDrive2();
		rightDriveSlave2 = robotControllers.getRightDrive3();

		mNavXBoard = robotControllers.getNavX();


		mPrevBrakeModeVal = false;
		setBrakeMode(true);

		mControlMode = DriveControlState.PATH_FOLLOWING;

	}


	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (DriveBaseSubsystem.this) {
				subsystemHome();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (DriveBaseSubsystem.this) {
				setDriveOpenLoop(DriveMotorValues.NEUTRAL);
				setBrakeMode(false);
				setDriveVelocity(new DriveMotorValues(0, 0));
			}
		}

		@Override
		public void onLoop(double timestamp, boolean isAuto) {
			synchronized (DriveBaseSubsystem.this) {
//				SmartDashboard.putNumber("Left Drive Velocity", getLeftVelocityInchesPerSec());
//				SmartDashboard.putNumber("Right Drive Velocity", getRightVelocityInchesPerSec());
				switch (mControlMode) {
					case OPEN_LOOP:
						break;
					case VELOCITY:
						break;
					case TURN_TO_HEADING:
						break;
					case PATH_FOLLOWING:
						if (mPathFollower != null) {
							updatePathFollower(timestamp);
							//mCSVWriter.add(mPathFollower.getDebug());
						}
						break;
					default:
						break;
				}

			}
		}
		@Override
		public void onStop(double timestamp) {
			setDriveOpenLoop(DriveMotorValues.NEUTRAL);
		}
	};

	@Override
	public void init() {
		mLeftMaster.setSensorPhase(true);
		mRightMaster.setSensorPhase(false);

		mLeftMaster.setInverted(false);
		leftDriveSlave1.setInverted(false);
		leftDriveSlave2.setInverted(false);

		mRightMaster.setInverted(true);
		rightDriveSlave1.setInverted(true);
		rightDriveSlave2.setInverted(true);

		setBrakeMode(true);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mLeftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mLeftMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mRightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mRightMaster.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mLeftMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(mRightMaster, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(mLeftMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
		setSucceeded &= TalonHelper.setPIDGains(mRightMaster, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);

		mLeftMaster.selectProfileSlot(0, 0);
		mRightMaster.selectProfileSlot(0, 0);
	}

	@Override
	public void subsystemHome() {
		mNavXBoard.zeroYaw();

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mLeftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
			setSucceeded &= mRightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

//		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
//			ConsoleReporter.report("Failed to zero DriveBaseSubsystem!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(mLoop);
	}

	public void setControlMode(DriveControlState controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {

			}
		}
	}

	public void setBrakeMode(boolean brakeMode) {
		if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();
			mLeftMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mRightMaster.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();
		}
	}

	public synchronized void setDriveOpenLoop(DriveMotorValues d) {
		setControlMode(DriveControlState.OPEN_LOOP);

		mLeftMaster.set(ControlMode.PercentOutput, d.leftDrive);
		mRightMaster.set(ControlMode.PercentOutput, d.rightDrive);
	}

	public synchronized void setDriveVelocity(DriveMotorValues d) {
		setDriveVelocity(d, true);
	}

	public synchronized void setDriveVelocity(DriveMotorValues d, boolean autoChangeMode) {
		if (autoChangeMode)
			setControlMode(DriveControlState.VELOCITY);
		mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.leftDrive));
		mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(d.rightDrive));
	}

	private void updatePathFollower(double timestamp) {
		RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
		Twist2d command = mPathFollower.update(timestamp, robot_pose,
				PathFollowerRobotState.getInstance().getDistanceDriven(), PathFollowerRobotState.getInstance().getPredictedVelocity().dx);

		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updatePathVelocitySetpoint(setpoint.left, setpoint.right);

		} else {
			updatePathVelocitySetpoint(0, 0);
			setControlMode(DriveControlState.VELOCITY);
		}
	}

	private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
		final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

		mLeftMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)));
		mRightMaster.set(ControlMode.Velocity, Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)));
		//ConsoleReporter.report("Requested Drive Velocity Left2Cube/Right2Cube: " + left_inches_per_sec + "/" + right_inches_per_sec);
		//ConsoleReporter.report("Actual Drive Velocity Left2Cube/Right2Cube: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
	}

	private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}

	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	public double getLeftDistanceInches() {
		return rotationsToInches(mLeftMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
	}

	public double getRightDistanceInches() {
		return rotationsToInches(mRightMaster.getSelectedSensorPosition(0)/Constants.kSensorUnitsPerRotation);
	}

	public double getLeftVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0))); }

	public double getRightVelocityInchesPerSec() { return rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0))); }

	public synchronized Rotation2d getGyroAngle() {
		return mNavXBoard.getYaw();
	}

	public synchronized void setGyroAngle(Rotation2d angle) {
		mNavXBoard.reset();
		mNavXBoard.setAngleAdjustment(angle);
	}

	public synchronized void setWantDrivePath(Path path, boolean reversed) {
		if (mCurrentPath != path || mControlMode != DriveControlState.PATH_FOLLOWING) {
			setControlMode(DriveControlState.PATH_FOLLOWING);
			PathFollowerRobotState.getInstance().resetDistanceDriven();
			mPathFollower = new PathFollower(path, reversed,
					new PathFollower.Parameters(
							new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
									Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
							Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
							Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
							Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
							Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
							Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
							Constants.kPathStopSteeringDistance));

			mCurrentPath = path;
		} else {

		}
	}

	public synchronized boolean isDoneWithPath() {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.isFinished();
		} else {
			if (mPathFollower != null)
				return mPathFollower.isFinished();
			else
				return true;
		}
	}

	public synchronized void forceDoneWithPath() {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			mPathFollower.forceFinish();
		} else {
		}
	}

	public synchronized boolean hasPassedMarker(String marker) {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.hasPassedMarker(marker);
		} else {
			if (mPathFollower != null)
				return (mPathFollower.isFinished() || mPathFollower.hasPassedMarker(marker));
			else {
				return true;
			}
		}
	}

}
