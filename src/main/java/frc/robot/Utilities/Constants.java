package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Solenoid;

public class Constants {

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////  Driver/Operator Buttons
	////////////////////////////////////////////////////////////////////////////////////////////////////


	//Driver control buttons
	public static final int DRIVE_X_AXIS = 0;
	public static final int DRIVE_Y_AXIS = 1;
	public static final double kJoystickDeadband = 0.08;


	//Operator controller buttons
	public static final int OPERATOR_Y_AXIS = 0;
	public static final int HATCH_SCORE = 1; 	 //A
	public static final int HATCH_PICKUP = 2;	 //B
	public static final int HATCH_ZERO = 3; 	 //X
	public static final int CARGO_INTAKE = 5;    //left bumper
	public static final int CARGO_SHOOTER = 6;   //right bumper
	//public static final int INTAKE_SOLENOID = 7; 
	public static final int CARGO_WINDUP = 7;
	public static final int CARGO_HOLD = 8;  //start
 
	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Hatch Subsystem Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	public static final double kHatchAcquiringPosition = 80; //this is made up
	public static final double kHatchHoldingPosition = -440; //this is also made up
	public static final double kHatchEjectPosition = 0; //definitely made up

	public static final double kHatchEjectTime = 2; //please just change these asap

	public static final double kHatchPositionKp = 7.0;
	public static final double kHatchPositionKi = 0.0;
	public static final double kHatchPositionKd = 70.0;
	public static final double kHatchPositionKf = 0.0;
	public static final double kHatchPositionRampRate = 0.0;
	public static final int kHatchPositionIZone = 0;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Cargo Subsystem Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	public static final double kCargoIntakeDownPosition = 80; //this is made up
	public static final double kCargoIntakeUpPosition = -440; //this is also made up
	
	public static final int kCargoIntakeSolenoidOut = 1; //TODO: CHANGE IMMEDIATELY
	public static final int kCargoIntakeSolenoidIn = 2; 

	public static final double kCargoShootingSpeed = 1.0; //TODO: CHANGE IMMEDIATELY
	public static final double kCargoFeedingSpeed = 0.5;  /////////
	public static final double kCargoIntakingSpeed = 0.5;       /////////

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Robot Drive Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	// Wheels
	public static final double kDriveWheelDiameterInches = 6.125;
	//public static final double kDriveWheelDiameterInches = 4.875;	//Practice bot calibrated 4.875
	//public static final double kDriveWheelDiameterInches = 5;	//Comp bot measured val
	public static final double kTrackWidthInches = 23.5;
	//public static final double kTrackWidthInches = 25.5;
	public static final double kTrackScrubFactor = 1.0; // 0.924 ?

	// Geometry
	public static final double kCenterToFrontBumperDistance = 18.75;
	public static final double kCenterToIntakeDistance = 18.75;
	public static final double kCenterToRearBumperDistance = 18.75;
	public static final double kCenterToSideBumperDistance = 16.375;

	// Path following constants
	public static final double kMinLookAhead = 12.0; // inches
	public static final double kMinLookAheadSpeed = 9.0; // inches per second
	public static final double kMaxLookAhead = 24.0; // inches
	public static final double kMaxLookAheadSpeed = 140.0; // inches per second
	public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
	public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

	public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain * our speed in inches per sec
	public static final double kSegmentCompletionTolerance = 1; // inches
	public static final double kPathFollowingMaxAccel = 100.0; // inches per second^2
	public static final double kPathFollowingMaxVel = 140.0; // inches per second

	public static final double kPathFollowingProfileKp = 5.0;   //Used to be 5 when tuning our paths
	public static final double kPathFollowingProfileKi = 0.03;
	public static final double kPathFollowingProfileKv = 0.2;
	public static final double kPathFollowingProfileKffv = 1.0;
	public static final double kPathFollowingProfileKffa = 0.05;
	public static final double kPathFollowingGoalPosTolerance = 1;
	public static final double kPathFollowingGoalVelTolerance = 18.0;
	public static final double kPathStopSteeringDistance = 9.0;

	public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;

	//public static final double kSensorUnitsPerRotation = 4096.0;    //original value for encoder 1:1 with shaft
	public static final double kSensorUnitsPerRotation = 30720.0;    //value for encoder on 3 cim ballshifter with 60:24 3rd stage

	public static final double k100msPerMinute = 600.0;
	public static final double kLooperDt = 0.005;

		/* CONTROL LOOP GAINS */

	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveHighGearVelocityKp = 0.004;
	public static final double kDriveHighGearVelocityKi = 0.0002;
	public static final double kDriveHighGearVelocityKd = 0.02;
	public static final double kDriveHighGearVelocityKf = 0.03175;
	public static final int kDriveHighGearVelocityIZone = 1000;
	public static final double kDriveHighGearVelocityRampRate = 0.1;
	public static final double kDriveHighGearMaxSetpoint = 12.0 * 12.0; // 12 fps

/* 	public static final double kDriveHighGearVelocityKp = 1;
	public static final double kDriveHighGearVelocityKi = 0.005;
	public static final double kDriveHighGearVelocityKd = 1.6;
	public static final double kDriveHighGearVelocityKf = 0.165;
	public static final int kDriveHighGearVelocityIZone = 0;
	public static final double kDriveHighGearVelocityRampRate = 0.1;
	public static final double kDriveHighGearMaxSetpoint = 12.0 * 12.0; // 12 fps */

	public static final int kTimeoutMs = 20;
	public static final int kTimeoutMsFast = 10;
	public static final int kTalonRetryCount = 3;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Motor Controller IDS
	//// Don't change unless robot is rewired
	///////////////////////////////////////////////////////////////////////////////////////////////////

	// Hatch
	public static final int kHatchMotorId = 9; //not the real value, needs to be changed
	public static final int kHatchMotorPDPChannel = 11;

	// Cargo
	public static final int kCargoIntakeMotorId = 0;
	public static final int kCargoShooterMotorId = 1;
	public static final int kCargoIntakeMotorPDPChannel = 4;
	public static final int kCargoShooterMotorPDPChannel = 5;

	// Drive
	public static final int kLeftDriveMasterId = 2; // left back
	public static final int kLeftDriveSlaveId = 3; // left top
	public static final int kLeftDriveSlaveId2 = 4; // left front
	public static final int kRightDriveMasterId = 5; // right back
	public static final int kRightDriveSlaveId = 6; // right top
	public static final int kRightDriveSlaveId2 = 7; //right front

//////// Check again after testing
	public static final int kLeftDriveMasterPDPChannel = 14; // left back
	public static final int kLeftDriveMasterPDPBreakerRating = 40;
	public static final int kLeftDriveSlave1PDPChannel = 13; // left top
	public static final int kLeftDriveSlave1PDPBreakerRating = 40;
	public static final int kLeftDriveSlave2PDPChannel = 12; // left front
	public static final int kLeftDriveSlave2PDPBreakerRating = 40;
	public static final int kRightDriveMasterPDPChannel = 15; // right back
	public static final int kRightDriveMasterPDPBreakerRating = 40;
	public static final int kRightDriveSlave1PDPChannel = 2; // right top
	public static final int kRightDriveSlave1PDPBreakerRating = 40;
	public static final int kRightDriveSlave2PDPChannel = 3; // right front
	public static final int kRightDriveSlave2PDPBreakerRating = 40;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Not used, but 195 or 254 used them and now if you take them out it breaks our code        ////
	///////////////////////////////////////////////////////////////////////////////////////////////////

	//195 LEDs
	public static final int kLEDThreadPriority = Thread.MIN_PRIORITY;
	public static final int kCANifierLEDId = 30;

	// Goal tracker constants
	public static final double kMaxGoalTrackAge = 1.0;
	public static final double kMaxTrackerDistance = 18.0;
	public static final double kCameraFrameRate = 30.0;
	public static final double kTrackReportComparatorStablityWeight = 1.0;
	public static final double kTrackReportComparatorAgeWeight = 1.0;

	// Pose of the camera frame w.r.t. the robot frame
	public static final double kCameraXOffset = -3.3211;
	public static final double kCameraYOffset = 0.0;
	public static final double kCameraZOffset = 20.9;
	public static final double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
	public static final double kCameraYawAngleDegrees = 0.0;
	public static final double kCameraDeadband = 0.0;

	// Target parameters
	// Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
	// Section 3.13
	// ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
	// Parts GE-17203-FLAT and GE-17371 (sheet 7)
	public static final double kBoilerTargetTopHeight = 88.0;
	public static final double kBoilerRadius = 7.5;


}
