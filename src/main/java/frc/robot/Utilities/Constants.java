package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Solenoid;

public class Constants {

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////  Driver/Operator Buttons
	////////////////////////////////////////////////////////////////////////////////////////////////////


	//Driver control axis
	public static final int DRIVE_X_AXIS = 4;  		//Right stick X-axis
	public static final int DRIVE_Y_AXIS = 1;		//Left stick Y-axis
	public static final double kJoystickDeadband = 0.08; //Not a button

	public static final int LEFT_TRIGGER = 2; //Left trigger
	public static final int RIGHT_TRIGGER = 3; //Right trigger
	public static final double kTriggerDeadband = 0.05; //not a button


	//Driver control buttons
	public static final int CLIMBER_RETRACT = 1; //A
	public static final int CLIMB_HOLD = 2;	 //B
	public static final int CLIMB_MANUAL = 3; //X
	public static final int CLIMBER_EXTEND = 4; //Y
	public static final int VISION_ASSIST = 5; //Left bumper
	public static final int DRIVER_SCORE = 6; //Right bumper
	public static final int DRIVER_INTAKE = 9; //Left stick click
	public static final int TURN_0 = 0;
	public static final int TURN_90 = 90;
	public static final int TURN_180 = 180;
	public static final int TURN_270 = 270;
 

	//Operator controller axis

	public static final int OPERATOR_X_AXIS = 1; //Left stick Y-axis

	//Operator controller buttons
	public static final int HATCH_SCORE = 1; //A
	public static final int HATCH_PICKUP = 2; //B
	public static final int HATCH_ZERO = 3; //X
	public static final int CLIMBER_INITIATE = 4; //Y
	public static final int CARGO_INTAKE = 5; //Left bumper
	public static final int CARGO_HOLD = 6; //Right bumper
	public static final int CARGO_SHOOTER = 7; //Left tiny button
	public static final int AUTO_RESET = 8; //Right tiny button??
	public static final int RESET_ENCODER = 10; //Left stick click
	public static final int CARGO_PUSH = 9; //Fake button? To be reassigned

	public static final int PIPELINE_0 = 0;   	//D-Pad Up
	public static final int PIPELINE_1 = 180;   //D-Pad Down
	public static final int PIPELINE_2 = 270;   //D-Pad Left
	public static final int PIPELINE_3 = 90;   	//D-Pad Right

	//System check
	//Axis
	public static final int DRIVE_Y = 0; //check 
	public static final int DRIVE_X = 1; //check
	public static final int CLIMB_EXTEND = 2; //Left trigger
	public static final int CLIMB_RETRACT = 3; //Right trigger

	//Buttons
	public static final int HATCH_OPEN = 1; //A
	public static final int HATCH_CLOSE = 2; //B
	public static final int RESET_ENCODERS = 3; //X
	public static final int CLIMB_INITIATE = 4; //Y
	public static final int INTAKE_IN = 5; //Left bumper
	public static final int INTAKE_OUT = 6; //Right bumper
	public static final int WINDUP = 7; //Left little button
	public static final int SHOOTING = 8; //Right little button
	public static final int HOLD = 9; //Left stick click
	
	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Hatch Subsystem Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	public static final double kHatchAcquiringPosition = 535;
	public static final double kHatchHoldingPosition = 0; 
	public static final double kHatchEjectPosition = 182; 
	public static final double kHatchAutoHoldingPosition = 475;

	public static final double kHatchAcquiringPercentage = 0.05;
	public static final double kHatchHomingPercentage = -0.30;
	public static final double kHatchSuperHold = -0.80;
	public static final double kHatchVoltageLimit = 0.3;
	public static final double kHatchVoltageUnlimited = 1.0;

	public static final double kHatchEjectTime = 0.25;
	public static final double kHatchHomeTime = 2.5;
	public static final double kHatchHoldTime = 2.0;

	public static final double kHatchPositionKp = 4.0;
	public static final double kHatchPositionKi = 0.0;
	public static final double kHatchPositionKd = 150.0;
	public static final double kHatchPositionKf = 0.0;
	public static final double kHatchPositionRampRate = 0.0;
	public static final int kHatchPositionIZone = 0;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Cargo Subsystem Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	public static final double kCargoShootingSpeed = 0.3; 
	public static final double kCargoFeedingSpeed = 1.0;  
	public static final double kCargoIntakingSpeed = 1.0;  

	public static final double kCargoShootingVelocity = 6000;

	
	public static final double kCargoShooterVelocityKp = 0.005;
	public static final double kCargoShooterVelocityKi = 0;
	public static final double kCargoShooterVelocityKd = 0;
	public static final double kCargoShooterVelocityKf = 0.0425;
	public static final int kCargoShooterVelocityIZone = 0;
	public static final double kCargoShooterVelocityRampRate = 1.0;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Climber Subsystem Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	public static final double kClimberExtendedPosition = 0.4; // TODO: change
	public static final double kClimberHomePosition = 0.0; 

	public static final int kClimberLimitSwitchBottom = 0;
	public static final int kClimberLimitSwitchTop = 1;
	public static final int kClimberLimitSwitchFront = 2;

	public static final double kClimbDrivebasePercent = 0.25;
	public static final double kClimbRetractTinyWheelsPercent = 0.40;

/*
	public static final double kClimberPositionkP = 0;
	public static final double kClimberPositionkI = 0;
	public static final double kClimberPositionkD = 0;
	public static final double kClimberPositionkIz = 0;
	public static final double kClimberPositionkFF = 0.005;
	public static final double kClimberPositionkMaxOutput = 1.0;
	public static final double kClimberPositionkMinOutput = -1.0;
	public static final double kClimberPositionkMaxRPM = 5700;
	public static final double kClimberMaxVel = 5700;
	public static final double kClimberMinVel = 0;
	public static final double kClimberMaxAccel = 5700;
	public static final double kClimberAllowedError = 0;
	public static final int kClimberSlotID = 0;
*/
	public static final double kClimberUp = 0;
	public static final double kClimberDown = 465/2; // 475
	//public static final double kStartingPosition = 0.024;
	public static final double kClimberFirstLimitSwitch = 235/2; // 228.1
	public static final double kClimberAlmostFirstLimitSwitch = 200/2;
	public static final double kClimberAlmostDown = 450/2;

	public static final double kClimberDownFast = 1.0;
	public static final double kClimberDownSlow = 0.5;
	public static final double kClimberRetractSpeed = -1.0;
	public static final double kClimberHoldPositionSpeed = 0.05;


	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Robot Drive Constants
	///////////////////////////////////////////////////////////////////////////////////////////////////

	// Vision Assist
	public static final double kVisionAssistP = 0.4;
	public static final double kVisionAssistI = 0;
	public static final double kVisionAssistD = 10;
	public static final double kVisionAssistF = 0.0;
	public static final double kVisionAssistEpsilon = 0;

	public static final double kAutoDriveVision = 0.25; //forward percent value for camera auto

	public static final double kTurnAssistP = 0.01;
	public static final double kTurnAssistI = 0;
	public static final double kTurnAssistD = 10;
	public static final double kTurnAssistF = 0.10;
	public static final double kTurnAssistEpsilon = 0;

	// Wheels
	public static final double kDriveWheelDiameterInches = 6.35;
	//public static final double kDriveWheelDiameterInches = 4.875;	//Practice bot calibrated 4.875
	//public static final double kDriveWheelDiameterInches = 5;	//Comp bot measured val
	public static final double kTrackWidthInches = 26;
	//public static final double kTrackWidthInches = 25.5;
	public static final double kTrackScrubFactor = 0.3; // 0.924 ?

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
	public static final double kPathFollowingMaxAccel = 140.0; // inches per second^2
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

	public static final double kSensorUnitsPerRotation = 4096.0;    //original value for encoder 1:1 with shaft
	// public static final double kSensorUnitsPerRotation = 30720.0;    //value for encoder on 3 cim ballshifter with 60:24 3rd stage

	public static final double k100msPerMinute = 600.0;
	public static final double kLooperDt = 0.005;

		/* CONTROL LOOP GAINS */

	//TODO: change
	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveHighGearVelocityKp = 0.375;
	public static final double kDriveHighGearVelocityKi = 0.0;
	public static final double kDriveHighGearVelocityKd = 15.0;
	public static final double kDriveHighGearVelocityKf = 0.2803;
	public static final int kDriveHighGearVelocityIZone = 0;
	public static final double kDriveHighGearVelocityRampRate = 0.1;
	public static final double kDriveHighGearMaxSetpoint = 12.0 * 14.0; // 14 fps

	public static final int kTimeoutMs = 20;
	public static final int kTimeoutMsFast = 10;
	public static final int kTalonRetryCount = 3;

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//// Motor Controller IDS
	//// Don't change unless robot is rewired
	///////////////////////////////////////////////////////////////////////////////////////////////////

	// Hatch
	public static final int kHatchMotorId = 8;
	public static final int kHatchMotorPDPChannel = 11;


	public static final int kHatchSolenoidOut = 0;
	public static final int kHatchSolenoidIn = 1;

	// Cargo
	public static final int kCargoIntakeMotorId = 13;
	//public static final int kBCCargoIntakeMotorId = 0; //TODO: remove once practice bot matches compbot (VictorSP -> VictorSPX)
	//public static final int kCargoShooterMotorId = 12;
	public static final int kCargoIntakeMotorPDPChannel = 4;
	//public static final int kCargoShooterMotorPDPChannel = 5;

	public static final int kCargoIntakeSolenoidOut = 2;
	public static final int kCargoIntakeSolenoidIn = 3; 

	// Climber
	public static final int kClimberDriveMotorId = 11; //TODO: change?? <-when is this from, is it still a todo??
	public static final int kClimberDriveMotorPDPChannel = 11;
	public static final int kLeftClimberMotorId = 9;
	public static final int kLeftClimberMotorPDPChannel = 0;
	public static final int kRightClimberMotorId = 10;
	public static final int kRightClimberMotorPDPChannel = 15;

	public static final int kClimberSolenoidOut = 4;
	public static final int kClimberSolenoidIn = 5;


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
