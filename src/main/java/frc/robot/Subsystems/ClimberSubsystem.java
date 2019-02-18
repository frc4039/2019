package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.Drivers.CustomTalonSRX;
import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.*;

import java.util.concurrent.locks.ReentrantLock;

/**
 * The gear grabber subsystem consists of one BAG motor used to intake and exhaust gears and one pancake piston used to
 * pivot the entire subsystem from a floor pickup position to a scoring position. The motor is driven in open loop.
 * Since the subsystem lacks any encoders, it detects when a gear has been acquired by checking whether current is above
 * a threshold value. The main things this subsystem has to are intake gears, score gears, and clear balls (run motor in
 * reverse while the grabber is down to push balls away).
 * 
 * @see Subsystem.java
 */

public class ClimberSubsystem extends Subsystem {
    private CustomJoystick driveJoystickThrottle;
    private CustomJoystick operatorJoystick;
    
    private static ClimberSubsystem mInstance;
    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private CANSparkMax mLeftClimberMotor;
    private CANSparkMax mRightClimberMotor;
    private VictorSPX mClimberDriveMotor;

    private ClimberWantedState mClimberWantedState;
    private ClimberSystemState mClimberSystemState;
    //private double mThresholdStart;

    private static DigitalInput mClimberLimitSwitchBottom = new DigitalInput(Constants.kClimberLimitSwitchBottom);
    private static DigitalInput mClimberLimitSwitchTop = new DigitalInput(Constants.kClimberLimitSwitchTop);

    private CANPIDController mClimber;
    private CANEncoder mClimberEncoder;

    private boolean mPrevBrakeModeVal;
    private boolean mHomeSuccess;

    public static ClimberSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberSubsystem();
        }
        return mInstance;
    }

    public enum ClimberWantedState {
        EXTEND,
        HOLD,    //default
        RETRACT,
        DRIVE,
        RESET,
        INITIATE
    }

    private enum ClimberSystemState {
        EXTENDING,
        HOLDING, // gripper fully closed
        RETRACTING, // gripper fully seperated
        DRIVING,
        RESETING,
        INITIATING
    }

    private ClimberSubsystem() {
        Controllers robotControllers = Controllers.getInstance();
        mLeftClimberMotor = robotControllers.getLeftClimberMotor();
        mRightClimberMotor = robotControllers.getRightClimberMotor();
        mClimberDriveMotor = robotControllers.getClimberDriveMotor();

        driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();
        operatorJoystick = robotControllers.getOperatorJoystick();

        mPrevBrakeModeVal = false;
		//setBrakeMode(true);
    }

    public void init(){

        mLeftClimberMotor.set(0);
        mRightClimberMotor.follow(mLeftClimberMotor, true);
        //mLeftClimberMotor.setInverted(true);

        mClimber = mLeftClimberMotor.getPIDController();
        mClimberEncoder = mLeftClimberMotor.getEncoder();
        //mLeftClimberMotor.
/*
        mClimber.setP(Constants.kClimberPositionkP);
        mClimber.setI(Constants.kClimberPositionkI);
        mClimber.setD(Constants.kClimberPositionkD);
        mClimber.setIZone(Constants.kClimberPositionkIz);
        mClimber.setFF(Constants.kClimberPositionkFF);
        mClimber.setOutputRange(Constants.kClimberPositionkMinOutput, Constants.kClimberPositionkMaxOutput);

        mClimber.setSmartMotionMaxVelocity(Constants.kClimberMaxVel, Constants.kClimberSlotID);
        mClimber.setSmartMotionMinOutputVelocity(Constants.kClimberMinVel, Constants.kClimberSlotID);
        mClimber.setSmartMotionMaxAccel(Constants.kClimberMaxAccel, Constants.kClimberSlotID);
        mClimber.setSmartMotionAllowedClosedLoopError(Constants.kClimberAllowedError, Constants.kClimberSlotID);
*/
        //setBrakeMode(true);
        //mHomeSuccess = false;
		boolean setSucceeded;
		int retryCounter = 0;

        //zeroSensors();
    }

    /* @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Hatch Gripper Current", mHatchGripper.getOutputCurrent());
    } */

    @Override
    public void stop() {
        setClimberWantedState(ClimberWantedState.HOLD);
    }

    @Override
    public void zeroSensors() {
        boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

            //setSucceeded &= mLeftClimberMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
            //setSucceeded &= mRightClimberMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount); 
    }

    private final Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;


        @Override
        public void onFirstStart(double timestamp) {
            synchronized (ClimberSubsystem.this) {
                mClimberSystemState = ClimberSystemState.HOLDING;
                mClimberWantedState = ClimberWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (ClimberSubsystem.this) {
                mClimberSystemState = ClimberSystemState.HOLDING;
                mClimberWantedState = ClimberWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (ClimberSubsystem.this) {
                ClimberSystemState newState = mClimberSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                
                switch (mClimberSystemState) {
                case EXTENDING:
                    newState = handleExtending(timeInState);
                    break;
                case HOLDING:
                    newState = handleHolding(timeInState);
                    break;
                case RETRACTING:
                    newState = handleRetracting(timeInState);
                    break;
                case DRIVING:
                    newState = handleDriving(timeInState);
                    break;
                case INITIATING:
                    newState = handleInitiating(timeInState);
                    break;
                case RESETING:
                    newState = handleReseting(timeInState);
                    break;
                default:
                    System.out.println("Unexpected climber system state: " + mClimberSystemState);
                    newState = mClimberSystemState;
                    break;
                }

                if (newState != mClimberSystemState) {
                    System.out.println(timestamp + ": Changed state: " + mClimberSystemState + " -> " + newState);
                    mClimberSystemState = newState;
                    mCurrentStateStartTime = Timer.getFPGATimestamp();
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            mClimberWantedState = ClimberWantedState.HOLD;
            mClimberSystemState = ClimberSystemState.HOLDING;
            // Set the states to what the robot falls into when disabled.
            stop();
        }
    };
    
    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    private ClimberSystemState handleInitiating(double timeInState) {

        if (getClimberLimitSwitchTop() == true && mClimberEncoder.getPosition() < Constants.kClimberAlmostFirstLimitSwitch) {
            mClimber.setReference(Constants.kClimberInitiateFast, ControlType.kDutyCycle);
        } else if (getClimberLimitSwitchTop() == true && mClimberEncoder.getPosition() < Constants.kClimberLimitSwitchTop) {
            mClimber.setReference(Constants.kClimberInitiateSlow, ControlType.kDutyCycle);
        } else if (getClimberLimitSwitchTop() == false) {
            mClimber.setReference(0, ControlType.kDutyCycle);
            mClimberEncoder.setPosition(0);
            setClimberWantedState(ClimberWantedState.HOLD);
        }

        SmartDashboard.putNumber("Neo encoder position: ", mClimberEncoder.getPosition());
        SmartDashboard.putNumber("Neo speed: ", mClimberEncoder.getVelocity());

        switch (mClimberWantedState) {
            case INITIATE:

                return ClimberSystemState.INITIATING;
            case EXTEND:
    
                return ClimberSystemState.EXTENDING;
            case HOLD:
    
                return ClimberSystemState.HOLDING;
            case RETRACT:
    
                return ClimberSystemState.RETRACTING;
            case DRIVE:
                
                return ClimberSystemState.DRIVING;
            case RESET:

                return ClimberSystemState.RESETING;
            default:
    
                return ClimberSystemState.HOLDING;
        }
    }

    private ClimberSystemState handleExtending(double timeInState) {

        if (getClimberLimitSwitchTop() == false && getClimberLimitSwitchBottom() == true && mClimberEncoder.getPosition() < Constants.kClimberAlmostDown) {
            mClimber.setReference(Constants.kClimberLiftFast, ControlType.kDutyCycle);
        } else if (getClimberLimitSwitchTop() == false && getClimberLimitSwitchBottom() == true && mClimberEncoder.getPosition() == Constants.kClimberDown) {
            mClimber.setReference(Constants.kClimberLiftSlow, ControlType.kDutyCycle);
        } else if (getClimberLimitSwitchTop() == false && getClimberLimitSwitchBottom() == false){
            mClimber.setReference(Constants.kClimberHoldPositionSpeed, ControlType.kDutyCycle);
            setClimberWantedState(ClimberWantedState.DRIVE);
        }

        SmartDashboard.putNumber("Neo encoder position: ", mClimberEncoder.getPosition());
        SmartDashboard.putNumber("Neo speed: ", mClimberEncoder.getVelocity());

        switch (mClimberWantedState) {
            case INITIATE:

                return ClimberSystemState.INITIATING;
            case EXTEND:
    
                return ClimberSystemState.EXTENDING;
            case HOLD:
    
                return ClimberSystemState.HOLDING;
            case RETRACT:
    
                return ClimberSystemState.RETRACTING;
            case DRIVE:
                
                return ClimberSystemState.DRIVING;
            case RESET:

                return ClimberSystemState.RESETING;
            default:
    
                return ClimberSystemState.HOLDING;
        }
    }

    private ClimberSystemState handleHolding(double timeInState) {        
        //mHomeSuccess = false;
        //mHomeSuccess = false;
        mClimber.setReference(0, ControlType.kDutyCycle);
        mClimberDriveMotor.set(ControlMode.PercentOutput, 0);

        SmartDashboard.putNumber("Neo encoder position: ", mClimberEncoder.getPosition());
        SmartDashboard.putNumber("Neo speed: ", mClimberEncoder.getVelocity());
           
        switch (mClimberWantedState) {
            case INITIATE:

                return ClimberSystemState.INITIATING;
            case EXTEND:
    
                return ClimberSystemState.EXTENDING;
            case HOLD:
    
                return ClimberSystemState.HOLDING;
            case RETRACT:
    
                return ClimberSystemState.RETRACTING;
            case DRIVE:
                
                return ClimberSystemState.DRIVING;
            case RESET:

                return ClimberSystemState.RESETING;
            default:
    
                return ClimberSystemState.HOLDING;
        }
    }

    private ClimberSystemState handleRetracting(double timeInState) {
        //mLeftClimberMotor.set(Constants.kClimberHomePosition);
        //if (getClimberLimitSwitchTop() == false) {
            //setClimberRetract();
            //setClimberDrive();

            SmartDashboard.putNumber("Neo encoder position: ", mClimberEncoder.getPosition());
            SmartDashboard.putNumber("Neo speed: ", mClimberEncoder.getVelocity());
            
            mClimberDriveMotor.set(ControlMode.PercentOutput, -Constants.kClimbRetractTinyWheelsPercent);

            
            if (mClimberEncoder.getPosition() > Constants.kClimberUp + 10){
                mClimber.setReference(Constants.kClimberRetractSpeed, ControlType.kDutyCycle);
            } else if (mClimberEncoder.getPosition() <= Constants.kClimberUp + 10) {
                mClimber.setReference(0, ControlType.kSmartMotion);
            }    
            
        //} else if (getClimberLimitSwitchTop() == true){
        //    setClimberWantedState(ClimberWantedState.HOLD);
        //}

        switch (mClimberWantedState) {
            case INITIATE:

                return ClimberSystemState.INITIATING;
            case EXTEND:
    
                return ClimberSystemState.EXTENDING;
            case HOLD:
    
                return ClimberSystemState.HOLDING;
            case RETRACT:
    
                return ClimberSystemState.RETRACTING;
            case DRIVE:
                
                return ClimberSystemState.DRIVING;
            case RESET:

                return ClimberSystemState.RESETING;
            default:
    
                return ClimberSystemState.HOLDING;
        }
    }
    
    private ClimberSystemState handleReseting(double timeInState) {
        //mLeftClimberMotor.set(Constants.kClimberHomePosition);
        //if (getClimberLimitSwitchTop() == false) {
        setClimberExtend();

        SmartDashboard.putNumber("Neo encoder position: ", mClimberEncoder.getPosition());
        SmartDashboard.putNumber("Neo speed: ", mClimberEncoder.getVelocity());
            
        //} else if (getClimberLimitSwitchTop() == true){
        //    setClimberWantedState(ClimberWantedState.HOLD);
        //}

        switch (mClimberWantedState) {
            case INITIATE:

                return ClimberSystemState.INITIATING;
            case EXTEND:
    
                return ClimberSystemState.EXTENDING;
            case HOLD:
    
                return ClimberSystemState.HOLDING;
            case RETRACT:
    
                return ClimberSystemState.RETRACTING;
            case DRIVE:
                
                return ClimberSystemState.DRIVING;
            case RESET:

                return ClimberSystemState.RESETING;
            default:
    
                return ClimberSystemState.HOLDING;
        }
    }

    private ClimberSystemState handleDriving(double timeInState) {
        mClimber.setReference(0, ControlType.kDutyCycle);
        setClimberDrive();
        //setClimberExtend();
        
        SmartDashboard.putNumber("Neo encoder position: ", mClimberEncoder.getPosition());
        SmartDashboard.putNumber("Neo speed: ", mClimberEncoder.getVelocity());

        switch (mClimberWantedState) {
            case INITIATE:

                return ClimberSystemState.INITIATING;
            case EXTEND:
    
                return ClimberSystemState.EXTENDING;
            case HOLD:
    
                return ClimberSystemState.HOLDING;
            case RETRACT:
    
                return ClimberSystemState.RETRACTING;
            case DRIVE:
                
                return ClimberSystemState.DRIVING;
            case RESET:

                return ClimberSystemState.RESETING;
            default:
    
                return ClimberSystemState.HOLDING;
        }
    }

    public void setClimberExtend() {
        double left = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.LEFT_TRIGGER), Constants.kTriggerDeadband);

        double right = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.RIGHT_TRIGGER), Constants.kTriggerDeadband);

        mClimber.setReference(right - left, ControlType.kDutyCycle);
    }

    public void setClimberDrive() {
        double right = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.RIGHT_TRIGGER), Constants.kTriggerDeadband);

        mClimberDriveMotor.set(ControlMode.PercentOutput, -right);

        SmartDashboard.putNumber("Wheels: ", right);
    }

    public void setClimberInitiate() {
        double left = QuickMaths.normalizeJoystickWithDeadband(operatorJoystick.getRawAxis(Constants.LEFT_TRIGGER), Constants.kTriggerDeadband);
        mClimber.setReference(left, ControlType.kDutyCycle);
    }

    public boolean getClimberLimitSwitchBottom() {
        return mClimberLimitSwitchBottom.get();
    }

    public boolean getClimberLimitSwitchTop() {
        return mClimberLimitSwitchTop.get();
    }

    public synchronized void setClimberWantedState(ClimberWantedState wanted) {
        mClimberWantedState = wanted;
    }

    public synchronized void reset() {
        mClimberWantedState = ClimberWantedState.HOLD;
        mClimberSystemState = ClimberSystemState.HOLDING;
    }

    public void outputToSmartDashboard(){
        //nothing
    }

    public void setBrakeMode(boolean brakeMode) {
		if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();
			//mLeftClimberMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			//mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();                                         
		}
    }
    
	public void subsystemHome() {        
        mClimberEncoder.setPosition(0);
	}

    public String getClimberSystemState() {
        
        switch (mClimberSystemState) {
            case HOLDING:
                return "HOLDING";
            case EXTENDING:
                return "EXTENDING";
            case RETRACTING: 
                return "RETRACTING";
            case DRIVING:
                return "DRIVING";
            case INITIATING:
                return "INITIATING";
            case RESETING:
                return "RESETING";
            default: 
                return "UNKNOWN";
        }
    }

}