package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Drivers.CustomJoystick;
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
        INITIATE
    }

    private enum ClimberSystemState {
        EXTENDING,
        HOLDING, // gripper fully closed
        RETRACTING, // gripper fully seperated
        DRIVING,
        INITIATING
    }

    private ClimberSubsystem() {
        Controllers robotControllers = Controllers.getInstance();
        mLeftClimberMotor = robotControllers.getLeftClimberMotor();
        mRightClimberMotor = robotControllers.getRightClimberMotor();

        driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();
        operatorJoystick = robotControllers.getOperatorJoystick();

        mPrevBrakeModeVal = false;
		//setBrakeMode(true);
    }

    public void init(){

        mLeftClimberMotor.set(0);
        mRightClimberMotor.follow(mLeftClimberMotor, true);
        mLeftClimberMotor.setInverted(true);

        mClimber = mLeftClimberMotor.getPIDController();
        mClimberEncoder = mLeftClimberMotor.getEncoder();

        mClimber.setP(Constants.kClimberPositionkP);
        mClimber.setI(Constants.kClimberPositionkI);
        mClimber.setD(Constants.kClimberPositionkD);
        mClimber.setIZone(Constants.kClimberPositionkIz);
        mClimber.setFF(Constants.kClimberPositionkFF);
        mClimber.setOutputRange(Constants.kClimberPositionkMinOutput, Constants.kClimberPositionkMaxOutput);

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

        if (getClimberLimitSwitchTop() == true) {
            setClimberInitiate();
        } else if (getClimberLimitSwitchTop() == false) {
            setClimberWantedState(ClimberWantedState.HOLD);
        }

        switch (mClimberWantedState) {
        case HOLD:
            return ClimberSystemState.HOLDING;
        case EXTEND:
            return ClimberSystemState.EXTENDING;
        default:
            return ClimberSystemState.INITIATING;
        }
    }

    private ClimberSystemState handleExtending(double timeInState) {
        //mLeftClimberMotor.set(Constants.kClimberExtendedPosition);
        if (getClimberLimitSwitchTop() == true && getClimberLimitSwitchBottom() == false) {
            setClimberExtend();
        } else if (getClimberLimitSwitchTop() == true && getClimberLimitSwitchBottom() == true){
            setClimberWantedState(ClimberWantedState.DRIVE);
        }

        switch (mClimberWantedState) {
        case EXTEND:

            return ClimberSystemState.EXTENDING;
        case HOLD:

            return ClimberSystemState.HOLDING;
        case RETRACT:

            return ClimberSystemState.RETRACTING;
        default:

            return ClimberSystemState.HOLDING;
        }
    }

    private ClimberSystemState handleHolding(double timeInState) {        
        //mHomeSuccess = false;
        //mHomeSuccess = false;
        //mLeftClimberMotor.set(0);
           
        switch (mClimberWantedState) {
        case HOLD:
            
            return ClimberSystemState.HOLDING;
        case INITIATE:

            return ClimberSystemState.INITIATING;
        case EXTEND:
            
            return ClimberSystemState.EXTENDING;
        case RETRACT:

            return ClimberSystemState.RETRACTING;
        default:            

            return ClimberSystemState.HOLDING;
        }
    }

    private ClimberSystemState handleRetracting(double timeInState) {
        //mLeftClimberMotor.set(Constants.kClimberHomePosition);
        if (getClimberLimitSwitchTop() == true) {
            setClimberRetract();
        } else if (getClimberLimitSwitchTop() == false){
            setClimberWantedState(ClimberWantedState.HOLD);
        }

        switch (mClimberWantedState) {
        case RETRACT:

            return ClimberSystemState.RETRACTING;
        case HOLD:

            return ClimberSystemState.HOLDING;
        default:

            return ClimberSystemState.RETRACTING;
        }
    }

    private ClimberSystemState handleDriving(double timeInState) {
        //mLeftClimberMotor.set(0);

        
        switch (mClimberWantedState) {
        case HOLD:
            return ClimberSystemState.HOLDING;
        case RETRACT:
            return ClimberSystemState.RETRACTING;
        default:
            return ClimberSystemState.DRIVING;
        }
    }

    public void setClimberExtend() {
        double left = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.LEFT_TRIGGER), Constants.kTriggerDeadband);

        mClimber.setReference(left, ControlType.kVelocity);
    }

    public void setClimberRetract() {
        double right = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.RIGHT_TRIGGER), Constants.kTriggerDeadband);

        mClimber.setReference(-right, ControlType.kVelocity);
    }

    public void setClimberInitiate() {
        double left = QuickMaths.normalizeJoystickWithDeadband(-operatorJoystick.getRawAxis(Constants.LEFT_TRIGGER), Constants.kTriggerDeadband);
        System.out.println(left);
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
    
	public void subsystemHome(double timeInState) {        
        /*mLeftClimberMotor.set(Constants.kClimberHomePosition);
        mRightClimberMotor.follow(mLeftClimberMotor);

        if (mLeftClimberMotor.getSelectedSensorVelocity() == 0 && mRightClimberMotor.getSelectedSensorVelocity() == 0) {
            zeroSensors();
            mLeftClimberMotor.set(ControlMode.PercentOutput, 0);
            mRightClimberMotor.follow(mLeftClimberMotor);
            mHomeSuccess = true;
        } else {
            mHomeSuccess = false;
        }*/
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
            default: 
                return "UNKNOWN";
        }
    }

}