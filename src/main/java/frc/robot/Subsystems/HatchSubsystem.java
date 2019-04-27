package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
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

public class HatchSubsystem extends Subsystem {
    private static HatchSubsystem mInstance;
    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final DoubleSolenoid mHatchSolenoid;
    private CustomTalonSRX mHatchMotor;
    private HatchWantedState mHatchWantedState;
    private HatchSystemState mHatchSystemState;
    //private double mThresholdStart;
    private final DoubleSolenoid mHatchMechSolenoid;

    public static boolean kHatchEject = false;
    public static boolean kHatchRetract = !kHatchEject;
    public static boolean kHatchUp = false;
    public static boolean kHatchDown = !kHatchUp;

    private boolean kHatchAcquired = false;
    private boolean kHatchVoltageLimit = false;

    private boolean mPrevBrakeModeVal;
    private boolean mHomeSuccess;

    public static HatchSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new HatchSubsystem();
        }
        return mInstance;
    }

    public enum HatchWantedState {
        HOME,
        ACQUIRE,    //default
        HOLD,
        SUPERHOLD,
        DETECT
    }

    private enum HatchSystemState {
        HOMING,
        ACQUIRING,// gripper fully closed
        HOLDING, // gripper fully seperated
        SUPERHOLDING,
        DETECTING
    }

    private HatchSubsystem() {
        Controllers robotControllers = Controllers.getInstance();
        mHatchMotor = robotControllers.getHatchMotor();
        mHatchSolenoid = robotControllers.getHatchSolenoid();
        mHatchMechSolenoid = robotControllers.getHatchMechSolenoid();

        mPrevBrakeModeVal = false;
		setBrakeMode(true);
    }

    public void init(){
        mHatchMotor.setSensorPhase(false);
		mHatchMotor.setInverted(true);
        setBrakeMode(true);
        mHomeSuccess = false;
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mHatchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mHatchMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mHatchMotor.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;
            setSucceeded &= mHatchMotor.configClosedLoopPeakOutput(0, 0.4) == ErrorCode.OK;
            
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mHatchMotor, 0, Constants.kHatchPositionKp, Constants.kHatchPositionKi, Constants.kHatchPositionKd, Constants.kHatchPositionKf, Constants.kHatchPositionRampRate, Constants.kHatchPositionIZone);
		setSucceeded &= TalonHelper.setPIDGains(mHatchMotor, 1, Constants.kHatchPositionKp, Constants.kHatchPositionKi, Constants.kHatchPositionKd, Constants.kHatchPositionKf, Constants.kHatchPositionRampRate, Constants.kHatchPositionIZone);
        mHatchMotor.selectProfileSlot(0, 0);
        zeroSensors();
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Hatch Subsystem State", getHatchSystemState());
        SmartDashboard.putNumber("Hatch Encoder Value", mHatchMotor.getSelectedSensorPosition());
    } 

    @Override
    public void stop() {
        setHatchWantedState(HatchWantedState.HOLD);
    }

    @Override
    public void zeroSensors() {
        boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mHatchMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount); 
    }

    private final Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;


        @Override
        public void onFirstStart(double timestamp) {
            synchronized (HatchSubsystem.this) {
                mHatchSystemState = HatchSystemState.HOMING;
                mHatchWantedState = HatchWantedState.HOME;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (HatchSubsystem.this) {
                mHatchSystemState = HatchSystemState.HOMING;
                mHatchWantedState = HatchWantedState.HOME;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (HatchSubsystem.this) {
                HatchSystemState newState = mHatchSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                
                outputToSmartDashboard();

                switch (mHatchSystemState) {
                case ACQUIRING:
                    newState = handleAcquiring(timeInState);
                    break;
                case HOLDING:
                    newState = handleHolding(timeInState);
                    break;
                case HOMING:
                    newState = handleHoming(timeInState);
                    break;
                case SUPERHOLDING:
                    newState = handleSuperHolding(timeInState);
                    break;
                case DETECTING:
                    newState = handleDetecting(timeInState);
                    break;
                default:
                    System.out.println("Unexpected hatch system state: " + mHatchSystemState);
                    newState = mHatchSystemState;
                    break;
                }

                if (newState != mHatchSystemState) {
                    System.out.println(timestamp + ": Changed hatch state: " + mHatchSystemState + " -> " + newState);
                    mHatchSystemState = newState;
                    mCurrentStateStartTime = Timer.getFPGATimestamp();
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            mHatchWantedState = HatchWantedState.HOLD;
            mHatchSystemState = HatchSystemState.HOLDING;
            // Set the states to what the robot falls into when disabled.
            stop();
        }
    };
    
    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    private HatchSystemState handleAcquiring(double timeInState) {
        mHatchMotor.set(ControlMode.Position, Constants.kHatchAcquiringPosition);
        
        if (timeInState > Constants.kHatchEjectTime && kHatchEject == true) {
            setRetract();
        }
	    if (timeInState > Constants.kHatchEjectTime && mHatchMotor.getSelectedSensorPosition() > Constants.kHatchAcquiringPosition - 20) {
	        setHatchWantedState(HatchWantedState.DETECT);
	    }
        
        switch (mHatchWantedState) {
        case DETECT:
            return HatchSystemState.DETECTING;
        case SUPERHOLD:
            mHatchMotor.configClosedLoopPeakOutput(0, Constants.kHatchVoltageUnlimited);
            return HatchSystemState.SUPERHOLDING;
        case HOME:
            return HatchSystemState.HOMING;
        default:
            return HatchSystemState.ACQUIRING;
        }
    }

    private HatchSystemState handleDetecting(double timeInState) {
        mHatchMotor.set(ControlMode.PercentOutput, 0);
        
        if (mHatchMotor.getSelectedSensorPosition()<=Constants.kHatchAutoHoldingPosition){
            System.out.println("Hatch Detected");
            setHatchWantedState(HatchWantedState.SUPERHOLD);
        }
        
        switch (mHatchWantedState) {
        case SUPERHOLD:
            mHatchMotor.configClosedLoopPeakOutput(0, Constants.kHatchVoltageUnlimited);
            return HatchSystemState.SUPERHOLDING;
        case ACQUIRE:
            return HatchSystemState.ACQUIRING;
        case HOME:
            return HatchSystemState.HOMING;
        default:
            return HatchSystemState.DETECTING;
        }
    }

    private HatchSystemState handleHoming(double timeInState) {

        if (timeInState < Constants.kHatchHomeTime && mHomeSuccess == false) {
            subsystemHome(timeInState);
        } else if (mHomeSuccess == true) {
            setHatchWantedState(HatchWantedState.HOLD);
        } else {
            mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
        }
        
        switch (mHatchWantedState) {
        case HOME:
            return HatchSystemState.HOMING;
        case ACQUIRE:
            return HatchSystemState.ACQUIRING;
        case HOLD:
            return HatchSystemState.HOLDING;
        case SUPERHOLD:
            mHatchMotor.configClosedLoopPeakOutput(0, Constants.kHatchVoltageUnlimited);
            return HatchSystemState.SUPERHOLDING;
        default:
            return HatchSystemState.HOMING;
        }
    }

    private HatchSystemState handleHolding(double timeInState) {        
        //mHomeSuccess = false;
        mHomeSuccess = false;

        mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
        
           
        switch (mHatchWantedState) {
        case HOLD:
            return HatchSystemState.HOLDING;
        case ACQUIRE:
            setEject();
            return HatchSystemState.ACQUIRING;
        case HOME:
            return HatchSystemState.HOMING;
        default:
            return HatchSystemState.HOLDING;
        }
    }

    private HatchSystemState handleSuperHolding(double timeInState) {
        
        mHatchMotor.set(ControlMode.PercentOutput, Constants.kHatchSuperHold);

        if (timeInState > Constants.kHatchHoldTime) {
            setHatchWantedState(HatchWantedState.HOLD);
        }

        switch (mHatchWantedState) {
            case HOLD:
                mHatchMotor.configClosedLoopPeakOutput(0, Constants.kHatchVoltageLimit);
                return HatchSystemState.HOLDING;
            case ACQUIRE:
                mHatchMotor.configClosedLoopPeakOutput(0, Constants.kHatchVoltageLimit);
                return HatchSystemState.ACQUIRING;
            default:            
                return HatchSystemState.SUPERHOLDING;
            }
    }

    private void setEject() {
        kHatchEject = true;
        kHatchRetract = !kHatchEject;
        mHatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    private void setRetract() {
        kHatchEject = false;
        kHatchRetract = !kHatchEject;
        mHatchSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    private void setNeutral() {
        kHatchEject = false;
        kHatchRetract = false;
        mHatchSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void setHatchUp() {
        kHatchUp = true;
        kHatchDown = !kHatchUp;
        mHatchMechSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setHatchDown() {
        kHatchUp = false;
        kHatchDown = !kHatchUp;
        mHatchMechSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public synchronized void setHatchWantedState(HatchWantedState wanted) {
        mHatchWantedState = wanted;
    }

    public synchronized void reset() {
        mHatchWantedState = HatchWantedState.HOLD;
        mHatchSystemState = HatchSystemState.HOLDING;
    }

    public void setBrakeMode(boolean brakeMode) {
		if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();
			mHatchMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();                                         
		}
    }
    
	public void subsystemHome(double timeInState) {        
        mHatchMotor.set(ControlMode.PercentOutput, Constants.kHatchHomingPercentage);

        if (mHatchMotor.getSelectedSensorVelocity() == 0 && timeInState > 0.2) {
            zeroSensors();
            mHatchMotor.set(ControlMode.PercentOutput, 0);
            mHomeSuccess = true;
        } else {
            mHomeSuccess = false;
        }
	}

    public String getHatchSystemState() {
        
        switch (mHatchSystemState) {
            case HOLDING:
                return "HOLDING";
            case ACQUIRING:
                return "ACQUIRING";
            case HOMING: 
                return "HOMING";
            case SUPERHOLDING:
                return "SUPERHOLDING";
            default: 
                return "UNKNOWN";
        }
    }
}
