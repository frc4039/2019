package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

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

    //public static boolean kWristDown = false;
    //public static boolean kWristUp = !kWristDown;

    private static HatchSubsystem mInstance;

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final DoubleSolenoid mHatchSolenoid;
    //private final Solenoid mWristSolenoid;
    private CustomTalonSRX mHatchMotor;
    private HatchWantedState mWantedState;
    private HatchSystemState mSystemState;
    //private double mThresholdStart;

    public static boolean kHatchEject = false;
    public static boolean kHatchRetract = !kHatchEject;

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
        HOLD
    }

    private enum HatchSystemState {
        HOMING,
        ACQUIRING,// gripper fully closed
        HOLDING // gripper fully seperated
    }

    private HatchSubsystem() {
        //mWristSolenoid = Constants.makeSolenoidForId(Constants.kGearWristSolenoid);
        //mHatchGripper.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        //mHatchGripper.changeControlMode(CANTalon.TalonControlMode.Voltage);

        Controllers robotControllers = Controllers.getInstance();
        mHatchMotor = robotControllers.getHatchMotor();
        mHatchSolenoid = robotControllers.getHatchSolenoid();

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

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mHatchMotor, 0, Constants.kHatchPositionKp, Constants.kHatchPositionKi, Constants.kHatchPositionKd, Constants.kHatchPositionKf, Constants.kHatchPositionRampRate, Constants.kHatchPositionIZone);
		setSucceeded &= TalonHelper.setPIDGains(mHatchMotor, 1, Constants.kHatchPositionKp, Constants.kHatchPositionKi, Constants.kHatchPositionKd, Constants.kHatchPositionKf, Constants.kHatchPositionRampRate, Constants.kHatchPositionIZone);
		
        mHatchMotor.selectProfileSlot(0, 0);
        
        zeroSensors();
    }

    /* @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Hatch Gripper Current", mHatchGripper.getOutputCurrent());
    } */

    @Override
    public void stop() {
        setWantedState(HatchWantedState.HOLD);
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
                mSystemState = HatchSystemState.HOMING;
                mWantedState = HatchWantedState.HOME;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (HatchSubsystem.this) {
                mSystemState = HatchSystemState.HOMING;
                mWantedState = HatchWantedState.HOME;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (HatchSubsystem.this) {
                HatchSystemState newState = mSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                
                switch (mSystemState) {
                case ACQUIRING:
                    newState = handleAcquiring(timeInState);
                    break;
                case HOLDING:
                    newState = handleHolding(timeInState);
                    break;
                case HOMING:
                    newState = handleHoming(timeInState);
                    break;
                default:
                    System.out.println("Unexpected hatch system state: " + mSystemState);
                    newState = mSystemState;
                    break;
                }

                if (newState != mSystemState) {
                    System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = Timer.getFPGATimestamp();
                }
            }

        }

        @Override
        public void onStop(double timestamp) {
            mWantedState = HatchWantedState.HOLD;
            mSystemState = HatchSystemState.HOLDING;
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
        
        switch (mWantedState) {
        case ACQUIRE:

            return HatchSystemState.ACQUIRING;
        case HOLD:

            return HatchSystemState.HOLDING;
        default:

            return HatchSystemState.HOMING;
        }
    }

    private HatchSystemState handleHoming(double timeInState) {

        if (timeInState < Constants.kHatchHomeTime && mHomeSuccess == false) {
            subsystemHome(timeInState);
        } else if (mHomeSuccess == true) {
            setWantedState(HatchWantedState.HOLD);
        } else {
            mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
        }
        
        switch (mWantedState) {
        case HOME:

            return HatchSystemState.HOMING;
        case ACQUIRE:

            return HatchSystemState.ACQUIRING;
        default:

            return HatchSystemState.HOLDING;
        }
    }

    private HatchSystemState handleHolding(double timeInState) {
        

        //mHomeSuccess = false;
        mHomeSuccess = false;
        mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
           
        switch (mWantedState) {
        case HOLD:
            
            return HatchSystemState.HOLDING;
        case ACQUIRE:

            setEject();
            return HatchSystemState.ACQUIRING;
        default:            

            return HatchSystemState.HOMING;
        }
    }

    //private boolean mWristUp = false;

    /* public void setOpenLoop(double position) {
        mHatchMotor.set(ControlMode.Position, position);
    } */

    /* private void setWristUp() {
        mWristUp = true;
        mWristSolenoid.set(mWristUp);
    } */

    /* private void setWristDown() {
        mWristUp = false;
        mWristSolenoid.set(mWristUp);
    } */

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

    public synchronized void setWantedState(HatchWantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void reset() {
        mWantedState = HatchWantedState.HOLD;
        mSystemState = HatchSystemState.HOLDING;
    }

    public void outputToSmartDashboard(){
        //nothing
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

        mHatchMotor.set(ControlMode.PercentOutput, -0.30);
        if (mHatchMotor.getSelectedSensorVelocity() == 0 && timeInState > 0.2) {
            zeroSensors();
            mHatchMotor.set(ControlMode.PercentOutput, 0);
            mHomeSuccess = true;
        } else {
            mHomeSuccess = false;
        }
        

	}

    public String getSystemState() {
        
        switch (mSystemState) {
            case HOLDING:
                return "HOLDING";
            case ACQUIRING:
                return "ACQUIRING";
            case HOMING: 
                return "HOMING";
            default: 
                return "UNKNOWN";
        }
    }
    

}