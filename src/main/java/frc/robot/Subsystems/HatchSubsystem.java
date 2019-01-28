package frc.robot.Subsystems;

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

    //private final Solenoid mWristSolenoid;
    private CustomTalonSRX mHatchMotor;
    private WantedState mWantedState;
    private SystemState mSystemState;
    //private double mThresholdStart;

    private boolean mPrevBrakeModeVal;
    private boolean mHomeSuccess;

    public static HatchSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new HatchSubsystem();
        }
        return mInstance;
    }

    public enum WantedState {
        ACQUIRE,    //default
        HOLD
    }

    private enum SystemState {
        ACQUIRING,// gripper fully closed
        HOLDING // gripper fully seperated
    }

    private HatchSubsystem() {
        //mWristSolenoid = Constants.makeSolenoidForId(Constants.kGearWristSolenoid);
        //mHatchGripper.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        //mHatchGripper.changeControlMode(CANTalon.TalonControlMode.Voltage);

        Controllers robotControllers = Controllers.getInstance();
        mHatchMotor = robotControllers.getHatchMotor();

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
        setWantedState(WantedState.HOLD);
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
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (HatchSubsystem.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (HatchSubsystem.this) {
                SystemState newState = mSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                
                switch (mSystemState) {
                case ACQUIRING:
                    newState = handleAcquiring(timeInState);
                    break;
                case HOLDING:
                    newState = handleHolding(timeInState);
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
            mWantedState = WantedState.HOLD;
            mSystemState = SystemState.HOLDING;
            // Set the states to what the robot falls into when disabled.
            stop();
        }
    };
    
    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    private SystemState handleAcquiring(double timeInState) {
        System.out.println("acquiring");

        //code to hold gripper in acquiring position 
        //if (timeInState < Constants.kHatchEjectTime && mHomeSuccess == false) {
        //   subsystemHome();
        //} else {
            mHatchMotor.set(ControlMode.Position, Constants.kHatchAcquiringPosition);
        //}

        switch (mWantedState) {
        case ACQUIRE:

            return SystemState.ACQUIRING;
        default:

            return SystemState.HOLDING;
        }
    }

    private SystemState handleHolding(double timeInState) {

        System.out.println("holding");
        mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
        // if (timeInState < Constants.kHatchEjectTime) {
           
        switch (mWantedState) {
        case HOLD:
            
            return SystemState.HOLDING;

        default:            

            return SystemState.ACQUIRING;
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

    public synchronized void setWantedState(WantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void reset() {
        mWantedState = WantedState.HOLD;
        mSystemState = SystemState.HOLDING;
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
    
	public void subsystemHome() {
        System.out.println("entered subsystemHome");
        mHomeSuccess = false;

        mHatchMotor.set(ControlMode.PercentOutput, 0.05);
        if (mHatchMotor.getOutputCurrent() > 5) {
            System.out.println("Current greater than 5");
            zeroSensors();
            mHomeSuccess = true;
        }
        

//		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
//			ConsoleReporter.report("Failed to zero DriveBaseSubsystem!!!", MessageLevel.DEFCON1);
	}

    //this will only work if we use a TalonSRX, Victors can't monitor current
     /* public boolean checkSystem() {
        System.out.println("Testing Hatch Motor.--------------------------------");
        final double kCurrentThres = 0.5;

        mHatchMotor.set(ControlMode.VoltageOutput, -6.0f);
        Timer.delay(4.0);
        final double current = mHatchMotor.getOutputCurrent();
        mHatchMotor.set(ControlMode.VoltageOutput, 0.0);

        System.out.println("Hatch Motor Current: " + current);

        if (current < kCurrentThres) {
            System.out.println("!!!!!!!!!!!! Hatch Motor Current Low !!!!!!!!!!!");
            return false;
        }
        return true;
    }   */

}