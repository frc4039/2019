package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Solenoid;
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
import frc.robot.Utilities.Drivers.CustomJoystick;
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

public class CargoSubsystem extends Subsystem {

    private static CargoSubsystem mInstance;

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final Solenoid mIntakeSolenoid;
    private CustomTalonSRX mCargoIntakeMotor;
    private CustomTalonSRX mCargoShooterMotor;

    private WantedState mWantedState;
    private SystemState mSystemState;
    
	private CustomJoystick operatorJoystick;

    private boolean mPrevBrakeModeVal;
    public static boolean kIntakeOut = false;
    public static boolean kIntakeIn = !kIntakeOut;

    public static CargoSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new CargoSubsystem();
        }
        return mInstance;
    }

    public enum WantedState {
        INTAKE,
        HOLD,
        WINDUP,
        SHOOT
    }

    private enum SystemState {
        INTAKING,   //Intake out, intake spinning
        HOLDING,    //Intake in, not spinning
        WINDINGUP,  //Intake in, shooter spinning
        SHOOTING    //Intake in, intake spinning reverse, shooter spinning
    }

    private CargoSubsystem() {
        
        mIntakeSolenoid = Constants.makeSolenoidForId(Constants.kCargoIntakeSolenoid);

        Controllers robotControllers = Controllers.getInstance();
        mCargoIntakeMotor = robotControllers.getCargoIntakeMotor();
        mCargoShooterMotor = robotControllers.getCargoShooterMotor();

        operatorJoystick = robotControllers.getOperatorJoystick();

        mPrevBrakeModeVal = false;
		setBrakeMode(true);
    }

    public void init(){

        mCargoIntakeMotor.setInverted(false);

        mCargoShooterMotor.setInverted(false);

        setBrakeMode(false);
		//mCargoShooterMotor.setNeutralMode(NeutralMode.Coast);

    }


    //Just leaving this here in case it helps with shuffleboard
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
        
        /* boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mHatchMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);  */

    }

    private final Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;

        @Override
        public void onFirstStart(double timestamp) {
            synchronized (CargoSubsystem.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (CargoSubsystem.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (CargoSubsystem.this) {
                SystemState newState = mSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;

                switch (mSystemState) {
                case INTAKING:
                    newState = handleIntaking(timeInState);
                    break;
                case HOLDING:
                    newState = handleHolding(timeInState);
                    break;
                case SHOOTING:
                    newState = handleShooting(timeInState);
                    break;
                case WINDINGUP:
                    newState = handleWindingUp(timeInState);
                    break;
                default:
                    System.out.println("Unexpected cargo system state: " + mSystemState);
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

    private SystemState handleIntaking(double timeInState) {
        
        mCargoIntakeMotor.set(ControlMode.PercentOutput, Constants.kCargoIntakingSpeed);
        
        switch (mWantedState) {
        case INTAKE:

            return SystemState.INTAKING;
        default:

            setIntakeIn();
            mCargoIntakeMotor.set(ControlMode.PercentOutput, 0);
            return SystemState.HOLDING;
        }
    }

    private SystemState handleHolding(double timeInState) {

        setCargoPositionOpenLoop();
        
        switch (mWantedState) {
        case HOLD:
            
            return SystemState.HOLDING;
        case INTAKE:
          
            setIntakeOut();
            return SystemState.INTAKING;
        default:
            
            return SystemState.WINDINGUP;
        }
    }

    private SystemState handleWindingUp(double timeInState) {
        
        mCargoShooterMotor.set(ControlMode.PercentOutput, Constants.kCargoShootingSpeed);
        setCargoPositionOpenLoop();
           
        switch (mWantedState) {
        case WINDUP:
            
            return SystemState.WINDINGUP;
        case INTAKE:
            
            setIntakeOut();
            mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            return SystemState.INTAKING;
        case HOLD:
        
            return SystemState.HOLDING;
        default:            

            return SystemState.SHOOTING;
        }
    }

    private SystemState handleShooting(double timeInState) {
        
        mCargoShooterMotor.set(ControlMode.PercentOutput, Constants.kCargoShootingSpeed);
        mCargoIntakeMotor.set(ControlMode.PercentOutput, Constants.kCargoFeedingSpeed);
           
        switch (mWantedState) {
        case SHOOT:
            
            return SystemState.SHOOTING;
        case INTAKE:
            
            setIntakeOut();
            mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            return SystemState.INTAKING;
        case HOLD:
        
            mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            return SystemState.HOLDING;
        default:            

            return SystemState.SHOOTING;
        }
    }

    private void setIntakeOut() {
        kIntakeOut = true;
        mIntakeSolenoid.set(kIntakeOut);
    }

    private void setIntakeIn() {
        kIntakeOut = false;
        mIntakeSolenoid.set(kIntakeIn);
    }

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
            mCargoIntakeMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mCargoShooterMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();
		}
    }

    public void setCargoPositionOpenLoop() {
        //There's probably a better way to do this, but I'm not sure what it is yet...
        //We can figure it out later...
        //these constants/whatever don't exist yet, so need to be added for the Xbox controller
        double y = QuickMaths.normalizeJoystickWithDeadband(-operatorJoystick.getRawAxis(Constants.OPERATOR_Y_AXIS), Constants.kJoystickDeadband);

        //if you chose a different name, this needs to be changed, that's why it's red
        mCargoIntakeMotor.set(ControlMode.PercentOutput, y);

    }

    //this might help with a better way to do the above, so I'm leaving it in, just commented out
    /* public void setOpenLoop(double percent) {
        mHatchMotor.set(ControlMode.PercentOutput, percent);
    } */

}