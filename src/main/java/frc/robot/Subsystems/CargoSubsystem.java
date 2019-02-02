package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.*;

import java.util.concurrent.locks.ReentrantLock;

public class CargoSubsystem extends Subsystem {

    public static boolean kIntakeDown = false;
    public static boolean kIntakeUp = !kIntakeDown;

    private static CargoSubsystem mInstance;

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final DoubleSolenoid mCargoIntakeSolenoid;
    private VictorSP mCargoIntakeMotor;
    private VictorSP mCargoShooterMotor;

    private CargoWantedState mWantedState;
    private CargoSystemState mSystemState;

    private CustomJoystick operatorJoystick;

    private boolean mPrevBrakeModeVal;
    public static boolean kIntakeOut;
    public static boolean kIntakeIn;

    public static CargoSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new CargoSubsystem();
        }
        return mInstance;
    }

    public enum CargoWantedState {
        INTAKE, HOLD, WINDUP, SHOOT, CARGO_WINDUP
    }

    private enum CargoSystemState {
        INTAKING, // intake out, rollers spinning
        HOLDING, // intake in, rollers not spinning
        WINDINGUP, // intake in, shooter rollers spining
        SHOOTING // runs upper and lower rollers
    }

    private CargoSubsystem() {

        Controllers robotControllers = Controllers.getInstance();
        mCargoIntakeMotor = robotControllers.getCargoIntakeMotor();
        mCargoShooterMotor = robotControllers.getCargoShooterMotor();
        mCargoIntakeSolenoid = robotControllers.getCargoIntakeSolenoid();

        operatorJoystick = robotControllers.getOperatorJoystick();

        mPrevBrakeModeVal = false;
        setBrakeMode(true);
    }

    public void init() {
        mCargoIntakeMotor.setInverted(true);

		mCargoShooterMotor.setInverted(false);

        setBrakeMode(false);

    }


    //Just leaving this here in case it helps with shuffleboard
    /* @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Hatch Gripper Current", mHatchGripper.getOutputCurrent());
    } */

    @Override
    public void stop() {
        setWantedState(CargoWantedState.HOLD);
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
                mSystemState = CargoSystemState.HOLDING;
                mWantedState = CargoWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (CargoSubsystem.this) {
                mSystemState = CargoSystemState.HOLDING;
                mWantedState = CargoWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (CargoSubsystem.this) {
                CargoSystemState newState = mSystemState;
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
            mWantedState = CargoWantedState.HOLD;
            mSystemState = CargoSystemState.HOLDING;
            // Set the states to what the robot falls into when disabled.
            stop();
        }
    };

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    protected CargoSystemState handleIntaking(double timeInState) {
        mCargoIntakeMotor.set(Constants.kCargoIntakingSpeed);
        
        switch (mWantedState) {
            case INTAKE:

                setIntakeOut();
                return CargoSystemState.INTAKING;
            case HOLD:
                setIntakeUp();
                mCargoIntakeMotor.set(0);
                return CargoSystemState.HOLDING;
            default:
                return CargoSystemState.INTAKING;
        }
  }
    private CargoSystemState handleHolding(double timeInState) {

        setCargoPositionOpenLoop();

        switch (mWantedState) {
        case HOLD:

            return CargoSystemState.HOLDING;
        case INTAKE:

            setIntakeOut();
            return CargoSystemState.INTAKING;
        case WINDUP:
            
            return CargoSystemState.WINDINGUP;
        default:

            return CargoSystemState.HOLDING;
        }
    }

    private CargoSystemState handleWindingUp(double timeInState)
    {
        mCargoShooterMotor.set(Constants.kCargoShootingSpeed);
        setCargoPositionOpenLoop();

        switch (mWantedState) {
            case WINDUP:
            
                return CargoSystemState.WINDINGUP;
            case INTAKE:
            
                setIntakeOut();
                mCargoShooterMotor.set(0);
                return CargoSystemState.INTAKING;
            case HOLD:

                mCargoShooterMotor.set(0);
                return CargoSystemState.HOLDING;
            case SHOOT:

                return CargoSystemState.SHOOTING;
            default:

                return CargoSystemState.WINDINGUP;
        }
    }

    private CargoSystemState handleShooting(double timeInState) {

        mCargoIntakeMotor.set(Constants.kCargoFeedingSpeed);
        mCargoShooterMotor.set(Constants.kCargoShootingSpeed);
    
        switch (mWantedState) {
        case SHOOT:

            return CargoSystemState.SHOOTING;
        case INTAKE:

            setIntakeOut();
            mCargoShooterMotor.set(0);
            return CargoSystemState.INTAKING;
        case HOLD:

            mCargoShooterMotor.set(0);
            mCargoIntakeMotor.set(0);
            return CargoSystemState.HOLDING;
        default:

            return CargoSystemState.SHOOTING;
        }
    }

    private void setIntakeOut() {
        kIntakeOut = true;
        kIntakeIn = !kIntakeOut;
        mCargoIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    private void setIntakeUp() {
        kIntakeOut = false;
        kIntakeIn = !kIntakeOut;
        mCargoIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    private void setIntakeNeutral() {
        kIntakeOut = false;
        kIntakeIn = false;
        mCargoIntakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public synchronized void setWantedState(CargoWantedState wanted) {
        mWantedState = wanted;
    }

    public synchronized void reset() {
        mWantedState = CargoWantedState.HOLD;
        mSystemState = CargoSystemState.HOLDING;
    }

    public void outputToSmartDashboard(){
        //nothing
    }

    public void setBrakeMode(boolean brakeMode) {
		/*if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();
            mCargoIntakeMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            mCargoShooterMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();
		}*/
    }

    public void setCargoPositionOpenLoop() {

        double y = QuickMaths.normalizeJoystickWithDeadband(-operatorJoystick.getRawAxis(Constants.OPERATOR_Y_AXIS), Constants.kJoystickDeadband);

        mCargoIntakeMotor.set(y);

    }

    //this might help with a better way to do the above, so I'm leaving it in, just commented out
    /* public void setOpenLoop(double percent) {
        mHatchMotor.set(ControlMode.PercentOutput, percent);
    } */
}