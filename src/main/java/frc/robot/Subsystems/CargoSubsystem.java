package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.*;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Utilities.Drivers.TalonHelper;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Drivers.CustomJoystick;
import frc.robot.Utilities.Drivers.CustomTalonSRX;
import frc.robot.Utilities.*;

import java.util.concurrent.locks.ReentrantLock;

public class CargoSubsystem extends Subsystem {

    public static boolean kIntakeDown = false;
    public static boolean kIntakeUp = !kIntakeDown;

    private static CargoSubsystem mInstance;

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final DoubleSolenoid mCargoIntakeSolenoid;
    private VictorSPX mCargoIntakeMotor;
    private CustomTalonSRX mCargoShooterMotor;

    private CargoWantedState mCargoWantedState;
    private CargoSystemState mCargoSystemState;

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
        INTAKE,
        HOLD,
        WINDUP,
        SHOOT,
        PUSH
    }

    private enum CargoSystemState {
        INTAKING, // intake out, rollers spinning
        HOLDING, // intake in, rollers spinning at a low % or not at all
        WINDINGUP, // intake in, shooter rollers spining
        SHOOTING, // runs upper and lower rollers
        PUSHING
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
		mCargoShooterMotor.setInverted(true);

        setBrakeMode(false);

        boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mCargoShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mCargoShooterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mCargoShooterMotor.configVelocityMeasurementWindow(32, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mCargoShooterMotor, 0, Constants.kCargoShooterVelocityKp, Constants.kCargoShooterVelocityKi, Constants.kCargoShooterVelocityKd, Constants.kCargoShooterVelocityKf, Constants.kCargoShooterVelocityRampRate, Constants.kCargoShooterVelocityIZone);
		
		mCargoShooterMotor.selectProfileSlot(0, 0);
    }


    //Just leaving this here in case it helps with shuffleboard
    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Cargo Subsystem State", getCargoSystemState());
    }

    @Override
    public void stop() {
        setCargoWantedState(CargoWantedState.HOLD);
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
                mCargoSystemState = CargoSystemState.HOLDING;
                mCargoWantedState = CargoWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (CargoSubsystem.this) {
                mCargoSystemState = CargoSystemState.HOLDING;
                mCargoWantedState = CargoWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (CargoSubsystem.this) {
                CargoSystemState newState = mCargoSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                
                switch (mCargoSystemState) {
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
                case PUSHING:
                    newState = handlePushing(timeInState);
                    break;
                default:
                    System.out.println("Unexpected cargo system state: " + mCargoSystemState);
                    newState = mCargoSystemState;
                    break;
                }

                if (newState != mCargoSystemState) {
                    System.out.println(timestamp + ": Changed state: " + mCargoSystemState + " -> " + newState);
                    mCargoSystemState = newState;
                    mCurrentStateStartTime = Timer.getFPGATimestamp();
                }
            }

        }

        @Override
        public void onStop(double timestamp) {
            mCargoWantedState = CargoWantedState.HOLD;
            mCargoSystemState = CargoSystemState.HOLDING;
            // Set the states to what the robot falls into when disabled.
            stop();
        }
    };

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    protected CargoSystemState handleIntaking(double timeInState) {
        mCargoIntakeMotor.set(ControlMode.PercentOutput, Constants.kCargoIntakingSpeed);
        
        switch (mCargoWantedState) {
            case INTAKE:
                outputToSmartDashboard();
                return CargoSystemState.INTAKING;
            case HOLD:
                setIntakeUp();
                mCargoIntakeMotor.set(ControlMode.PercentOutput, 0);
                return CargoSystemState.HOLDING;
            case PUSH:
                setIntakeOut();
                return CargoSystemState.PUSHING;
            default:
                return CargoSystemState.INTAKING;
        }
  }

    private CargoSystemState handlePushing(double timeInState) {
        mCargoIntakeMotor.set(ControlMode.PercentOutput, -Constants.kCargoIntakingSpeed);

        switch (mCargoWantedState) {
        case HOLD:
            setIntakeUp();
            mCargoIntakeMotor.set(ControlMode.PercentOutput, 0);
            return CargoSystemState.HOLDING;
        case INTAKE:
            return CargoSystemState.INTAKING;
        default:
            return CargoSystemState.PUSHING;
        }
    }

    private CargoSystemState handleHolding(double timeInState) {
        setCargoPositionOpenLoop();

        switch (mCargoWantedState) {
        case HOLD:
            return CargoSystemState.HOLDING;
        case INTAKE:
            setIntakeOut();
            return CargoSystemState.INTAKING;
        case WINDUP:
            return CargoSystemState.WINDINGUP;
        case PUSH:
            setIntakeOut();
            return CargoSystemState.PUSHING;
        default:
            return CargoSystemState.HOLDING;
        }
    }

    private CargoSystemState handleWindingUp(double timeInState)
    {
        mCargoShooterMotor.set(ControlMode.Velocity, Constants.kCargoShootingVelocity);
        setCargoPositionOpenLoop();


        switch (mCargoWantedState) {
            case WINDUP:
                return CargoSystemState.WINDINGUP;
            case INTAKE:
                setIntakeOut();
                mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
                return CargoSystemState.INTAKING;
            case HOLD:
                mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
                return CargoSystemState.HOLDING;
            case SHOOT:
                return CargoSystemState.SHOOTING;
            case PUSH:
                setIntakeOut();
                mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            default:
                return CargoSystemState.WINDINGUP;
        }
    }

    private CargoSystemState handleShooting(double timeInState) {

        mCargoIntakeMotor.set(ControlMode.PercentOutput, Constants.kCargoFeedingSpeed);
        mCargoShooterMotor.set(ControlMode.Velocity, Constants.kCargoShootingVelocity);
    
        switch (mCargoWantedState) {
        case SHOOT:
            return CargoSystemState.SHOOTING;
        case INTAKE:
            setIntakeOut();
            mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            return CargoSystemState.INTAKING;
        case HOLD:
            mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            mCargoIntakeMotor.set(ControlMode.PercentOutput, 0);
            return CargoSystemState.HOLDING;
        case PUSH:
            setIntakeOut();
            mCargoShooterMotor.set(ControlMode.PercentOutput, 0);
            return CargoSystemState.PUSHING;
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

    public synchronized void setCargoWantedState(CargoWantedState wanted) {
        mCargoWantedState = wanted;
    }

    public synchronized void reset() {
        mCargoWantedState = CargoWantedState.HOLD;
        mCargoSystemState = CargoSystemState.HOLDING;
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

        double y = QuickMaths.normalizeJoystickWithDeadband(-operatorJoystick.getRawAxis(Constants.OPERATOR_X_AXIS), Constants.kJoystickDeadband);

        mCargoIntakeMotor.set(ControlMode.PercentOutput, y*y*y/2); //TODO: might have to be thirded or more

    }

    //this might help with a better way to do the above, so I'm leaving it in, just commented out
    /* public void setOpenLoop(double percent) {
        mHatchMotor.set(ControlMode.PercentOutput, percent);
    } */

    public String getCargoSystemState() {
        
        switch (mCargoSystemState) {
            case INTAKING:
                return "INTAKING";
            case HOLDING:
                return "HOLDING";
            case WINDINGUP:
                return "WINDINGUP";
            case SHOOTING: 
                return "SHOOTING";
            /*case PUSHING:  
                return "PUSHING";*/
            default: 
                return "UNKNOWN";
        }
    }
}