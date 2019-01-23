package frc.robot.Subsystems;

//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.*;


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

    //private final Solenoid mWristSolenoid;
    private BaseMotorController mHatchMotor;
    private WantedState mWantedState;
    private SystemState mSystemState;
    //private double mThresholdStart;

    private HatchSubsystem() {
        //mWristSolenoid = Constants.makeSolenoidForId(Constants.kGearWristSolenoid);
        //mHatchGripper.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 15);
        //mHatchGripper.changeControlMode(CANTalon.TalonControlMode.Voltage);

        Controllers robotControllers = Controllers.getInstance();
        
        mHatchMotor = robotControllers.getHatchMotor();
    }

    public void init(){
        //initialize here
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

        //code to hold gripper in acquiring position 
        mHatchMotor.set(ControlMode.Position, Constants.kHatchAcquiringPosition);
        switch (mWantedState) {
        case ACQUIRE:
            //code to keep gripper in acquiring position
            //this probably isn't necessary, can likely just set new systemstate
            mHatchMotor.set(ControlMode.Position, Constants.kHatchAcquiringPosition);
            return SystemState.ACQUIRING;
        default:
            //code to move gripper to holding
            mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
            return SystemState.HOLDING;
        }
    }

    private SystemState handleHolding(double timeInState) {

        mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
       
        switch (mWantedState) {
        case HOLD:
            //code to keep gripper in holding position
            //this probably isn't necessary, can likely just set new systemstate
            mHatchMotor.set(ControlMode.Position, Constants.kHatchHoldingPosition);
            
            return SystemState.HOLDING;

        default:
            
            //code to eject gear, then move to acquiring position
            if (timeInState < Constants.kHatchEjectTime) {
                mHatchMotor.set(ControlMode.Position, Constants.kHatchEjectPosition);
            } else {
                mHatchMotor.set(ControlMode.Position, Constants.kHatchAcquiringPosition);
            }

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


    //this will only work if we use a TalonSRX, Victors can't monitor current
/*     public boolean checkSystem() {
        System.out.println("Testing Hatch Motor.--------------------------------");
        final double kCurrentThres = 0.5;

        mHatchMotor.set(ControlMode.VoltageOutput, -6.0f);
        Timer.delay(4.0);
        //final double current = mHatchMotor.getOutputCurrent(); //doesn't work on Victors?
        mHatchMotor.set(ControlMode.VoltageOutput, 0.0);

        System.out.println("Hatch Motor Current: " + current);

        if (current < kCurrentThres) {
            System.out.println("!!!!!!!!!!!! Hatch Motor Current Low !!!!!!!!!!!");
            return false;
        }
        return true;
    } */

}