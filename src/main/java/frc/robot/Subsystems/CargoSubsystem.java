package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

/* import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.VictorSPX; */

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Drivers.CustomJoystick;
//import frc.robot.Utilities.Drivers.CustomTalonSRX;
//import frc.robot.Utilities.Drivers.TalonHelper;
import frc.robot.Utilities.*;

import java.util.concurrent.locks.ReentrantLock;

/**
 * The gear grabber subsystem consists of one BAG motor used to intake and
 * exhaust gears and one pancake piston used to pivot the entire subsystem from
 * a floor pickup position to a scoring position. The motor is driven in open
 * loop. Since the subsystem lacks any encoders, it detects when a gear has been
 * acquired by checking whether current is above a threshold value. The main
 * things this subsystem has to are intake gears, score gears, and clear balls
 * (run motor in reverse while the grabber is down to push balls away).
 * 
 * @see Subsystem.java
 */

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

    /////////////////////////////////////////////////////////////
    // Need to update the states to Cargo Subsystem states
    // Something like the following:
    // Intaking, Holding, Shooting
    // Even though you might not know them 100% now, you can take a guess
    // and update them as you figure it out
    /////////////////////////////////////////////////////////////

    public enum CargoWantedState {
        INTAKE, HOLD, WINDUP, SHOOT, CARGO_WINDUP
    }

    private enum CargoSystemState {
        INTAKING, // intake out, intake spinning
        HOLDING, // intake in, not spinning
        WINDINGUP, // intake in, shooter spining
        SHOOTING // rolls upper and lower intake
    }

    private CargoSubsystem() {

        ////////////////////////////////////////////////////////////////////////////////////////
        // Need to create kCargoIntake Solenoid in Constants, but VSCode should tell you
        //////////////////////////////////////////////////////////////////////////////////////// that
        ////////////////////////////////////////////////////////////////////////////////////////


        /////////////////////////////////////////////////////////////////////////////////////////////
        // Need to change "mHatchMotor" to "mCargoIntakeMotor" or whatever you had in your version
        // What you had was fine, and your code in Controllers.java was good, just need to add it
        // here
        /////////////////////////////////////////////////////////////////////////////////////////////

        Controllers robotControllers = Controllers.getInstance();
        mCargoIntakeMotor = robotControllers.getCargoIntakeMotor();
        mCargoShooterMotor = robotControllers.getCargoShooterMotor();
        mCargoIntakeSolenoid = robotControllers.getCargoIntakeSolenoid();

        operatorJoystick = robotControllers.getOperatorJoystick();

        mPrevBrakeModeVal = false;
        setBrakeMode(true);
    }

    public void init() {

        ////////////////////////////////////////////////////////////////////////////////////////
        // again, switch 'mHatchMotor's to 'mCargoIntakeMotor', etc.
        ////////////////////////////////////////////////////////////////////////////////////////

        mCargoIntakeMotor.setInverted(true);

		mCargoShooterMotor.setInverted(false);

        setBrakeMode(false);

    }


    //Just leaving this here in case it helps with shuffleboard
    /* @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Hatch Gripper Current", mHatchGripper.getOutputCurrent());
    } */


    ////////////////////////////////////////////////////////////////////////////////////////
    //update the stop() function for the CargoSubsystem
    //all you have to change is the WantedState from 'WantedState.Hold' to 'WantedState.Holding'
    //      or whatever the "end" state for the cargo subsystem would be
    //you don't have to change this now, you can come back and do it later once you have
    //      a better idea what the system states are
    ////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop() {
        setWantedState(CargoWantedState.HOLD);
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    //The subsystem class needs a zeroSensors() function, 
    //      but we have no sensors on this subsystem
    //We can leave it empty without any issues, since we're never forced to call it
    //We'll just comment out the default code so we still have it if we decide to add sensors
    ////////////////////////////////////////////////////////////////////////////////////////
    
    @Override
    public void zeroSensors() {
        
        /* boolean setSucceeded;
		int retryCounter = 0;
		do {
			setSucceeded = true;
			setSucceeded &= mHatchMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMsFast) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);  */

    }


    ////////////////////////////////////////////////////////////////////////////////////////
    //Honestly this loop stuff is kinda magic, so we won't change very much of it. 
    ////////////////////////////////////////////////////////////////////////////////////////

    private final Loop mLoop = new Loop() {
        private double mCurrentStateStartTime;


        ////////////////////////////////////////////////////////////////////////////////////////
        //The onFirstStart() function specifies what state the subsystem will enter when the 
        //      robot is turned on/enable/etc.
        //All you have to change here is the 'SystemState.HOMING' and 'WantedState.HOME' to something
        //      like 'SystemState.HOLDING' and 'WantedState.HOLD'
        ////////////////////////////////////////////////////////////////////////////////////////

        @Override
        public void onFirstStart(double timestamp) {
            synchronized (CargoSubsystem.this) {
                mSystemState = CargoSystemState.HOLDING;
                mWantedState = CargoWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }


        ////////////////////////////////////////////////////////////////////////////////////////
        //Basically the same as the 'onFirstStart()' function, so just make it exactly the same.
        //The technically do slightly different things, but it doesn't really matter for 95% of our stuff
        //      but if we leave it blank, everything breaks.
        ////////////////////////////////////////////////////////////////////////////////////////

        @Override
        public void onStart(double timestamp) {
            synchronized (CargoSubsystem.this) {
                mSystemState = CargoSystemState.HOLDING;
                mWantedState = CargoWantedState.HOLD;
            }
            mCurrentStateStartTime = Timer.getFPGATimestamp();
        }

        ////////////////////////////////////////////////////////////////////////////////////////
        //This the more important block, the important part is the switch-case section.
        //If you don't know what that is, it's basically an easier way of writing a bunch of if-else loops
        //      I can explain this better in person
        ////////////////////////////////////////////////////////////////////////////////////////

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (CargoSubsystem.this) {
                CargoSystemState newState = mSystemState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                
                ////////////////////////////////////////////////////////////////////////////////////////
                //You only have to change the cases below
                //The capitalized word is just a SystemState, the same as the ones you wrote at the top of this file.
                //So from my example ones, they would be something like: "INTAKING", "HOLDING", "SHOOTING", etc.
                //You can leave the default section the same, and the 'if' block after it the same as well.
                //
                //You'll also have to change the 'handleAcquiring(timeInState)' functions 
                //      to 'handleShooting(timeInState)' or similar (name based on system states)
                //Then you'll have to create these functions later on in the file.
                //You'll see them later, just edit the names now and suffer through the red lines
                //
                //When the looper/scheduler gets to the cargo subsystem, it runs this onLoop() function
                //The main part of onLoop is this switch-case statement, which tells the robot which function 
                //      to run based on it's current SystemState. 
                //So if the SystemState is 'HOLDING' it runs 'handleHolding()'.
                //We'll edit what those states do, and how we get from one to another later. 
                ////////////////////////////////////////////////////////////////////////////////////////

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


        ////////////////////////////////////////////////////////////////////////////////////////
        //Kind of like 'onFirstStart()' this sets the system state when the robot is disabled
        //This one also isn't super important for what we do, I would just make it hte default state
        //      for the subsystem. Maybe "HOLDING" or something. 
        //Like 'onStart()' you just have to change 'WantedState.HOLD' and 'SystemState.HOLDING'
        ////////////////////////////////////////////////////////////////////////////////////////
        @Override
        public void onStop(double timestamp) {
            mWantedState = CargoWantedState.HOLD;
            mSystemState = CargoSystemState.HOLDING;
            // Set the states to what the robot falls into when disabled.
            stop();
        }
    };

    ////////////////////////////////////////////////////////////////////////////////////////
    //This is magic, don't change it.
    //It basically just puts the subsystem into the looper/scheduler
    ////////////////////////////////////////////////////////////////////////////////////////
    
    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    protected CargoSystemState handleIntaking(double timeInState) {
        mCargoIntakeMotor.set(Constants.kCargoIntakingSpeed);
        
        switch (mWantedState) {
            case INTAKE:
                return CargoSystemState.INTAKING;
            case HOLD:
                setIntakeUp();
                mCargoIntakeMotor.set(0);
                return CargoSystemState.HOLDING;
            default:
                return CargoSystemState.INTAKING;
        }
  }

    ////////////////////////////////////////////////////////////////////////////////////////
    //This is the first of the handleWhatever functions. 
    //
    //The comments describe what to do.
    ////////////////////////////////////////////////////////////////////////////////////////
    private CargoSystemState handleHolding(double timeInState) {

        ////////////////////////////////////////////////////////////////////////////////////////
        //Code in this section is what should happen WHILE the robot IS IN this state, and runs
        //      repeatedly every 20ms. So the only code that should be run continuously should be here.
        //For the hatch it was move to the "acquiring position"
        //If this were the "handleIntake()" function, this section might tell the intake motor 
        //      to spin at a certain rate.
        //This section will probably never have pneumatics, because we don't want to fire
        //      pneumatics every 20ms 
        ////////////////////////////////////////////////////////////////////////////////////////
        
        setCargoPositionOpenLoop();

        ////////////////////////////////////////////////////////////////////////////////////////
        //This switch-case block is how the subsystem transfers to a new SystemState.
        //The OI.java file changes the WantedState based on joystick input, then when 'handleAcquiring()'
        //      is run every 20ms, if the WantedState is changed, this section will execute code, then
        //      set a new SystemState.
        //Just like before, the 'case's should be the WantedState s that you listed at the top of the file
        //The return statement changes the SystemState to match the WantedState
        //If we need to do something to transition between the states, that code would go here
        ////////////////////////////////////////////////////////////////////////////////////////

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
        ////////////////////////////////////////////////////////////////////////////////////////
        //I left the first one without comments as an example. But for this one, if we wanted
        //      to put the intake out in this step we would add a line like the one I commented out.
        //      (the 'setIntakeOut()' function is defined later)
        //You probably don't want to put any code that needs to be run for more than 20ms here 
        //      (such as a motor command, like 'move to encoder position')
        //      but pneumatic firing is perfect for this.
        ////////////////////////////////////////////////////////////////////////////////////////
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

    ////////////////////////////////////////////////////////////////////////////////////////
    //I changed this following 'handleWhatever()' function to a "handleHolding" function as 
    //      an example, since you'll have to make a few of these and completed one might be
    //      helpful
    ////////////////////////////////////////////////////////////////////////////////////////

    private CargoSystemState handleShooting(double timeInState) {

        //handleHolding is red underlined because it's not in the 'onLoop()' function, needs to be fixed

        //I made this up, and added the code for it later on in the file since it's a bit weird. 
        //I'm not sure if it actually works, but I guess we'll find out!
        //the point of it is so that the operator can change the cargo position while it's being held
        //      so that it can be more secure, and not get knocked out
        mCargoIntakeMotor.set(Constants.kCargoFeedingSpeed);
        mCargoShooterMotor.set(Constants.kCargoShootingSpeed);
    
        
        //I don't think there's anything else we'd want to do for the HOLDING systemState, but we'll
        //      find out if there is once we test it.
        //Speaking of testing, when we do test it's probably a good idea to comment out the above 
        //      'setCargoPositionOpenLoop()' function, because it's super likely it doesn't work
        //      and we don't want that to confuse us when other parts don't work. 
        //We can make it work later, once we're sure the overall structure is good.


        switch (mWantedState) {
        case SHOOT:
            //The first 'case' should be the one it's currently in, because it's most likely to be correct
            //The sooner the code finds a match, the faster it will run, and faster is better
            return CargoSystemState.SHOOTING;
        case INTAKE:

            setIntakeOut();
            mCargoShooterMotor.set(0);
            return CargoSystemState.INTAKING;
        case HOLD:
          //again, INTAKE and INTAKING don't exist, unless you added them earlier to the 
          //      SystemState and WantedState enums

          //The setIntakeOut() function doesn't exist yet, but we'll create it soon
            mCargoShooterMotor.set(0);
            mCargoIntakeMotor.set(0);
            return CargoSystemState.HOLDING;
        default:
            //We don't need to call the 'setIntakeIn()' function, because the intake should be in already
            //      since we're already in the 'HOLDING' state.
            //This is also the reason we don't call 'setIntakeIn()' in the first 'case' to return to HOLDING
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


    ///////////////////////////////////////////////////////
    //This needs to be uncommented and adapted to the cargo subsystem, instead of some
    //      other team's 2018's robot subsystems
    //It should be as simple as changing 'wristUp/Down' to 'intakeIn/Out' or whatever you choose
    //////////////////////////////////////////////////////

    //if you don't have this the code breaks. Also, it might be helpful for Iris' shuffleboard stuff
    public void outputToSmartDashboard(){
        //nothing
    }

    //this just changes the motor controller's brake/coastmode. You can leave it as is, other than changing
    //      mHatchMotor to mIntakeMotor/mShooterMotor (one for each)
    public void setBrakeMode(boolean brakeMode) {
		/*if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();
            mCargoIntakeMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
            mCargoShooterMotor.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();
		}*/
    }

// TODO: Change ^^^^
 

    public void setCargoPositionOpenLoop() {
        //There's probably a better way to do this, but I'm not sure what it is yet...
        //We can figure it out later...
        //these constants/whatever don't exist yet, so need to be added for the Xbox controller
        double y = QuickMaths.normalizeJoystickWithDeadband(-operatorJoystick.getRawAxis(Constants.OPERATOR_Y_AXIS), Constants.kJoystickDeadband);

        //if you chose a different name, this needs to be changed, that's why it's red
        mCargoIntakeMotor.set(y);

    }

    //this might help with a better way to do the above, so I'm leaving it in, just commented out
    /* public void setOpenLoop(double percent) {
        mHatchMotor.set(ControlMode.PercentOutput, percent);
    } */

    

    ///////////////////////////////////////////////////////////
    //That's it, we're done.
    //Now you just need to write a section in the OI.java file.
    //I didn't add anything for that, so let me know when you reach this point and 
    //      I can help you.
    //////////////////////////////////////////////////////////

}