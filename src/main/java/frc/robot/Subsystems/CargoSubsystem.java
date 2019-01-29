package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

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

public class CargoSubsystem extends Subsystem {

    public static boolean kIntakeOut = false;
    public static boolean kIntakeIn = !kIntakeOut;

    private static CargoSubsystem mInstance;

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final Solenoid mCargoIntakeSolenoid;
    private VictorSPX mCargoIntakeMotor;
    private VictorSPX mCargoShooterMotor;


    private boolean mPrevBrakeModeVal;

    private boolean CargoIntakeIn = true;

    public static CargoSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new CargoSubsystem();
        }
        return mInstance;
    }

    private CargoSubsystem() {

        Controllers robotControllers = Controllers.getInstance();
        mCargoIntakeMotor = robotControllers.getCargoIntakeMotor();
        mCargoShooterMotor = robotControllers.getCargoShooterMotor();
        mCargoIntakeSolenoid.close(); //check

        mPrevBrakeModeVal = false;
		    setBrakeMode(true);
    }

  private void setBrakeMode(boolean b) {
  }

  public void init() {
      mCargoIntakeMotor.setSensorPhase(false);
      mCargoIntakeMotor.setInverted(true);
      mCargoShooterMotor.setSensorPhase(false);
      mCargoShooterMotor.setInverted(true);
      
		  setBrakeMode(false);
    }

    private void IntakeOut() {
      CargoIntakeIn = false;
      mCargoIntakeSolenoid(CargoIntakeIn);
    }
  
    public void IntakeIn() {
      CargoIntakeIn = true;
      mCargoIntakeSolenoid(CargoIntakeIn);
    }

    public void rollersIn()
    {
      mCargoIntakeMotor.set(ControlMode.PercentOutput, 0.5);
      mCargoShooterMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void runShooter() {
      mCargoIntakeMotor.set(ControlMode.PercentOutput, 0.5);
      mCargoShooterMotor.set(ControlMode.PercentOutput, 0.5);
    }

    @Override
    public void stop() {
      mCargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);
      mCargoShooterMotor.set(ControlMode.PercentOutput, 0.0);
      CargoIntakeIn = true;
    }

  private final Loop mLoop = new Loop() {


        @Override
        public void onFirstStart() {
          mCargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);
          mCargoShooterMotor.set(ControlMode.PercentOutput, 0.0);
          CargoIntakeIn = false;
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {

        }

  public void outputToSmartDashboard(){
     //nothing
  }
    
	public void subsystemZero() {}


  @Override
  public void outputToSmartDashboard() {

  }

  @Override
  public void zeroSensors() {

  }

@Override
public void registerEnabledLoops(Looper enabledLooper) {

}


