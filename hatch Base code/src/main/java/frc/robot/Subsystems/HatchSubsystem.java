/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class HatchSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public VictorSP hatchMotor = new VictorSP(1);
  public Solenoid hatchPlunger = new Solenoid(1);
  //public Servo hatchServo = new Servo(1); //in case we decide a servo is easier

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void open()
  {
    hatchMotor.set(0.25);
    hatchPlunger.set(false);
  }

  public void close()
  {
    hatchMotor.set(-0.25);
    hatchPlunger.set(false);
  }

  public void eject()
  {
    hatchMotor.set(0);
    hatchPlunger.set(true);
  }

  public void retract()
  {
    hatchMotor.set(0);
    hatchPlunger.set(false);
  }
}
