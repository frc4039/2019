/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Commands.SpinMotor;

/**
 * Add your docs here.
 */
public class Motor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  
  private TalonSRX leftFront = new TalonSRX(RobotMap.motor1);
  private TalonSRX rightFront = new TalonSRX(RobotMap.motor2);
  
  public void TestSpin(){
    leftFront.set(ControlMode.PercentOutput, 0.5);
    rightFront.set(ControlMode.PercentOutput, 0.5);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new SpinMotor());
  }
}
