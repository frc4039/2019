/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Commands.SpinMotor;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Motor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  VictorSPX motor1 = new VictorSPX(RobotMap.motor1);
  
public void SpinForward() {
  motor1.set(ControlMode.PercentOutput, 0.5);
}
public void SpinBack(){
  motor1.set(ControlMode.PercentOutput, -0.5);
}
public void SpinStop(){
  motor1.set(ControlMode.PercentOutput, 0);
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new SpinMotor());
  }
}