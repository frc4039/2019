/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// Make sure you have the phoenix tools installed:
// http://www.ctr-electronics.com/talon-srx.html#product_tabs_technical_resources
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Run;

/**
 * Add your docs here.
 */
public class MotorSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX motor = new TalonSRX(RobotMap.motor);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.

    // Making the default command Run() means it will be running when there
    // is no other command running. This ensures that it starts running when
    // the robot is enabled.
    setDefaultCommand(new Run());
  }

  public void run(double speed)
  {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void stop()
  {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
