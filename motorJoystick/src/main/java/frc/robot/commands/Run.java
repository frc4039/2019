/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotorSub;

public class Run extends Command {

  public MotorSub MotorSub = new MotorSub();
  
  public Run() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(MotorSub);
  }

  // We don't need to override initialize() anymore because execute() is
  // doing all the work

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // This will call the function every robot tick.
    MotorSub.run(Robot.m_oi.getStickY());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    MotorSub.stop();
  }

  // If you don't implement interupted(), it will default to just calling end()
  // This is almost always what you want to happen.
}
