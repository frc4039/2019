/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Commands.HatchEject;
import frc.robot.Commands.HatchOpen;

/**
 * Add your docs here.
 */
public class OI {
    Joystick driver = new Joystick(0);
    XboxController operator = new XboxController(1);

    public OI()
    {
        JoystickButton buttonA = new JoystickButton(operator, 1); // A button on operator Xbox
        buttonA.whenPressed(new HatchEject()); // Pressed once, ejects hatch

        // Looked on CD to figure out exactly how it's supposed to work
        // and used the errors to figure out the rest
        /*JoystickButton leftTrigger = new JoystickButton(operator, (int) GetRawAxis(3));
        leftTrigger.whenActive(new HatchOpen());*/
        JoystickButton buttonB = new JoystickButton(operator, 2);
        buttonB.whenPressed(new HatchOpen());
    }

    public double GetRawAxis(int value) {
        return operator.getTriggerAxis(GenericHID.Hand.kLeft);
    }
}
