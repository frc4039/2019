package frc.robot.Actions;

import frc.robot.Actions.Framework.Action;
import frc.robot.Subsystems.DriveBaseSubsystem;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */
public class AutoVisionAction implements Action {

    private DriveBaseSubsystem mDrive = DriveBaseSubsystem.getInstance();

    public AutoVisionAction() {
    }

    @Override
    public boolean isFinished() {
        if ((mDrive.getLeftVelocityInchesPerSec() + mDrive.getRightVelocityInchesPerSec()) < 0.5) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mDrive.setAutoVisionAssist();
    }
}
