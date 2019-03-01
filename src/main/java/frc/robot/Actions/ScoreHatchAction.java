package frc.robot.Actions;

import frc.robot.Actions.Framework.Action;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Subsystems.HatchSubsystem;
import frc.robot.Subsystems.HatchSubsystem.HatchWantedState;
import frc.robot.Utilities.TrajectoryFollowingMotion.Path;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */

 

public class ScoreHatchAction implements Action {

    private HatchSubsystem hatchSubsystem;    

    public ScoreHatchAction() {
        hatchSubsystem = HatchSubsystem.getInstance();
	}

	@Override
	public boolean isFinished() {
		return true;
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

		hatchSubsystem.setHatchWantedState(HatchWantedState.ACQUIRE);

	}
}