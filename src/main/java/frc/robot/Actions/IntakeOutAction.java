package frc.robot.Actions;

import frc.robot.Actions.Framework.Action;
import frc.robot.Subsystems.DriveBaseSubsystem;
import frc.robot.Subsystems.CargoSubsystem;
import frc.robot.Subsystems.CargoSubsystem.CargoWantedState;
import frc.robot.Utilities.TrajectoryFollowingMotion.Path;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */

 

public class IntakeOutAction implements Action {

    private CargoSubsystem cargoSubsystem;    

    public IntakeOutAction() {
        cargoSubsystem = CargoSubsystem.getInstance();
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

		cargoSubsystem.setIntakeOut();

	}
}