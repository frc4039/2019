package frc.robot.Autonomous.Modes;

import frc.robot.Actions.DrivePathAction;
import frc.robot.Actions.Framework.WaitAction;
import frc.robot.Actions.ResetPoseFromPathAction;
import frc.robot.Actions.ScoreHatchAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Autonomous.Paths.LeftToFrontCargo;
import frc.robot.Autonomous.Paths.ToHumanLoad2A;
import frc.robot.Autonomous.Paths.ToHumanLoad2B;
import frc.robot.Autonomous.Paths.StraightPath;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class CargoShipFrontMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new LeftToFrontCargo();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new WaitAction(3));
		//runAction(new ScoreHatchAction());
		//runAction(new WaitAction(3));
		//runAction(new DrivePathAction(new ToHumanLoad2A()));
		//runAction(new WaitAction(3));
		//runAction(new DrivePathAction(new ToHumanLoad2B()));

		//runAction(new WaitAction(15));
	}
}