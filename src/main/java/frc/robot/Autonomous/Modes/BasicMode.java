package frc.robot.Autonomous.Modes;

import frc.robot.Actions.DrivePathAction;
import frc.robot.Actions.Framework.WaitAction;
import frc.robot.Actions.ResetPoseFromPathAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Autonomous.Paths.SamplePath;
import frc.robot.Autonomous.Paths.SecondPath;
import frc.robot.Autonomous.Paths.StraightPath;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class BasicMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new StraightPath();
		runAction(new ResetPoseFromPathAction(pathContainer));

		/* PathContainer pathContainer = new SamplePath();
		runAction(new ResetPoseFromPathAction(pathContainer));

		runAction(new DrivePathAction(pathContainer));

		runAction(new WaitAction(2));

		runAction(new DrivePathAction(new SecondPath()));

		runAction(new WaitAction(15)); */
	}
}
