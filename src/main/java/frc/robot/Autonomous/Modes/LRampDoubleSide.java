package frc.robot.Autonomous.Modes;

import frc.robot.Actions.DrivePathAction;
import frc.robot.Actions.Framework.WaitAction;
import frc.robot.Actions.ResetPoseFromPathAction;
import frc.robot.Actions.ScoreHatchAction;
import frc.robot.Actions.IntakeInAction;
import frc.robot.Actions.IntakeOutAction;
import frc.robot.Actions.Framework.ParallelAction;
import frc.robot.Actions.Framework.SeriesAction;
import frc.robot.Actions.Framework.WaitForPathMarkerAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Autonomous.Paths.LRampDoubleSide_1;
import frc.robot.Autonomous.Paths.LRampDoubleSide_2;
import frc.robot.Autonomous.Paths.LRampDoubleSide_3;
import frc.robot.Autonomous.Paths.LRampDoubleSide_4;
import frc.robot.Autonomous.Paths.LRampDoubleSide_5;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LRampDoubleSide extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new LRampDoubleSide_1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new WaitAction(3));
		runAction(new DrivePathAction(new LRampDoubleSide_2()));
		runAction(new WaitAction(3));
		runAction(new DrivePathAction(new LRampDoubleSide_3()));
		runAction(new WaitAction(3));
        runAction(new DrivePathAction(new LRampDoubleSide_4())); 
        runAction(new WaitAction(3));
        runAction(new DrivePathAction(new LRampDoubleSide_5()));   
	}
}