package frc.robot.Autonomous.Modes;

import frc.robot.Actions.DrivePathAction;
import frc.robot.Actions.GrabHatchAction;
import frc.robot.Actions.Framework.WaitAction;
import frc.robot.Actions.ResetPoseFromPathAction;
import frc.robot.Actions.ScoreHatchAction;
import frc.robot.Actions.IntakeInAction;
import frc.robot.Actions.IntakeOutAction;
import frc.robot.Actions.AutoVisionAction;
import frc.robot.Actions.Framework.ParallelAction;
import frc.robot.Actions.Framework.SeriesAction;
import frc.robot.Actions.Framework.WaitForPathMarkerAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Autonomous.Paths.CRVisionFrontSide_1;
import frc.robot.Autonomous.Paths.CRVisionFrontSide_2;
import frc.robot.Autonomous.Paths.CRVisionFrontSide_3;
import frc.robot.Autonomous.Paths.CRVisionFrontSide_4;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class CRVisionFrontSide extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new CRVisionFrontSide_1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new WaitAction(0.1));
		runAction(new AutoVisionAction());
		runAction(new ScoreHatchAction());
		//runAction(new WaitAction(1));
		runAction(new DrivePathAction(new CRVisionFrontSide_2()));
		runAction(new DrivePathAction(new CRVisionFrontSide_3()));
		//runAction(new AutoVisionAction());
		//runAction(new GrabHatchAction());
		//runAction(new DrivePathAction(new CRVisionFrontSide_4()));
		//runAction(new AutoVisionAction());
		//runAction(new ScoreHatchAction()); 
	}
}