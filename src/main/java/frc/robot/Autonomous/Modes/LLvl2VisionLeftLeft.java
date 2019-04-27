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
import frc.robot.Autonomous.Paths.LLvl2VisionLeftLeft_1;
import frc.robot.Autonomous.Paths.LLvl2VisionLeftLeft_2;
import frc.robot.Autonomous.Paths.LLvl2VisionLeftLeft_3;
import frc.robot.Autonomous.Paths.LLvl2VisionLeftLeft_4;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class LLvl2VisionLeftLeft extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new LLvl2VisionLeftLeft_1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new WaitAction(2));
		runAction(new AutoVisionAction());
		runAction(new ScoreHatchAction());
		//runAction(new DrivePathAction(new LLvl2VisionLeftLeft_2()));
		//runAction(new DrivePathAction(new LLvl2VisionLeftLeft_3()));
		//runAction(new AutoVisionAction());
		//runAction(new GrabHatchAction());
		//runAction(new DrivePathAction(new LLvl2VisionLeftLeft_4()));
		//runAction(new AutoVisionAction());
		//runAction(new ScoreHatchAction());
        
	}
}