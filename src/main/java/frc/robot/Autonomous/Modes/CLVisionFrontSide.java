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
import frc.robot.Autonomous.Paths.CLVisionFrontSide_1;
import frc.robot.Autonomous.Paths.CLVisionFrontSide_2;
import frc.robot.Autonomous.Paths.CLVisionFrontSide_3;
import frc.robot.Autonomous.Paths.CLVisionFrontSide_4;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

import java.util.Arrays;

public class CLVisionFrontSide extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new CLVisionFrontSide_1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new AutoVisionAction());
		runAction(new ScoreHatchAction());
		runAction(new DrivePathAction(new CLVisionFrontSide_2()));
		runAction(new DrivePathAction(new CLVisionFrontSide_3()));
		runAction(new AutoVisionAction());
		runAction(new GrabHatchAction());
		runAction(new DrivePathAction(new CLVisionFrontSide_4()));
		runAction(new AutoVisionAction());
		runAction(new ScoreHatchAction());
        ////runAction(new IntakeOutAction());
        //runAction(new ParallelAction(Arrays.asList(new DrivePathAction(new CRampReverseFrontSide_2()),
		//		new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("IntakeUp"),
		//				new IntakeInAction())))));
        //runAction(new DrivePathAction(new CRampReverseFrontSide_2()));
        //runAction(new IntakeInAction());
        //runAction(new GrabHatchAction());
		//runAction(new WaitAction(3));
        //runAction(new DrivePathAction(new CRampReverseFrontSide_3()));
		//runAction(new WaitAction(3));
        //runAction(new DrivePathAction(new CRampReverseFrontSide_4()));  
        //runAction(new ScoreHatchAction()); 
	}
}