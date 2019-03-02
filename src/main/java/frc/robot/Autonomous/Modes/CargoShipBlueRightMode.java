package frc.robot.Autonomous.Modes;

import frc.robot.Actions.DrivePathAction;
import frc.robot.Actions.Framework.WaitAction;
import frc.robot.Actions.ResetPoseFromPathAction;
import frc.robot.Actions.ScoreHatchAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Autonomous.Paths.LeftToLeftCargo;
import frc.robot.Autonomous.Paths.StraightPath;
import frc.robot.Autonomous.Paths.SideCargoBlueRight1;
import frc.robot.Autonomous.Paths.SideCargoBlueRight2;
import frc.robot.Autonomous.Paths.SideCargoBlueRight3;
import frc.robot.Autonomous.Paths.Back90Turn;
import frc.robot.Autonomous.Paths.ToHumanLoad;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

public class CargoShipBlueRightMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {

		PathContainer pathContainer = new SideCargoBlueRight1();
		runAction(new ResetPoseFromPathAction(pathContainer));
		runAction(new DrivePathAction(pathContainer));
		runAction(new WaitAction(0.1));
		//runAction(new ScoreHatchAction());
		//runAction(new WaitAction(2));
		//runAction(new DrivePathAction(new SideCargoBlueRight2()));
		//runAction(new WaitAction(2));
		//runAction(new DrivePathAction(new SideCargoBlueRight3()));
		//runAction(new WaitAction(15));
	}
}
