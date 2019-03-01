package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class SideCargoBlueRight2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(275,127,0,0));
		sWaypoints.add(new Waypoint(275,107,15,20));
		sWaypoints.add(new Waypoint(290,92,0,30));


		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(275, 197), Rotation2d.fromDegrees(-90));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}