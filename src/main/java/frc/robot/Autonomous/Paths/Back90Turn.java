package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class Back90Turn implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(275,198,0,0));
		sWaypoints.add(new Waypoint(275,217,15,20));
		sWaypoints.add(new Waypoint(290,232,0,30));


		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(275, 198), Rotation2d.fromDegrees(-90));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}