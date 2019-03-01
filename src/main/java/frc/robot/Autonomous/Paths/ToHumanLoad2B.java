package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class ToHumanLoad2B implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(190,203,0,0));
        sWaypoints.add(new Waypoint(130,287,60,20));
        sWaypoints.add(new Waypoint(60,283,0,20));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(190, 203), Rotation2d.fromDegrees(150));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}