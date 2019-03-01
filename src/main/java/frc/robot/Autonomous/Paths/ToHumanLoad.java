package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class ToHumanLoad implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(290,232,0,0));
        sWaypoints.add(new Waypoint(272,214,15,20));
        sWaypoints.add(new Waypoint(150,279,50,40));
		sWaypoints.add(new Waypoint(40,279,0,60));
		sWaypoints.add(new Waypoint(25,279,0,40));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(290, 232), Rotation2d.fromDegrees(-150));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}