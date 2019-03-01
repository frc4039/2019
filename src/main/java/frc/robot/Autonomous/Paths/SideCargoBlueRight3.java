package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class SideCargoBlueRight3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(290,92,0,0));
        sWaypoints.add(new Waypoint(272,110,15,20));
        sWaypoints.add(new Waypoint(150,45,50,40));
		sWaypoints.add(new Waypoint(40,45,0,60));
		sWaypoints.add(new Waypoint(25,45,0,40));

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