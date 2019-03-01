package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class SideCargoBlueRight1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		
		sWaypoints.add(new Waypoint(68,124,0,0));
        sWaypoints.add(new Waypoint(140,124,40,20));
        sWaypoints.add(new Waypoint(271,74,40,40));
        sWaypoints.add(new Waypoint(271,129,0,30));
		
		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(68,124), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}