package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class CRVisionFrontSide_1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(70,324-173,0,0));
		sWaypoints.add(new Waypoint(160,324-173,0,30));
        //sWaypoints.add(new Waypoint(208,173,0,30));    //original value to drive full distance
        

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(70, 324-173), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}