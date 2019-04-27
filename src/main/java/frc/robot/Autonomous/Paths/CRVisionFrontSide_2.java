package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class CRVisionFrontSide_2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(208,324-173,0,0));		//guess at endpoint of previous path
        sWaypoints.add(new Waypoint(160,324-173,30,30));
        sWaypoints.add(new Waypoint(160,324-295,10,30));
        sWaypoints.add(new Waypoint(180,324-295,0,30));
        

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(208, 324-173), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}