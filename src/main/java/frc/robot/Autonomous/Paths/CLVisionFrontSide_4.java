package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class CLVisionFrontSide_4 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(25,295,0,0));
        sWaypoints.add(new Waypoint(100,295,30,30));
        sWaypoints.add(new Waypoint(175,230,30,30));
        sWaypoints.add(new Waypoint(265,230,10,30));
        sWaypoints.add(new Waypoint(265,250,0,30));
        

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(25, 295), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}