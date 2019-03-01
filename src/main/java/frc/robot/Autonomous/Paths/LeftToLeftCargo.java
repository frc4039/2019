package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftToLeftCargo implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		
		/*sWaypoints.add(new Waypoint(68,200,0,0));
        sWaypoints.add(new Waypoint(111,200,0,20));
        sWaypoints.add(new Waypoint(160,200,25,20));
        sWaypoints.add(new Waypoint(190,250,20,20));
        sWaypoints.add(new Waypoint(270,250,25,20));
        sWaypoints.add(new Waypoint(270,215,0,20));
		sWaypoints.add(new Waypoint(270,200,0,20));*/
		
		sWaypoints.add(new Waypoint(68,200,0,0));
        sWaypoints.add(new Waypoint(140,200,40,20));
        sWaypoints.add(new Waypoint(271,250,40,40));
        sWaypoints.add(new Waypoint(271,195,0,30));


		
		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(68,200), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}