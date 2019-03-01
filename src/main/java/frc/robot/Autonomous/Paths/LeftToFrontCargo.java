package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftToFrontCargo implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(68,200,0,0));
        sWaypoints.add(new Waypoint(124,200,20,20));
        sWaypoints.add(new Waypoint(148,165,20,20));
        sWaypoints.add(new Waypoint(192,163,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(68, 200), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}