package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class CRampReverseFrontSide_3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(25,295,0,0));
        sWaypoints.add(new Waypoint(70,295,20,30));
        sWaypoints.add(new Waypoint(161,221,50,30));
        sWaypoints.add(new Waypoint(265,220,10,30));
        sWaypoints.add(new Waypoint(265,240,0,30));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(25, 295), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}