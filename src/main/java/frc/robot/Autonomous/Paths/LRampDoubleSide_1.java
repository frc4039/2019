package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LRampDoubleSide_1 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(72,207,0,0));
        sWaypoints.add(new Waypoint(148,207,40,30));
        sWaypoints.add(new Waypoint(263,240,20,30));
        sWaypoints.add(new Waypoint(263,208,0,30));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(72, 207), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}