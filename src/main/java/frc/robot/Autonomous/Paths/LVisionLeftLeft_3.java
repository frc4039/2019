package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TrajectoryFollowingMotion.*;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LVisionLeftLeft_3 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(265,270,0,0));
        sWaypoints.add(new Waypoint(265,245,18,30));
        sWaypoints.add(new Waypoint(161,245,50,30));
        sWaypoints.add(new Waypoint(110,295,20,30));
        sWaypoints.add(new Waypoint(60,295,0,30));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(265, 270), Rotation2d.fromDegrees(-90)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}