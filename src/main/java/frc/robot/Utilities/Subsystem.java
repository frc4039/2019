package frc.robot.Utilities;

import frc.robot.Utilities.Loops.Looper;

public abstract class Subsystem {

    public void writeToLog() {

    }

    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);

}