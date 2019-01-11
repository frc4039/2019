package frc.robot.Utilities;

import frc.robot.Utilities.Loops.Looper;

public interface CustomSubsystem {
	void init();
	void subsystemHome();
	void registerEnabledLoops(Looper in);
}
