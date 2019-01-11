package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utilities.TrajectoryFollowingMotion.MovingAverage;

public class ThreadRateControl {
	private double startTime;
	private double endTime;
	private int elapsedTimeMS;
	private boolean started;
	private double mPrevStartTime;
	private double mLoopTimeMS;
	private MovingAverage mAverageLoopTime;


	public ThreadRateControl() {
		startTime = 0;
		mPrevStartTime = 0;
		mLoopTimeMS = 0;
		endTime = 0;
		elapsedTimeMS = 0;
		started = false;
		mAverageLoopTime = new MovingAverage(20);
	}

	public synchronized void start(boolean resetStart) {
		if (resetStart)
			started = false;
		start();
	}

	public synchronized void start() {
		if (!started) {
			startTime = Timer.getFPGATimestamp();
			mPrevStartTime = startTime;
			started = true;
		} else {

		}
	}

	public synchronized void doRateControl(int minLoopTime) {
		mLoopTimeMS = (startTime - mPrevStartTime) * 1000;
		mAverageLoopTime.addNumber(mLoopTimeMS);
		if (startTime != 0) {
			do {
				endTime = Timer.getFPGATimestamp();
				elapsedTimeMS = (int) ((endTime - startTime) * 1000);
				if (elapsedTimeMS < minLoopTime) {
					try {
						Thread.sleep(minLoopTime - elapsedTimeMS);
					} catch (Exception ex) {

					}
				}
			} while (elapsedTimeMS < minLoopTime);
		} else {

		}
		mPrevStartTime = startTime;
		startTime = Timer.getFPGATimestamp();
	}

	public double getLoopTime() {
		return mLoopTimeMS;
	}

	public double getAverageLoopTime() {
		return mAverageLoopTime.getAverage();
	}
}
