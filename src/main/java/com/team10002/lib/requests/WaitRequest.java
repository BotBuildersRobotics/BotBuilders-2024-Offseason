package com.team10002.lib.requests;

import com.team10002.lib.util.Stopwatch;

public class WaitRequest extends Request {
	private final double waitTimeSeconds;
	private final Stopwatch stopwatch = new Stopwatch();

	public WaitRequest(double waitTimeSeconds) {
		this.waitTimeSeconds = waitTimeSeconds;
	}

	@Override
	public void act() {
		stopwatch.start();
	}

	@Override
	public boolean isFinished() {
		return stopwatch.getTime() > waitTimeSeconds;
	}
}
