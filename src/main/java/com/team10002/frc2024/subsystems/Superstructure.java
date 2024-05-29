package com.team10002.frc2024.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team10002.frc2024.loops.ILooper;
import com.team10002.lib.requests.ParallelRequest;
import com.team10002.lib.requests.Request;
import com.team10002.lib.requests.SequentialRequest;

public class Superstructure extends Subsystem {
    
    private static Superstructure mInstance;

	public static synchronized Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

    // Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;


    private final IntakeSubsystem mIntakeRollers = IntakeSubsystem.getInstance();


    public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

    private void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	private void clearRequestQueue() {
		queuedRequests.clear();
	}

    private void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	private void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}



    private Request idleRequest() {
		return new ParallelRequest(
			mIntakeRollers.stateRequest(IntakeSubsystem.State.IDLE)
			
		);
	}

    public void exhaustState() {
		request(new ParallelRequest(
			
			mIntakeRollers.stateRequest(IntakeSubsystem.State.EXHAUST)
        ));
	}

    public void intake(){
        request(new SequentialRequest(mIntakeRollers.stateRequest(IntakeSubsystem.State.INTAKING)));
    }

    public void intakeIdle(){
        request(new SequentialRequest(mIntakeRollers.stateRequest(IntakeSubsystem.State.INTAKING)));
    }

}
