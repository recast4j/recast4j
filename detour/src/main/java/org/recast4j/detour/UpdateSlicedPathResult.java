package org.recast4j.detour;

//TODO: (PP) Add comments
public class UpdateSlicedPathResult {
	private final Status status;
	private final int iterations;

	public UpdateSlicedPathResult(Status status, int iterations) {
		this.status = status;
		this.iterations = iterations;
	}

	public Status getStatus() {
		return status;
	}

	public int getIterations() {
		return iterations;
	}
}