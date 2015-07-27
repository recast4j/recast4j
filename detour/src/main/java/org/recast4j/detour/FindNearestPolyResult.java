package org.recast4j.detour;

public class FindNearestPolyResult {
	private final long nearestRef;
	private final float[] nearestPos;

	public FindNearestPolyResult(long nearestRef, float[] nearestPos) {
		this.nearestRef = nearestRef;
		this.nearestPos = nearestPos;
	}

	/** Returns the reference id of the nearest polygon. */
	public long getNearestRef() {
		return nearestRef;
	}

	/** Returns the nearest point on the polygon. [opt] [(x, y, z)] */
	public float[] getNearestPos() {
		return nearestPos;
	}

}
