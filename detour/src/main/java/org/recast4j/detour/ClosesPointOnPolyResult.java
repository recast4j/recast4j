package org.recast4j.detour;

public class ClosesPointOnPolyResult {

	private final boolean posOverPoly;
	private final float[] closest;

	public ClosesPointOnPolyResult(boolean posOverPoly, float[] closest) {
		this.posOverPoly = posOverPoly;
		this.closest = closest;
	}

	/** Returns true if the position is over the polygon. */
	public boolean isPosOverPoly() {
		return posOverPoly;
	}

	/** Returns the closest point on the polygon. [(x, y, z)] */
	public float[] getClosest() {
		return closest;
	}

}
