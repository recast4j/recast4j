package org.recast4j.detour.crowd;

public class SweepCircleCircleResult {

	final boolean intersection;
	final float htmin;
	final float htmax;

	public SweepCircleCircleResult(boolean intersection, float htmin, float htmax) {
		this.intersection = intersection;
		this.htmin = htmin;
		this.htmax = htmax;
	}

}
