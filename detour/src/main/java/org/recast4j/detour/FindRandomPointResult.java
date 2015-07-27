package org.recast4j.detour;

//TODO: (PP) Add comments
public class FindRandomPointResult {
	public final Status status;
	///  @param[out]	randomRef		The reference id of the random location.
	public final long randomRef;
	///  @param[out]	randomPt		The random location. 
	public final float[] randomPt;

	public FindRandomPointResult(Status status, long randomRef, float[] randomPt) {
		this.status = status;
		this.randomRef = randomRef;
		this.randomPt = randomPt;
	}
}