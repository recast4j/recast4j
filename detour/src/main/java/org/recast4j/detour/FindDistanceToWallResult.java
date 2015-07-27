package org.recast4j.detour;

//TODO: (PP) Add comments
public class FindDistanceToWallResult {
	private final float distance;
	private final float[] position;
	private final float[] normal;

	public FindDistanceToWallResult(float distance, float[] position, float[] normal) {
		this.distance = distance;
		this.position = position;
		this.normal = normal;
	}

	public float getDistance() {
		return distance;
	}

	public float[] getPosition() {
		return position;
	}

	public float[] getNormal() {
		return normal;
	}

}