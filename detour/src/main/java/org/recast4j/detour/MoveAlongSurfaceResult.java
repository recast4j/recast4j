package org.recast4j.detour;

import java.util.List;

public class MoveAlongSurfaceResult {

	/** The result position of the mover. [(x, y, z)] */
	private final float[] resultPos;
	/** The reference ids of the polygons visited during the move. */
	private final List<Long> visited;

	public MoveAlongSurfaceResult(float[] resultPos, List<Long> visited) {
		this.resultPos = resultPos;
		this.visited = visited;
	}

	public float[] getResultPos() {
		return resultPos;
	}

	public List<Long> getVisited() {
		return visited;
	}

}
