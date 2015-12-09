package org.recast4j.detour.tilecache;

import java.util.ArrayList;
import java.util.List;

public class TileCacheObstacle {

	final int index;
	float[] pos = new float[3];
	float radius, height;
	List<Long> touched = new ArrayList<>();
	List<Long> pending = new ArrayList<>();
	int salt;
	ObstacleState state = ObstacleState.DT_OBSTACLE_EMPTY;
	TileCacheObstacle next;

	public TileCacheObstacle(int index) {
		this.index = index;
	}

	public void reset() {
		for (int i = 0; i < 3; i++) {
			pos[i] = 0;
		}
		radius = 0;
		height = 0;
		touched.clear();
		pending.clear();
		salt = 0;
		state = ObstacleState.DT_OBSTACLE_EMPTY;
		next = null;
	}
}
