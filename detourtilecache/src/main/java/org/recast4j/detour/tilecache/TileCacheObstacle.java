package org.recast4j.detour.tilecache;

public class TileCacheObstacle {

	static final int DT_MAX_TOUCHED_TILES = 8;
	
	float[] pos = new float[3];
	float radius, height;
	long[] touched = new long[DT_MAX_TOUCHED_TILES];
	long[] pending = new long[DT_MAX_TOUCHED_TILES];
	int salt;
	int state;
	int ntouched;
	int npending;
	TileCacheObstacle next;
}
