package org.recast4j.detour.tilecache;

public class TileCacheParams {
	float[] orig = new float[3];
	float cs, ch;
	int width, height;
	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float maxSimplificationError;
	int maxTiles;
	int maxObstacles;
}
