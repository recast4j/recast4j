package org.recast4j.detour.extras.unity.astar;

import org.recast4j.detour.extras.Vector3f;

class GraphMeta {
	float characterRadius;
	float contourMaxError;
	float cellSize;
	float walkableHeight;
	float walkableClimb;
	float maxSlope;
	float maxEdgeLength;
	float minRegionSize;
	/** Size of tile along X axis in voxels */
	float tileSizeX;
	/** Size of tile along Z axis in voxels */
	float tileSizeZ;
	boolean useTiles;
	Vector3f rotation;
	Vector3f forcedBoundsCenter;
	Vector3f forcedBoundsSize;
}
