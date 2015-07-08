package org.recast4j.detour;

import java.util.ArrayList;
import java.util.List;

/// Provides information about raycast hit
/// filled by NavMeshQuery::raycast
public class RaycastHit {
	
	Status status;
	/// The hit parameter. (FLT_MAX if no wall hit.)
	float t; 
	
	/// hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	final float[] hitNormal = new float[3];
	
	/// Visited polygons. [opt]
	final List<Long> path = new ArrayList<>();
	
	///  The cost of the path until hit.
	float pathCost;
}
