package org.recast4j.detour.tilecache;

import org.recast4j.detour.NavMeshCreateParams;

public interface TileCacheMeshProcess {

	void process(NavMeshCreateParams params, int[] areas, int[] flags);
}
