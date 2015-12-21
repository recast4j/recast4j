package org.recast4j.detour.tilecache.io;

import org.recast4j.detour.NavMeshParams;
import org.recast4j.detour.tilecache.TileCacheParams;

public class TileCacheSetHeader {

	static final int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T'; // 'TSET';
	static final int TILECACHESET_VERSION = 1;

	int magic;
	int version;
	int numTiles;
	NavMeshParams meshParams = new NavMeshParams();
	TileCacheParams cacheParams = new TileCacheParams();

}
