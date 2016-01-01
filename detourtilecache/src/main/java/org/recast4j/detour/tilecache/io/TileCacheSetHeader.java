package org.recast4j.detour.tilecache.io;

import org.recast4j.detour.NavMeshParams;
import org.recast4j.detour.tilecache.TileCacheParams;

public class TileCacheSetHeader {

	public static final int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T'; // 'TSET';
	public static final int TILECACHESET_VERSION = 1;
	public static final int TILECACHESET_VERSION_RECAST4J = 0x8801;

	int magic;
	int version;
	int numTiles;
	NavMeshParams meshParams = new NavMeshParams();
	TileCacheParams cacheParams = new TileCacheParams();

}
