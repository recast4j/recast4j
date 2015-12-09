package org.recast4j.detour.tilecache.io;

import org.recast4j.detour.NavMeshParams;


public class NavMeshSetHeader {

	static final int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
	static final int NAVMESHSET_VERSION = 1;

	int magic;
	int version;
	int numTiles;
	NavMeshParams params = new NavMeshParams();

}

