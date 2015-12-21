package org.recast4j.detour.io;

import java.nio.ByteBuffer;

import org.recast4j.detour.NavMeshParams;

public class NavMeshParamReader {

	public NavMeshParams read(ByteBuffer bb) {
		NavMeshParams params = new NavMeshParams();
		params.orig[0] = bb.getFloat();
		params.orig[1] = bb.getFloat();
		params.orig[2] = bb.getFloat();
		params.tileWidth = bb.getFloat();
		params.tileHeight = bb.getFloat();
		params.maxTiles = bb.getInt();
		params.maxPolys = bb.getInt();
		return params;
	}

}
