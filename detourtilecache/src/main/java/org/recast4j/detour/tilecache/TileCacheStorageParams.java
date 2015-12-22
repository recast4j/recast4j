package org.recast4j.detour.tilecache;

import java.nio.ByteOrder;

public class TileCacheStorageParams {

	final ByteOrder byteOrder;
	final boolean cCompatibility;

	public TileCacheStorageParams(ByteOrder byteOrder, boolean cCompatibility) {
		this.byteOrder = byteOrder;
		this.cCompatibility = cCompatibility;
	}

}
