package org.recast4j.detour.tilecache.io.compress;

import org.recast4j.detour.tilecache.TileCacheCompressor;

public class TileCacheCompressorFactory {

	public static TileCacheCompressor get(boolean cCompatibility) {
		return cCompatibility ? new FastLzTileCacheCompressor() : new LZ4TileCacheCompressor();
	}
}
