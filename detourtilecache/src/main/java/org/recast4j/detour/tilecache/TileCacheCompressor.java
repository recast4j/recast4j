package org.recast4j.detour.tilecache;

public interface TileCacheCompressor {

	byte[] decompress(byte[] buf, int offset, int len, int outputlen);

}
