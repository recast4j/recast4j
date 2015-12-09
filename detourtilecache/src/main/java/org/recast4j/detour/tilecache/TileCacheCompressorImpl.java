package org.recast4j.detour.tilecache;

import org.recast4j.detour.tilecache.io.FastLz;

public class TileCacheCompressorImpl implements TileCacheCompressor {

	@Override
	public byte[] decompress(byte[] buf, int offset, int len, int outputlen) {
		byte[] output = new byte[outputlen];
		FastLz.decompress(buf, offset, len, output, 0, outputlen);
		return output;
	}

}
