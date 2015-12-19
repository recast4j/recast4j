package org.recast4j.detour.tilecache.io.compress;

import org.recast4j.detour.tilecache.TileCacheCompressor;

import net.jpountz.lz4.LZ4Factory;

public class LZ4TileCacheCompressor implements TileCacheCompressor {

	@Override
	public byte[] decompress(byte[] buf, int offset, int len, int outputlen) {
		return LZ4Factory.fastestInstance().fastDecompressor().decompress(buf, offset, outputlen);
	}

	@Override
	public byte[] compress(byte[] buf) {
		return LZ4Factory.fastestInstance().highCompressor().compress(buf);
	}

}
