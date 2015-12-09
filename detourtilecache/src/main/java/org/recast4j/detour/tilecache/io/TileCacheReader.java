package org.recast4j.detour.tilecache.io;

import java.io.IOException;
import java.nio.ByteBuffer;

import org.recast4j.detour.tilecache.TileCacheLayerHeader;

public class TileCacheReader {

	static final int DT_TILECACHE_MAGIC = 'D'<<24 | 'T'<<16 | 'L'<<8 | 'R'; ///< 'DTLR';
	static final int DT_TILECACHE_VERSION = 1;
	
	public TileCacheLayerHeader readLayerHeader(ByteBuffer data) throws IOException {
		TileCacheLayerHeader header = new TileCacheLayerHeader();
		header.magic = data.getInt();
		header.version = data.getInt();

		if (header.magic != DT_TILECACHE_MAGIC)
			throw new IOException("Invalid magic");
		if (header.version != DT_TILECACHE_VERSION)
			throw new IOException("Invalid version");

		header.tx = data.getInt();
		header.ty = data.getInt();
		header.tlayer = data.getInt();
		for (int j = 0; j < 3; j++) {
			header.bmin[j] = data.getFloat();
		}
		for (int j = 0; j < 3; j++) {
			header.bmax[j] = data.getFloat();
		}
		header.hmin = data.getShort() & 0xFFFF;
		header.hmax = data.getShort() & 0xFFFF;
		header.width = data.get()  & 0xFF;
		header.height = data.get()  & 0xFF;
		header.minx = data.get()  & 0xFF;
		header.maxx = data.get()  & 0xFF;
		header.miny = data.get()  & 0xFF;
		header.maxy = data.get()  & 0xFF;
		return header;
	}

	
}
