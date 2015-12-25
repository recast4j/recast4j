package org.recast4j.detour.tilecache.io;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.io.IOUtils;
import org.recast4j.detour.io.NavMeshParamReader;
import org.recast4j.detour.tilecache.TileCache;
import org.recast4j.detour.tilecache.TileCacheCompressor;
import org.recast4j.detour.tilecache.TileCacheParams;
import org.recast4j.detour.tilecache.TileCacheStorageParams;

public class TileCacheReader {

	private final NavMeshParamReader paramReader = new NavMeshParamReader();

	public TileCache read(InputStream is, int maxVertPerPoly, TileCacheCompressor compressor, ByteOrder order, boolean cCompatibility)
			throws IOException {
		// Read header.
		ByteBuffer bb = IOUtils.toByteBuffer(is);
		bb.order(order);
		return read(bb, maxVertPerPoly, compressor, cCompatibility);
	}

	public TileCache read(ByteBuffer bb, int maxVertPerPoly, TileCacheCompressor compressor, boolean cCompatibility) throws IOException {
		TileCacheSetHeader header = new TileCacheSetHeader();
		header.magic = bb.getInt();
		if (header.magic != TileCacheSetHeader.TILECACHESET_MAGIC) {
			throw new IOException("Invalid magic");
		}
		header.version = bb.getInt();
		if (header.version != TileCacheSetHeader.TILECACHESET_VERSION) {
			throw new IOException("Invalid version");
		}
		header.numTiles = bb.getInt();
		header.meshParams = paramReader.read(bb);
		header.cacheParams = readCacheParams(bb, cCompatibility);
		NavMesh mesh = new NavMesh(header.meshParams, maxVertPerPoly);
		TileCache tc = new TileCache(header.cacheParams, new TileCacheStorageParams(bb.order(), cCompatibility), mesh, compressor, null);
		// Read tiles.
		for (int i = 0; i < header.numTiles; ++i) {
			long tileRef = bb.getInt();
			int dataSize = bb.getInt();
			if (tileRef == 0 || dataSize == 0) {
				break;
			}
			byte[] data = new byte[dataSize];
			bb.get(data);
			long tile = tc.addTile(data, 0);
			if (tile != 0)
				tc.buildNavMeshTile(tile);
		}
		return tc;
	}

	private TileCacheParams readCacheParams(ByteBuffer bb, boolean cCompatibility) {
		TileCacheParams params = new TileCacheParams();
		for (int i = 0; i < 3; i++) {
			params.orig[i] = bb.getFloat();
		}
		params.cs = bb.getFloat();
		params.ch = bb.getFloat();
		params.width = bb.getInt();
		params.height = bb.getInt();
		params.walkableHeight = bb.getFloat();
		params.walkableRadius = bb.getFloat();
		params.walkableClimb = bb.getFloat();
		params.maxSimplificationError = bb.getFloat();
		params.maxTiles = bb.getInt();
		params.maxObstacles = bb.getInt();
		return params;
	}
}
