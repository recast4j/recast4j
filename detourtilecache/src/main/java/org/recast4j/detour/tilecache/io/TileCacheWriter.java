package org.recast4j.detour.tilecache.io;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteOrder;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.io.DetourWriter;
import org.recast4j.detour.io.NavMeshParamWriter;
import org.recast4j.detour.tilecache.CompressedTile;
import org.recast4j.detour.tilecache.TileCache;
import org.recast4j.detour.tilecache.TileCacheCompressor;
import org.recast4j.detour.tilecache.TileCacheParams;

public class TileCacheWriter extends DetourWriter {

	private final NavMeshParamWriter paramWriter = new NavMeshParamWriter();

	public void write(OutputStream stream, TileCache cache, TileCacheCompressor compressor, NavMesh mesh,
			ByteOrder order, boolean cCompatibility) throws IOException {
		// Write header.
		write(stream, TileCacheSetHeader.TILECACHESET_MAGIC, order);
		write(stream, TileCacheSetHeader.TILECACHESET_VERSION, order);
		int numTiles = 0;
		for (int i = 0; i < cache.getTileCount(); ++i) {
			CompressedTile tile = cache.getTile(i);
			if (tile == null || tile.data == null)
				continue;
			numTiles++;
		}
		write(stream, numTiles, order);
		paramWriter.write(stream, mesh.getParams(), order);
		writeCacheParams(stream, cache.getParams(), order, cCompatibility);
		for (int i = 0; i < cache.getTileCount(); i++) {
			CompressedTile tile = cache.getTile(i);
			if (tile == null || tile.data == null)
				continue;
			write(stream, (int) cache.getTileRef(tile), order);
			write(stream, tile.data.length, order);
			stream.write(tile.data);
		}
	}

	private void writeCacheParams(OutputStream stream, TileCacheParams params, ByteOrder order, boolean cCompatibility)
			throws IOException {
		for (int i = 0; i < 3; i++) {
			write(stream, params.orig[i], order);
		}
		write(stream, params.cs, order);
		write(stream, params.ch, order);
		write(stream, params.width, order);
		write(stream, params.height, order);
		write(stream, params.walkableHeight, order);
		write(stream, params.walkableRadius, order);
		write(stream, params.walkableClimb, order);
		write(stream, params.maxSimplificationError, order);
		write(stream, params.maxTiles, order);
		write(stream, params.maxObstacles, order);
	}

}
