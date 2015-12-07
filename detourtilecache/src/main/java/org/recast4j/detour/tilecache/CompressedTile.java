package org.recast4j.detour.tilecache;

public class CompressedTile {
	int salt;						///< Counter describing modifications to the tile.
	TileCacheLayerHeader header;
	byte[] compressed;
	int compressedSize;
	byte[] data;
	int dataSize;
	int flags;
	CompressedTile next;
}
