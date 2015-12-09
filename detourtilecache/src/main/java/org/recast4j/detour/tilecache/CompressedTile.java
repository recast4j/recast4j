package org.recast4j.detour.tilecache;

public class CompressedTile {
	final int index;
	int salt; /// < Counter describing modifications to the tile.
	TileCacheLayerHeader header;
	byte[] data;
	int compressed; // offset of compressed data
	int flags;
	CompressedTile next;

	public CompressedTile(int index) {
		this.index = index;
		this.salt = 1;
	}
}
