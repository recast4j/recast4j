package org.recast4j.detour.tilecache;

public class TileCacheLayer {
	TileCacheLayerHeader header;
	int regCount;					///< Region count.
	int[] heights;
	int[] areas;
	int[] cons;
	int[] regs;
}
