package org.recast4j.detour.tilecache;

public class TileCacheLayer {
	
	TileCacheLayerHeader header;
	int regCount;					///< Region count.
	byte[] heights; // char 
	byte[] areas;   // char
	byte[] cons;    // char
	byte[] regs;    // char
	
}
