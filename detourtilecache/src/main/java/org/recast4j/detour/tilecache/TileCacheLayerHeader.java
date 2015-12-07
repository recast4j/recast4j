package org.recast4j.detour.tilecache;

public class TileCacheLayerHeader {
	int magic; /// < Data magic
	int version; /// < Data version
	int tx, ty, tlayer;
	float[] bmin = new float[3];
	float[] bmax = new float[3];
	int hmin, hmax; /// < Height min/max range
	int width, height; /// < Dimension of the layer.
	int minx, maxx, miny, maxy; /// < Usable sub-region.
}
