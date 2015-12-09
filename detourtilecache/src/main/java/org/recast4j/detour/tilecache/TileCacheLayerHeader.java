package org.recast4j.detour.tilecache;

public class TileCacheLayerHeader {
	public int magic; /// < Data magic
	public int version; /// < Data version
	public int tx, ty, tlayer;
	public float[] bmin = new float[3];
	public float[] bmax = new float[3];
	public int hmin, hmax; /// < Height min/max range
	public int width, height; /// < Dimension of the layer.
	public int minx, maxx, miny, maxy; /// < Usable sub-region.

}
