package org.recast4j.detour.tilecache;

public class TileCachePolyMesh {
	int nvp;
	int nverts;				///< Number of vertices.
	int npolys;				///< Number of polygons.
	int[] verts;	///< Vertices of the mesh, 3 elements per vertex.
	int[] polys;	///< Polygons of the mesh, nvp*2 elements per polygon.
	int[] flags;	///< Per polygon flags.
	int[] areas;	///< Area ID of polygons.
}
