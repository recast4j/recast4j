/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour;

import java.util.List;

/// Defines a navigation mesh tile.
/// @ingroup detour
public class MeshTile {
	final int index;

	int salt; /// < Counter describing modifications to the tile.

	MeshHeader header; /// < The tile header.
	Poly[] polys; /// < The tile polygons. [Size: dtMeshHeader::polyCount]
	float[] verts; /// < The tile vertices. [Size: dtMeshHeader::vertCount]
	List<Link> links; /// < The tile links. [Size: dtMeshHeader::maxLinkCount]
	PolyDetail[] detailMeshes; /// < The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]

	/// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	float[] detailVerts;

	/// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	int[] detailTris;

	/// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount]
	/// (Will be null if bounding volumes are disabled.)
	BVNode[] bvTree;

	OffMeshConnection[] offMeshCons; /// < The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]

	NavMeshData data; /// < The tile data. (Not directly accessed under normal situations.)
	int flags; /// < Tile flags. (See: #dtTileFlags)
	MeshTile next; /// < The next free tile, or the next tile in the spatial grid.

	public MeshTile(int index) {
		this.index = index;
	}

}
