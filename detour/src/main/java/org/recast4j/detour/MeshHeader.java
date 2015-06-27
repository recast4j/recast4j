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

public class MeshHeader {
	/// Provides high level information related to a dtMeshTile object.
	/// @ingroup detour
	int magic; /// < Tile magic number. (Used to identify the data format.)
	int version; /// < Tile data format version number.
	int x; /// < The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	int y; /// < The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	int layer; /// < The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	int userId; /// < The user defined id of the tile.
	int polyCount; /// < The number of polygons in the tile.
	int vertCount; /// < The number of vertices in the tile.
	int maxLinkCount; /// < The number of allocated links.
	int detailMeshCount; /// < The number of sub-meshes in the detail mesh.

	/// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	int detailVertCount;

	int detailTriCount; /// < The number of triangles in the detail mesh.
	int bvNodeCount; /// < The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	int offMeshConCount; /// < The number of off-mesh connections.
	int offMeshBase; /// < The index of the first polygon which is an off-mesh connection.
	float walkableHeight; /// < The height of the agents using the tile.
	float walkableRadius; /// < The radius of the agents using the tile.
	float walkableClimb; /// < The maximum climb height of the agents using the tile.
	float[] bmin = new float[3]; /// < The minimum bounds of the tile's AABB. [(x, y, z)]
	float[] bmax = new float[3]; /// < The maximum bounds of the tile's AABB. [(x, y, z)]

	/// The bounding volume quantization factor.
	float bvQuantFactor;
}
