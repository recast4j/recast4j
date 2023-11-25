/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.detour.extras.unity.astar;

import java.io.File;
import java.util.List;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshParams;

/**
 * Import navmeshes created with A* Pathfinding Project Unity plugin (https://arongranberg.com/astar/). Graph data is
 * loaded from a zip archive and converted to Recast navmesh objects.
 */
public class UnityAStarPathfindingImporter {

    private final UnityAStarPathfindingReader reader = new UnityAStarPathfindingReader();
    private final BVTreeCreator bvTreeCreator = new BVTreeCreator();
    private final LinkBuilder linkCreator = new LinkBuilder();
    private final OffMeshLinkCreator offMeshLinkCreator = new OffMeshLinkCreator();

    public NavMesh[] load(File zipFile) throws Exception {
        GraphData graphData = reader.read(zipFile);
        Meta meta = graphData.meta;
        NodeLink2[] nodeLinks2 = graphData.nodeLinks2;
        NavMesh[] meshes = new NavMesh[meta.graphs];
        int nodeOffset = 0;
        for (int graphIndex = 0; graphIndex < meta.graphs; graphIndex++) {
            GraphMeta graphMeta = graphData.graphMeta.get(graphIndex);
            GraphMeshData graphMeshData = graphData.graphMeshData.get(graphIndex);
            List<int[]> connections = graphData.graphConnections.get(graphIndex);
            int nodeCount = graphMeshData.countNodes();
            if (connections.size() != nodeCount) {
                throw new IllegalArgumentException("Inconsistent number of nodes in data file: " + nodeCount
                        + " and connection files: " + connections.size());
            }
            // Build BV tree
            bvTreeCreator.build(graphMeshData);
            // Create links between nodes (both internal and portals between tiles)
            linkCreator.build(nodeOffset, graphMeshData, connections);
            // Finally, process all the off-mesh links that can be actually converted to detour data
            offMeshLinkCreator.build(graphMeshData, nodeLinks2, nodeOffset);
            NavMeshParams params = new NavMeshParams();
            params.maxTiles = graphMeshData.tiles.length;
            params.maxPolys = 32768;
            params.tileWidth = graphMeta.tileSizeX * graphMeta.cellSize;
            params.tileHeight = graphMeta.tileSizeZ * graphMeta.cellSize;
            params.orig[0] = -0.5f * graphMeta.forcedBoundsSize.x + graphMeta.forcedBoundsCenter.x;
            params.orig[1] = -0.5f * graphMeta.forcedBoundsSize.y + graphMeta.forcedBoundsCenter.y;
            params.orig[2] = -0.5f * graphMeta.forcedBoundsSize.z + graphMeta.forcedBoundsCenter.z;
            NavMesh mesh = new NavMesh(params, 3);
            for (MeshData t : graphMeshData.tiles) {
                mesh.addTile(t, 0, 0);
            }
            meshes[graphIndex] = mesh;
            nodeOffset += graphMeshData.countNodes();
        }
        return meshes;
    }

}
