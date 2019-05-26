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

import java.util.Arrays;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.OffMeshConnection;
import org.recast4j.detour.Poly;
import org.recast4j.detour.VectorPtr;

class OffMeshLinkCreator {

    void build(GraphMeshData graphData, NodeLink2[] links, int nodeOffset) {
        if (links.length > 0) {
            for (NodeLink2 l : links) {
                MeshData startTile = graphData.getTile(l.startNode - nodeOffset);
                Poly startNode = graphData.getNode(l.startNode - nodeOffset);
                MeshData endTile = graphData.getTile(l.endNode - nodeOffset);
                Poly endNode = graphData.getNode(l.endNode - nodeOffset);
                if (startNode != null && endNode != null) {
                    startTile.polys = Arrays.copyOf(startTile.polys, startTile.polys.length + 1);
                    int poly = startTile.header.polyCount;
                    startTile.polys[poly] = new Poly(poly, 2);
                    startTile.polys[poly].verts[0] = startTile.header.vertCount;
                    startTile.polys[poly].verts[1] = startTile.header.vertCount + 1;
                    startTile.polys[poly].setType(Poly.DT_POLYTYPE_OFFMESH_CONNECTION);
                    startTile.verts = Arrays.copyOf(startTile.verts, startTile.verts.length + 6);
                    startTile.header.polyCount++;
                    startTile.header.vertCount += 2;
                    OffMeshConnection connection = new OffMeshConnection();
                    connection.poly = poly;
                    connection.pos = new float[] { l.clamped1.x, l.clamped1.y, l.clamped1.z, l.clamped2.x, l.clamped2.y,
                            l.clamped2.z };
                    connection.rad = 0.1f;
                    connection.side = startTile == endTile ? 0xFF
                            : NavMeshBuilder.classifyOffMeshPoint(new VectorPtr(connection.pos, 3),
                                    startTile.header.bmin, startTile.header.bmax);
                    connection.userId = (int) l.linkID;
                    if (startTile.offMeshCons == null) {
                        startTile.offMeshCons = new OffMeshConnection[1];
                    } else {
                        startTile.offMeshCons = Arrays.copyOf(startTile.offMeshCons, startTile.offMeshCons.length + 1);
                    }
                    startTile.offMeshCons[startTile.offMeshCons.length - 1] = connection;
                    startTile.header.offMeshConCount++;
                }
            }
        }
    }
}
