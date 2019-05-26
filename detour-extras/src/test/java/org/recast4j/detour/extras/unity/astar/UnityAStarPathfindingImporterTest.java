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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.BVNode;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Poly;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;
import org.recast4j.detour.io.MeshSetWriter;

public class UnityAStarPathfindingImporterTest {

    /**
     * 3 is enough for recast4j as all nodes created by A* Star Pathfinding are triangles. However 6 can be used to to
     * keep the result compatible with RecastDemo.
     */
    static final int MAX_VERTS_PER_POLY = 6;

    @Test
    public void test_v4_0_6() throws Exception {
        NavMesh mesh = loadNavMesh("graph.zip");
        float[] startPos = new float[] { 8.200293f, 2.155071f, -26.176147f };
        float[] endPos = new float[] { 11.971109f, 0.000000f, 8.663261f };
        Result<List<Long>> path = findPath(mesh, startPos, endPos);
        assertEquals(Status.SUCCSESS, path.status);
        assertEquals(57, path.result.size());
        saveMesh(mesh, "v4_0_6");
    }

    @Test
    public void test_v4_1_16() throws Exception {
        NavMesh mesh = loadNavMesh("graph_v4_1_16.zip");
        float[] startPos = new float[] { 22.93f, -2.37f, -5.11f };
        float[] endPos = new float[] { 16.81f, -2.37f, 25.52f };
        Result<List<Long>> path = findPath(mesh, startPos, endPos);
        assertTrue(path.status.isSuccess());
        assertEquals(15, path.result.size());
        saveMesh(mesh, "v4_1_16");
    }

    @Test
    public void testBoundsTree() throws Exception {
        NavMesh mesh = loadNavMesh("test_boundstree.zip");
        float[] position = { 387.52988f, 19.997f, 368.86282f };

        int[] tilePos = mesh.calcTileLoc(position);
        long tileRef = mesh.getTileRefAt(tilePos[0], tilePos[1], 0);
        MeshTile tile = mesh.getTileByRef(tileRef);
        MeshData data = tile.data;
        BVNode[] bvNodes = data.bvTree;
        data.bvTree = null; // set BV-Tree empty to get 'clear' search poly without BV
        FindNearestPolyResult clearResult = getNearestPolys(mesh, position)[0]; // check poly to exists

        // restore BV-Tree and try search again
        // important aspect in that test: BV result must equals result without BV
        // if poly not found or found other poly - tile bounds is wrong!
        data.bvTree = bvNodes;
        FindNearestPolyResult bvResult = getNearestPolys(mesh, position)[0];

        assertEquals(clearResult.getNearestRef(), bvResult.getNearestRef());
    }

    private NavMesh loadNavMesh(String filename) throws Exception {
        // Import the graphs
        UnityAStarPathfindingImporter importer = new UnityAStarPathfindingImporter();
        NavMesh[] meshes = importer.load(new File(ClassLoader.getSystemResource(filename).getFile()),
                MAX_VERTS_PER_POLY);
        return meshes[0];
    }

    private Result<List<Long>> findPath(NavMesh mesh, float[] startPos, float[] endPos) {
        // Perform a simple pathfinding
        NavMeshQuery query = new NavMeshQuery(mesh);
        QueryFilter filter = new DefaultQueryFilter();

        FindNearestPolyResult[] polys = getNearestPolys(mesh, startPos, endPos);
        return query.findPath(polys[0].getNearestRef(), polys[1].getNearestRef(), startPos, endPos, filter);
    }

    private FindNearestPolyResult[] getNearestPolys(NavMesh mesh, float[]... positions) {
        NavMeshQuery query = new NavMeshQuery(mesh);
        QueryFilter filter = new DefaultQueryFilter();
        float[] extents = new float[] { 0.1f, 0.1f, 0.1f };

        FindNearestPolyResult[] results = new FindNearestPolyResult[positions.length];
        for (int i = 0; i < results.length; i++) {
            float[] position = positions[i];
            Result<FindNearestPolyResult> result = query.findNearestPoly(position, extents, filter);
            assertTrue(result.succeeded());
            assertNotNull("Nearest start position is null!", result.result.getNearestPos());
            results[i] = result.result;
        }
        return results;
    }

    private void saveMesh(NavMesh mesh, String filePostfix) throws IOException {
        // Set the flag to RecastDemo work properly
        for (int i = 0; i < mesh.getTileCount(); i++) {
            for (Poly p : mesh.getTile(i).data.polys) {
                p.flags = 1;
            }
        }

        // Save the mesh as recast file,
        MeshSetWriter writer = new MeshSetWriter();
        FileOutputStream os = new FileOutputStream(String.format("all_tiles_navmesh_%s.bin", filePostfix));
        writer.write(os, mesh, ByteOrder.LITTLE_ENDIAN, true);
        os.close();
    }

}
