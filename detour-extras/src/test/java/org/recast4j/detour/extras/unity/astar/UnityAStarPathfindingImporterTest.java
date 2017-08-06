package org.recast4j.detour.extras.unity.astar;

import static org.junit.Assert.assertEquals;

import java.io.File;
import java.io.FileOutputStream;
import java.nio.ByteOrder;

import org.junit.Test;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.FindPathResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Poly;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.io.MeshSetWriter;

public class UnityAStarPathfindingImporterTest {

	/**
	 * 3 is enough for recast4j as all nodes created by A* Star Pathfinding are triangles. However 6 can be used to to
	 * keep the result compatible with RecastDemo.
	 */
	static final int MAX_VERTS_PER_POLY = 6;

	@Test
	public void test() throws Exception {
		// Import the graphs 
		UnityAStarPathfindingImporter importer = new UnityAStarPathfindingImporter();
		NavMesh[] meshes = importer.load(new File(ClassLoader.getSystemResource("graph.zip").getFile()),
				MAX_VERTS_PER_POLY);
		NavMesh mesh = meshes[0];
		// Perform a simple pathfinding
		NavMeshQuery query = new NavMeshQuery(mesh);
		QueryFilter filter = new QueryFilter();
		float[] startPos = new float[] { 8.200293f, 2.155071f, -26.176147f };
		float[] endPos = new float[] { 11.971109f, 0.000000f, 8.663261f };
		FindNearestPolyResult start = query.findNearestPoly(startPos, new float[] { 0.1f, 0.1f, 0.1f }, filter);
		FindNearestPolyResult end = query.findNearestPoly(endPos, new float[] { 0.1f, 0.1f, 0.1f }, filter);
		FindPathResult path = query.findPath(start.getNearestRef(), end.getNearestRef(), startPos, endPos, filter);
		assertEquals(57, path.getRefs().size());
		// Set the flag to RecastDemo work properly
		for (int i = 0; i < mesh.getTileCount();i++) {
			for (Poly p : mesh.getTile(i).data.polys) {
				p.flags = 1;
			}
		}
		// Save the mesh as recast file, 
		MeshSetWriter writer = new MeshSetWriter();
		FileOutputStream os = new FileOutputStream("all_tiles_navmesh.bin");
		writer.write(os, mesh, ByteOrder.LITTLE_ENDIAN, true);
		os.close();
	}

}
