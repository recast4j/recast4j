package org.recast4j.detour.io;

import static org.junit.Assert.assertEquals;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;

import org.junit.Before;
import org.junit.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.RecastTestMeshBuilder;

public class MeshReaderWriterTest {

	private MeshData meshData;

	@Before
	public void setUp() {
		RecastTestMeshBuilder rcBuilder = new RecastTestMeshBuilder();
		meshData = rcBuilder.getMeshData();
		// meshData.offMeshCons
	}

	@Test
	public void testCCompatibility() throws IOException {
		test(true);
	}

	@Test
	public void testCompact() throws IOException {
		test(false);
	}

	public void test(boolean cCompatibility) throws IOException {
		ByteArrayOutputStream os = new ByteArrayOutputStream();
		MeshWriter writer = new MeshWriter();
		writer.write(os, meshData, ByteOrder.BIG_ENDIAN, cCompatibility);
		ByteArrayInputStream bais = new ByteArrayInputStream(os.toByteArray());
		MeshReader reader = new MeshReader();
		MeshData readData = reader.read(bais, ByteOrder.BIG_ENDIAN, cCompatibility);

		System.out.println("verts: " + meshData.header.vertCount);
		System.out.println("polys: " + meshData.header.polyCount);
		System.out.println("detail vert: " + meshData.header.detailVertCount);
		System.out.println("detail mesh: " + meshData.header.detailMeshCount);
		assertEquals(meshData.header.vertCount, readData.header.vertCount);
		assertEquals(meshData.header.polyCount, readData.header.polyCount);
		assertEquals(meshData.header.detailMeshCount, readData.header.detailMeshCount);
		assertEquals(meshData.header.detailTriCount, readData.header.detailTriCount);
		assertEquals(meshData.header.detailVertCount, readData.header.detailVertCount);
		assertEquals(meshData.header.bvNodeCount, readData.header.bvNodeCount);
		assertEquals(meshData.header.offMeshConCount, readData.header.offMeshConCount);
		for (int i = 0; i < meshData.header.vertCount; i++) {
			assertEquals(meshData.verts[i], readData.verts[i], 0.0f);
		}
		for (int i = 0; i < meshData.header.polyCount; i++) {
			assertEquals(meshData.polys[i].firstLink, readData.polys[i].firstLink);
			assertEquals(meshData.polys[i].vertCount, readData.polys[i].vertCount);
			assertEquals(meshData.polys[i].areaAndtype, readData.polys[i].areaAndtype);
			for (int j = 0; j < meshData.polys[i].vertCount; j++) {
				assertEquals(meshData.polys[i].verts[j], readData.polys[i].verts[j]);
				assertEquals(meshData.polys[i].neis[j], readData.polys[i].neis[j]);
			}
		}
		for (int i = 0; i < meshData.header.detailMeshCount; i++) {
			assertEquals(meshData.detailMeshes[i].vertBase, readData.detailMeshes[i].vertBase);
			assertEquals(meshData.detailMeshes[i].vertCount, readData.detailMeshes[i].vertCount);
			assertEquals(meshData.detailMeshes[i].triBase, readData.detailMeshes[i].triBase);
			assertEquals(meshData.detailMeshes[i].triCount, readData.detailMeshes[i].triCount);
		}
		for (int i = 0; i < meshData.header.detailVertCount; i++) {
			assertEquals(meshData.detailVerts[i], readData.detailVerts[i], 0.0f);
		}
		for (int i = 0; i < meshData.header.detailTriCount; i++) {
			assertEquals(meshData.detailTris[i], readData.detailTris[i]);
		}
		for (int i = 0; i < meshData.header.bvNodeCount; i++) {
			assertEquals(meshData.bvTree[i].i, readData.bvTree[i].i);
			for (int j = 0; j < 3; j++) {
				assertEquals(meshData.bvTree[i].bmin[j], readData.bvTree[i].bmin[j]);
				assertEquals(meshData.bvTree[i].bmax[j], readData.bvTree[i].bmax[j]);
			}
		}
		for (int i = 0; i < meshData.header.offMeshConCount; i++) {
			assertEquals(meshData.offMeshCons[i].flags, readData.offMeshCons[i].flags);
			assertEquals(meshData.offMeshCons[i].rad, readData.offMeshCons[i].rad, 0.0f);
			assertEquals(meshData.offMeshCons[i].poly, readData.offMeshCons[i].poly);
			assertEquals(meshData.offMeshCons[i].side, readData.offMeshCons[i].side);
			assertEquals(meshData.offMeshCons[i].userId, readData.offMeshCons[i].userId);
			for (int j = 0; j < 6; j++) {
				assertEquals(meshData.offMeshCons[i].pos[j], readData.offMeshCons[i].pos[j], 0.0f);
			}
		}
	}
}
