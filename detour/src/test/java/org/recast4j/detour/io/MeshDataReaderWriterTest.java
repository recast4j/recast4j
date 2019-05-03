/*
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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

public class MeshDataReaderWriterTest {

    private static final int VERTS_PER_POLYGON = 6;
    private MeshData meshData;

    @Before
    public void setUp() {
        RecastTestMeshBuilder rcBuilder = new RecastTestMeshBuilder();
        meshData = rcBuilder.getMeshData();
    }

    @Test
    public void testCCompatibility() throws IOException {
        test(true, ByteOrder.BIG_ENDIAN);
    }

    @Test
    public void testCompact() throws IOException {
        test(false, ByteOrder.BIG_ENDIAN);
    }

    @Test
    public void testCCompatibilityLE() throws IOException {
        test(true, ByteOrder.LITTLE_ENDIAN);
    }

    @Test
    public void testCompactLE() throws IOException {
        test(false, ByteOrder.LITTLE_ENDIAN);
    }

    public void test(boolean cCompatibility, ByteOrder order) throws IOException {
        ByteArrayOutputStream os = new ByteArrayOutputStream();
        MeshDataWriter writer = new MeshDataWriter();
        writer.write(os, meshData, order, cCompatibility);
        ByteArrayInputStream bais = new ByteArrayInputStream(os.toByteArray());
        MeshDataReader reader = new MeshDataReader();
        MeshData readData = reader.read(bais, VERTS_PER_POLYGON);

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
