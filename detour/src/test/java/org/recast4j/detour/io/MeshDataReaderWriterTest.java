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

import static org.assertj.core.api.Assertions.assertThat;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.RecastTestMeshBuilder;

public class MeshDataReaderWriterTest {

    private static final int VERTS_PER_POLYGON = 6;
    private MeshData meshData;

    @BeforeEach
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

        assertThat(readData.header.vertCount).isEqualTo(meshData.header.vertCount);
        assertThat(readData.header.polyCount).isEqualTo(meshData.header.polyCount);
        assertThat(readData.header.detailMeshCount).isEqualTo(meshData.header.detailMeshCount);
        assertThat(readData.header.detailTriCount).isEqualTo(meshData.header.detailTriCount);
        assertThat(readData.header.detailVertCount).isEqualTo(meshData.header.detailVertCount);
        assertThat(readData.header.bvNodeCount).isEqualTo(meshData.header.bvNodeCount);
        assertThat(readData.header.offMeshConCount).isEqualTo(meshData.header.offMeshConCount);
        for (int i = 0; i < meshData.header.vertCount; i++) {
            assertThat(readData.verts[i]).isEqualTo(meshData.verts[i]);
        }
        for (int i = 0; i < meshData.header.polyCount; i++) {
            assertThat(readData.polys[i].vertCount).isEqualTo(meshData.polys[i].vertCount);
            assertThat(readData.polys[i].areaAndtype).isEqualTo(meshData.polys[i].areaAndtype);
            for (int j = 0; j < meshData.polys[i].vertCount; j++) {
                assertThat(readData.polys[i].verts[j]).isEqualTo(meshData.polys[i].verts[j]);
                assertThat(readData.polys[i].neis[j]).isEqualTo(meshData.polys[i].neis[j]);
            }
        }
        for (int i = 0; i < meshData.header.detailMeshCount; i++) {
            assertThat(readData.detailMeshes[i].vertBase).isEqualTo(meshData.detailMeshes[i].vertBase);
            assertThat(readData.detailMeshes[i].vertCount).isEqualTo(meshData.detailMeshes[i].vertCount);
            assertThat(readData.detailMeshes[i].triBase).isEqualTo(meshData.detailMeshes[i].triBase);
            assertThat(readData.detailMeshes[i].triCount).isEqualTo(meshData.detailMeshes[i].triCount);
        }
        for (int i = 0; i < meshData.header.detailVertCount; i++) {
            assertThat(readData.detailVerts[i]).isEqualTo(meshData.detailVerts[i]);
        }
        for (int i = 0; i < meshData.header.detailTriCount; i++) {
            assertThat(readData.detailTris[i]).isEqualTo(meshData.detailTris[i]);
        }
        for (int i = 0; i < meshData.header.bvNodeCount; i++) {
            assertThat(readData.bvTree[i].i).isEqualTo(meshData.bvTree[i].i);
            for (int j = 0; j < 3; j++) {
                assertThat(readData.bvTree[i].bmin[j]).isEqualTo(meshData.bvTree[i].bmin[j]);
                assertThat(readData.bvTree[i].bmax[j]).isEqualTo(meshData.bvTree[i].bmax[j]);
            }
        }
        for (int i = 0; i < meshData.header.offMeshConCount; i++) {
            assertThat(readData.offMeshCons[i].flags).isEqualTo(meshData.offMeshCons[i].flags);
            assertThat(readData.offMeshCons[i].rad).isEqualTo(meshData.offMeshCons[i].rad);
            assertThat(readData.offMeshCons[i].poly).isEqualTo(meshData.offMeshCons[i].poly);
            assertThat(readData.offMeshCons[i].side).isEqualTo(meshData.offMeshCons[i].side);
            assertThat(readData.offMeshCons[i].userId).isEqualTo(meshData.offMeshCons[i].userId);
            for (int j = 0; j < 6; j++) {
                assertThat(readData.offMeshCons[i].pos[j]).isEqualTo(meshData.offMeshCons[i].pos[j]);
            }
        }
    }
}
