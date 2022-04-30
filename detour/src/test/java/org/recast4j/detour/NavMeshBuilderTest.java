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
package org.recast4j.detour;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class NavMeshBuilderTest {

    private MeshData nmd;

    @BeforeEach
    public void setUp() {
        nmd = new RecastTestMeshBuilder().getMeshData();
    }

    @Test
    public void testBVTree() {
        assertThat(nmd.verts.length / 3).isEqualTo(225);
        assertThat(nmd.polys.length).isEqualTo(119);
        assertThat(nmd.header.maxLinkCount).isEqualTo(457);
        assertThat(nmd.detailMeshes.length).isEqualTo(118);
        assertThat(nmd.detailTris.length / 4).isEqualTo(291);
        assertThat(nmd.detailVerts.length / 3).isEqualTo(60);
        assertThat(nmd.offMeshCons.length).isEqualTo(1);
        assertThat(nmd.header.offMeshBase).isEqualTo(118);
        assertThat(nmd.bvTree.length).isEqualTo(236);
        assertThat(nmd.bvTree.length).isGreaterThanOrEqualTo(nmd.header.bvNodeCount);
        for (int i = 0; i < nmd.header.bvNodeCount; i++) {
            assertThat(nmd.bvTree[i]).isNotNull();
        }
        for (int i = 0; i < 6; i++) {
            assertThat(nmd.verts[223 * 3 + i]).isEqualTo(nmd.offMeshCons[0].pos[i]);
        }
        assertThat(nmd.offMeshCons[0].rad).isEqualTo(0.1f);
        assertThat(nmd.offMeshCons[0].poly).isEqualTo(118);
        assertThat(nmd.offMeshCons[0].flags).isEqualTo(NavMesh.DT_OFFMESH_CON_BIDIR);
        assertThat(nmd.offMeshCons[0].side).isEqualTo(0xFF);
        assertThat(nmd.offMeshCons[0].userId).isEqualTo(0x4567);
        assertThat(nmd.polys[118].vertCount).isEqualTo(2);
        assertThat(nmd.polys[118].verts[0]).isEqualTo(223);
        assertThat(nmd.polys[118].verts[1]).isEqualTo(224);
        assertThat(nmd.polys[118].flags).isEqualTo(12);
        assertThat(nmd.polys[118].getArea()).isEqualTo(2);
        assertThat(nmd.polys[118].getType()).isEqualTo(Poly.DT_POLYTYPE_OFFMESH_CONNECTION);

    }
}