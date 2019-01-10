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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

public class NavMeshBuilderTest {

    private MeshData nmd;

    @Before
    public void setUp() {
        nmd = new RecastTestMeshBuilder().getMeshData();
    }

    @Test
    public void testBVTree() {
        assertEquals(225, nmd.verts.length / 3);
        assertEquals(119, nmd.polys.length);
        assertEquals(457, nmd.header.maxLinkCount);
        assertEquals(118, nmd.detailMeshes.length);
        assertEquals(291, nmd.detailTris.length / 4);
        assertEquals(60, nmd.detailVerts.length / 3);
        assertEquals(1, nmd.offMeshCons.length);
        assertEquals(118, nmd.header.offMeshBase);
        assertEquals(236, nmd.bvTree.length);
        assertTrue(nmd.header.bvNodeCount <= nmd.bvTree.length);
        for (int i = 0; i < nmd.header.bvNodeCount; i++) {
            assertNotNull(nmd.bvTree[i]);
        }
        for (int i = 0; i < 6; i++) {
            assertEquals(nmd.offMeshCons[0].pos[i], nmd.verts[223 * 3 + i], 0.0f);
        }
        assertEquals(0.1f, nmd.offMeshCons[0].rad, 0.0f);
        assertEquals(118, nmd.offMeshCons[0].poly);
        assertEquals(NavMesh.DT_OFFMESH_CON_BIDIR, nmd.offMeshCons[0].flags);
        assertEquals(0xFF, nmd.offMeshCons[0].side);
        assertEquals(0x4567, nmd.offMeshCons[0].userId);
        assertEquals(2, nmd.polys[118].vertCount);
        assertEquals(223, nmd.polys[118].verts[0]);
        assertEquals(224, nmd.polys[118].verts[1]);
        assertEquals(12, nmd.polys[118].flags);
        assertEquals(2, nmd.polys[118].getArea());
        assertEquals(Poly.DT_POLYTYPE_OFFMESH_CONNECTION, nmd.polys[118].getType());

    }
}