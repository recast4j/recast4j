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

import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class TiledFindPathTest {

    private final Status[] statuses = { Status.SUCCSESS };
    private final long[][] results = { { 281475015507969L, 281475014459393L, 281475014459392L, 281475013410816L,
            281475012362240L, 281475012362241L, 281475012362242L, 281475003973634L, 281475003973635L, 281475003973633L,
            281475002925059L, 281475002925057L, 281475002925056L, 281474998730753L, 281474998730754L, 281474994536450L,
            281474994536451L, 281474994536452L, 281474994536448L, 281474990342146L, 281474990342145L, 281474991390723L,
            281474991390724L, 281474991390725L, 281474992439298L, 281474992439300L, 281474992439299L, 281474992439297L,
            281474988244996L, 281474988244995L, 281474988244997L, 281474985099266L } };
    protected final long[] startRefs = { 281475015507969L };
    protected final long[] endRefs = { 281474985099266L };
    protected final float[][] startPoss = { { 39.447338f, 9.998177f, -0.784811f } };
    protected final float[][] endPoss = { { 19.292645f, 11.611748f, -57.750366f } };

    protected NavMeshQuery query;
    protected NavMesh navmesh;

    @Before
    public void setUp() {
        navmesh = createNavMesh();
        query = new NavMeshQuery(navmesh);
    }

    protected NavMesh createNavMesh() {
        return new TestTiledNavMeshBuilder().getNavMesh();
    }

    @Test
    public void testFindPath() {
        QueryFilter filter = new DefaultQueryFilter();
        for (int i = 0; i < startRefs.length; i++) {
            long startRef = startRefs[i];
            long endRef = endRefs[i];
            float[] startPos = startPoss[i];
            float[] endPos = endPoss[i];
            Result<List<Long>> path = query.findPath(startRef, endRef, startPos, endPos, filter);
            Assert.assertEquals(statuses[i], path.status);
            Assert.assertEquals(results[i].length, path.result.size());
            for (int j = 0; j < results[i].length; j++) {
                Assert.assertEquals(results[i][j], path.result.get(j).longValue());
            }
        }
    }
}
