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

import org.junit.Assert;
import org.junit.Test;
import org.recast4j.detour.NavMeshQuery.FRand;

public class RandomPointTest extends AbstractDetourTest {

    @Test
    public void testRandom() {
        FRand f = new FRand();
        QueryFilter filter = new DefaultQueryFilter();
        for (int i = 0; i < 1000; i++) {
            Result<FindRandomPointResult> point = query.findRandomPoint(filter, f);
            Assert.assertTrue(point.succeeded());
            Tupple2<MeshTile, Poly> tileAndPoly = navmesh.getTileAndPolyByRef(point.result.getRandomRef()).result;
            float[] bmin = new float[2];
            float[] bmax = new float[2];
            for (int j = 0; j < tileAndPoly.second.vertCount; j++) {
                int v = tileAndPoly.second.verts[j] * 3;
                bmin[0] = j == 0 ? tileAndPoly.first.data.verts[v] : Math.min(bmin[0], tileAndPoly.first.data.verts[v]);
                bmax[0] = j == 0 ? tileAndPoly.first.data.verts[v] : Math.max(bmax[0], tileAndPoly.first.data.verts[v]);
                bmin[1] = j == 0 ? tileAndPoly.first.data.verts[v + 2]
                        : Math.min(bmin[1], tileAndPoly.first.data.verts[v + 2]);
                bmax[1] = j == 0 ? tileAndPoly.first.data.verts[v + 2]
                        : Math.max(bmax[1], tileAndPoly.first.data.verts[v + 2]);
            }
            Assert.assertTrue(point.result.getRandomPt()[0] >= bmin[0]);
            Assert.assertTrue(point.result.getRandomPt()[0] <= bmax[0]);
            Assert.assertTrue(point.result.getRandomPt()[2] >= bmin[1]);
            Assert.assertTrue(point.result.getRandomPt()[2] <= bmax[1]);
        }
    }

    @Test
    public void testRandomInCircle() {
        FRand f = new FRand();
        QueryFilter filter = new DefaultQueryFilter();
        FindRandomPointResult point = query.findRandomPoint(filter, f).result;
        for (int i = 0; i < 1000; i++) {
            Result<FindRandomPointResult> result = query.findRandomPointAroundCircle(point.getRandomRef(),
                    point.getRandomPt(), 5f, filter, f);
            Assert.assertFalse(result.failed());
            point = result.result;
            Tupple2<MeshTile, Poly> tileAndPoly = navmesh.getTileAndPolyByRef(point.getRandomRef()).result;
            float[] bmin = new float[2];
            float[] bmax = new float[2];
            for (int j = 0; j < tileAndPoly.second.vertCount; j++) {
                int v = tileAndPoly.second.verts[j] * 3;
                bmin[0] = j == 0 ? tileAndPoly.first.data.verts[v] : Math.min(bmin[0], tileAndPoly.first.data.verts[v]);
                bmax[0] = j == 0 ? tileAndPoly.first.data.verts[v] : Math.max(bmax[0], tileAndPoly.first.data.verts[v]);
                bmin[1] = j == 0 ? tileAndPoly.first.data.verts[v + 2]
                        : Math.min(bmin[1], tileAndPoly.first.data.verts[v + 2]);
                bmax[1] = j == 0 ? tileAndPoly.first.data.verts[v + 2]
                        : Math.max(bmax[1], tileAndPoly.first.data.verts[v + 2]);
            }
            Assert.assertTrue(point.getRandomPt()[0] >= bmin[0]);
            Assert.assertTrue(point.getRandomPt()[0] <= bmax[0]);
            Assert.assertTrue(point.getRandomPt()[2] >= bmin[1]);
            Assert.assertTrue(point.getRandomPt()[2] <= bmax[1]);
        }
    }
}
