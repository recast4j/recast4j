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
import static org.assertj.core.data.Offset.offset;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class FindNearestPolyTest extends AbstractDetourTest {

    private static final long[] POLY_REFS = { 281474976710696L, 281474976710773L, 281474976710680L, 281474976710753L, 281474976710733L };
    private static final float[][] POLY_POS = { { 22.606520f, 10.197294f, -45.918674f }, { 22.331268f, 10.197294f, -1.040187f },
            { 18.694363f, 15.803535f, -73.090416f }, { 0.745335f, 10.197294f, -5.940050f },
            { -20.651257f, 5.904126f, -13.712508f } };

    @Test
    public void testFindNearestPoly() {
        QueryFilter filter = new DefaultQueryFilter();
        float[] extents = { 2, 4, 2 };
        for (int i = 0; i < startRefs.length; i++) {
            float[] startPos = startPoss[i];
            Result<FindNearestPolyResult> poly = query.findNearestPoly(startPos, extents, filter);
            assertThat(poly.succeeded()).isTrue();
            assertThat(poly.result.getNearestRef()).isEqualTo(POLY_REFS[i]);
            for (int v = 0; v < POLY_POS[i].length; v++) {
                assertThat(poly.result.getNearestPos()[v]).isEqualTo(POLY_POS[i][v], offset(0.001f));
            }
        }

    }

    @Test
    public void shouldReturnStartPosWhenNoPolyIsValid() {
        QueryFilter filter = new QueryFilter() {
            @Override
            public boolean passFilter(long ref, MeshTile tile, Poly poly) {
                return false;
            }

            @Override
            public float getCost(float[] pa, float[] pb, long prevRef, MeshTile prevTile, Poly prevPoly, long curRef,
                    MeshTile curTile, Poly curPoly, long nextRef, MeshTile nextTile, Poly nextPoly) {
                return 0;
            }
        };
        float[] extents = { 2, 4, 2 };
        for (int i = 0; i < startRefs.length; i++) {
            float[] startPos = startPoss[i];
            Result<FindNearestPolyResult> poly = query.findNearestPoly(startPos, extents, filter);
            assertThat(poly.succeeded()).isTrue();
            assertEquals(0L, poly.result.getNearestRef());
            for (int v = 0; v < POLY_POS[i].length; v++) {
                assertThat(poly.result.getNearestPos()[v]).isEqualTo(startPos[v], offset(0.001f));
            }
        }

    }
}
