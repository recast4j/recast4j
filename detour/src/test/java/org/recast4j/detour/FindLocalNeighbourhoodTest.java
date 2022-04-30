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

import org.junit.jupiter.api.Test;

public class FindLocalNeighbourhoodTest extends AbstractDetourTest {

    private static final long[][] REFS = { { 281474976710696L, 281474976710695L, 281474976710691L, 281474976710697L },
            { 281474976710773L, 281474976710769L, 281474976710772L },
            { 281474976710680L, 281474976710674L, 281474976710679L, 281474976710684L, 281474976710683L,
                    281474976710678L, 281474976710677L, 281474976710676L },
            { 281474976710753L, 281474976710748L, 281474976710750L, 281474976710752L },
            { 281474976710733L, 281474976710735L, 281474976710736L }

    };
    private static final long[][] PARENT_REFS = { { 0L, 281474976710696L, 281474976710695L, 281474976710695L },
            { 0L, 281474976710773L, 281474976710773L },
            { 0L, 281474976710680L, 281474976710680L, 281474976710680L, 281474976710680L, 281474976710679L,
                    281474976710683L, 281474976710678L },
            { 0L, 281474976710753L, 281474976710753L, 281474976710748L }, { 0L, 281474976710733L, 281474976710733L } };

    @Test
    public void testFindNearestPoly() {
        QueryFilter filter = new DefaultQueryFilter();
        for (int i = 0; i < startRefs.length; i++) {
            float[] startPos = startPoss[i];
            Result<FindLocalNeighbourhoodResult> poly = query.findLocalNeighbourhood(startRefs[i], startPos, 3.5f,
                    filter);
            assertThat(poly.result.getRefs()).hasSize(REFS[i].length);
            for (int v = 0; v < REFS[i].length; v++) {
                assertThat(poly.result.getRefs().get(v).longValue()).isEqualTo(REFS[i][v]);
            }
        }

    }

}
