/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2026 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

/**
 * Tests for Recast filter functions ported from upstream C++ tests. See:
 * https://github.com/recastnavigation/recastnavigation/blob/main/Tests/Recast/Tests_RecastFilter.cpp
 */
public class RecastFilterTest {

    private static final int RC_NULL_AREA = 0;

    @Test
    public void testFilterLowHangingWalkableObstacles() {
        Telemetry context = new Telemetry();
        int walkableHeight = 5;

        Heightfield heightfield = new Heightfield(1, 1, new float[] { 0, 0, 0 }, new float[] { 1, 1, 1 }, 1, 1, 0);

        // Span with no spans above it is unchanged
        {
            Span span = new Span();
            span.area = 1;
            span.next = null;
            span.smin = 0;
            span.smax = 1;
            heightfield.spans[0] = span;

            RecastFilter.filterLowHangingWalkableObstacles(context, walkableHeight, heightfield);

            assertThat(heightfield.spans[0].area).isEqualTo(1);
        }

        // Span with span above that is higher than walkableHeight is unchanged
        {
            heightfield.spans[0] = null;

            Span secondSpan = new Span();
            secondSpan.area = RC_NULL_AREA;
            secondSpan.next = null;
            secondSpan.smin = 1 + walkableHeight;
            secondSpan.smax = secondSpan.smin + 1;

            Span span = new Span();
            span.area = 1;
            span.next = secondSpan;
            span.smin = 0;
            span.smax = 1;

            heightfield.spans[0] = span;

            RecastFilter.filterLowHangingWalkableObstacles(context, walkableHeight, heightfield);

            // Check that nothing has changed.
            assertThat(heightfield.spans[0].area).isEqualTo(1);
            assertThat(heightfield.spans[0].next.area).isEqualTo(RC_NULL_AREA);

            // Check again but with more clearance
            secondSpan.smin += 10;
            secondSpan.smax += 10;

            RecastFilter.filterLowHangingWalkableObstacles(context, walkableHeight, heightfield);

            // Check that nothing has changed.
            assertThat(heightfield.spans[0].area).isEqualTo(1);
            assertThat(heightfield.spans[0].next.area).isEqualTo(RC_NULL_AREA);
        }

        // Marks low obstacles walkable if they're below the walkableHeight
        {
            heightfield.spans[0] = null;

            Span secondSpan = new Span();
            secondSpan.area = RC_NULL_AREA;
            secondSpan.next = null;
            secondSpan.smin = 1 + (walkableHeight - 1);
            secondSpan.smax = secondSpan.smin + 1;

            Span span = new Span();
            span.area = 1;
            span.next = secondSpan;
            span.smin = 0;
            span.smax = 1;

            heightfield.spans[0] = span;

            RecastFilter.filterLowHangingWalkableObstacles(context, walkableHeight, heightfield);

            // Check that the second span was changed to walkable.
            assertThat(heightfield.spans[0].area).isEqualTo(1);
            assertThat(heightfield.spans[0].next.area).isEqualTo(1);
        }

        // Low obstacle that overlaps the walkableHeight distance is not changed
        {
            heightfield.spans[0] = null;

            Span secondSpan = new Span();
            secondSpan.area = RC_NULL_AREA;
            secondSpan.next = null;
            secondSpan.smin = 2 + (walkableHeight - 1);
            secondSpan.smax = secondSpan.smin + 1;

            Span span = new Span();
            span.area = 1;
            span.next = secondSpan;
            span.smin = 0;
            span.smax = 1;

            heightfield.spans[0] = span;

            RecastFilter.filterLowHangingWalkableObstacles(context, walkableHeight, heightfield);

            // Check that the second span was not changed.
            assertThat(heightfield.spans[0].area).isEqualTo(1);
            assertThat(heightfield.spans[0].next.area).isEqualTo(RC_NULL_AREA);
        }
    }

    @Test
    public void testFilterLedgeSpans() {
        Telemetry context = new Telemetry();
        int walkableClimb = 5;
        int walkableHeight = 10;

        Heightfield heightfield = new Heightfield(10, 10, new float[] { 0, 0, 0 }, new float[] { 10, 1, 10 }, 1, 1, 0);

        // Edge spans are marked unwalkable
        {
            // Create a flat plane.
            for (int x = 0; x < heightfield.width; ++x) {
                for (int z = 0; z < heightfield.height; ++z) {
                    Span span = new Span();
                    span.area = 1;
                    span.next = null;
                    span.smin = 0;
                    span.smax = 1;
                    heightfield.spans[x + z * heightfield.width] = span;
                }
            }

            RecastFilter.filterLedgeSpans(context, walkableHeight, walkableClimb, heightfield);

            for (int x = 0; x < heightfield.width; ++x) {
                for (int z = 0; z < heightfield.height; ++z) {
                    Span span = heightfield.spans[x + z * heightfield.width];
                    assertThat(span).isNotNull();

                    if (x == 0 || z == 0 || x == 9 || z == 9) {
                        assertThat(span.area).isEqualTo(RC_NULL_AREA)
                                .describedAs("Edge span at (" + x + ", " + z + ") should be marked unwalkable");
                    } else {
                        assertThat(span.area).isEqualTo(1)
                                .describedAs("Interior span at (" + x + ", " + z + ") should remain walkable");
                    }

                    assertThat(span.next).isNull();
                    assertThat(span.smin).isEqualTo(0);
                    assertThat(span.smax).isEqualTo(1);
                }
            }
        }
    }

    @Test
    public void testFilterWalkableLowHeightSpans() {
        Telemetry context = new Telemetry();
        int walkableHeight = 5;

        Heightfield heightfield = new Heightfield(1, 1, new float[] { 0, 0, 0 }, new float[] { 1, 1, 1 }, 1, 1, 0);

        // Span with nothing above is unchanged
        {
            Span span = new Span();
            span.area = 1;
            span.next = null;
            span.smin = 0;
            span.smax = 1;
            heightfield.spans[0] = span;

            RecastFilter.filterWalkableLowHeightSpans(context, walkableHeight, heightfield);

            assertThat(heightfield.spans[0].area).isEqualTo(1);
        }

        // Span with lots of room above is unchanged
        {
            heightfield.spans[0] = null;

            Span overheadSpan = new Span();
            overheadSpan.area = RC_NULL_AREA;
            overheadSpan.next = null;
            overheadSpan.smin = 10;
            overheadSpan.smax = 11;

            Span span = new Span();
            span.area = 1;
            span.next = overheadSpan;
            span.smin = 0;
            span.smax = 1;
            heightfield.spans[0] = span;

            RecastFilter.filterWalkableLowHeightSpans(context, walkableHeight, heightfield);

            assertThat(heightfield.spans[0].area).isEqualTo(1);
            assertThat(heightfield.spans[0].next.area).isEqualTo(RC_NULL_AREA);
        }

        // Span with low hanging obstacle is marked as unwalkable
        {
            heightfield.spans[0] = null;

            Span overheadSpan = new Span();
            overheadSpan.area = RC_NULL_AREA;
            overheadSpan.next = null;
            overheadSpan.smin = 3;
            overheadSpan.smax = 4;

            Span span = new Span();
            span.area = 1;
            span.next = overheadSpan;
            span.smin = 0;
            span.smax = 1;
            heightfield.spans[0] = span;

            RecastFilter.filterWalkableLowHeightSpans(context, walkableHeight, heightfield);

            assertThat(heightfield.spans[0].area).isEqualTo(RC_NULL_AREA);
            assertThat(heightfield.spans[0].next.area).isEqualTo(RC_NULL_AREA);
        }
    }
}
