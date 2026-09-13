/*
recast4j copyright (c) 2021-2026 Piotr Piastucki piotr@recast4j.org

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

import java.util.Arrays;

import org.assertj.core.data.Offset;
import org.junit.jupiter.api.Test;

public class ConvexConvexIntersectionTest {

    @Test
    public void shouldHandleSamePolygonIntersection() {
        float[] p = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] q = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] expected = { -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4, -4, 0, 0 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleIntersection() {
        float[] p = { -5, 0, -5, -5, 0, 4, 1, 0, 4, 1, 0, -5 };
        float[] q = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] expected = { 1, 0, -3.4f, -2, 0, -4, -4, 0, 0, -3, 0, 3, 1, 0, 3 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandlePartialOverlap() {
        float[] p = { 0, 0, 2, 2, 0, 2, 2, 0, 0, 0, 0, 0 };
        float[] q = { 1, 0, 3, 3, 0, 3, 3, 0, 1, 1, 0, 1 };
        float[] expected = { 1, 0, 2, 2, 0, 2, 2, 0, 1, 1, 0, 1 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleOneInsideAnother() {
        float[] p = { 0, 0, 10, 10, 0, 10, 10, 0, 0, 0, 0, 0 };
        float[] q = { 2, 0, 3, 3, 0, 3, 3, 0, 2, 2, 0, 2 };
        float[] expected = { 2, 0, 3, 3, 0, 3, 3, 0, 2, 2, 0, 2 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleTouchingAtCorner() {
        float[] p = { 0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2 };
        float[] q = { 2, 0, 2, 4, 0, 2, 4, 0, 4, 2, 0, 4 };
        float[] expected = null;
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleCollinearEdgeOverlap() {
        float[] p = { 0, 0, 3, 3, 0, 3, 3, 0, 0, 0, 0, 0 };
        float[] q = { 2, 0, 5, 5, 0, 5, 5, 0, 0, 2, 0, 0 };
        float[] expected = {2, 0, 3, 3, 0, 3, 3, 0, 0, 2, 0, 0};
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleRotatedSquareIntersection() {
        float[] p = { 0, 0, 4, 4, 0, 4, 4, 0, 0, 0, 0, 0 };
        float[] q = { -2, 0, -2, -2, 0, 6, 2, 0, 6, 2, 0, -2 };
        float[] expected = {0, 0, 4, 2, 0, 4, 2, 0, 0, 0, 0, 0};
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleNoIntersection() {
        float[] p = { 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1 };
        float[] q = { 5, 0, 5, 6, 0, 5, 6, 0, 6, 5, 0, 6 };
        float[] expected = null;
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleSamePolygonIntersectionWithDifferentStartVertex() {
        float[] p = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] q = { 3, 0, -3, -2, 0, -4, -4, 0, 0, -3, 0, 3, 2, 0, 3 };
        float[] expected = {-3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4, -4, 0, 0};
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void performanceTest() {
        // Regular 12-vertex polygon (large, radius 10)
        float[] p = { 10, 0, 0, 8.66f, 0, 5, 5, 0, 8.66f, 0, 0, 10, -5, 0, 8.66f, -8.66f, 0, 5, -10, 0, 0, -8.66f, 0, -5, -5, 0,
                -8.66f, 0, 0, -10, 5, 0, -8.66f, 8.66f, 0, -5 };

        // 6-vertex polygon (small, with 4 vertices inside p)
        float[] q = { -3, 0, -2, 2, 0, -3, 4, 0, 1, 3, 0, 5, -1, 0, 6, -5, 0, 2 };

        final int WARMUP_ITERATIONS = 50000;
        final int MEASURE_ITERATIONS = 500000;

        System.out.println("\n=== ConvexPolygonIntersector ===");
        for (int i = 0; i < WARMUP_ITERATIONS; i++) {
            ConvexPolygonIntersector.intersect(p, q);
        }

        long startTime = System.nanoTime();
        for (int i = 0; i < MEASURE_ITERATIONS; i++) {
            ConvexPolygonIntersector.intersect(p, q);
        }
        long elapsedTime = System.nanoTime() - startTime;
        double elapsedMillis = elapsedTime / 1_000_000.0;
        System.out.printf("Time for %d iterations: %.2f ms (%.4f ms per iteration)%n", MEASURE_ITERATIONS, elapsedMillis,
                elapsedMillis / MEASURE_ITERATIONS);

    }

    private static void assertResultsMatch(float[] p, float[] q, float[] expected) {
        float[] actual = ConvexPolygonIntersector.intersect(p, q);

        if (expected == null) {
            assertThat(actual).isNull();
            return;
        }

        assertThat(actual).isNotNull();
        assertThat(normalize(actual)).containsExactly(normalize(expected), Offset.offset(0.0001f));
    }

    private static float[] normalize(float[] polygon) {
        if (polygon == null) {
            return null;
        }
        int vertexCount = polygon.length / 3;
        if (vertexCount <= 1) {
            return Arrays.copyOf(polygon, polygon.length);
        }

        int startIndex = 0;
        for (int i = 1; i < vertexCount; i++) {
            if (compareVertices(polygon, i, startIndex) < 0) {
                startIndex = i;
            }
        }

        float[] normalized = new float[polygon.length];
        for (int i = 0; i < vertexCount; i++) {
            int sourceIndex = (startIndex + i) % vertexCount;
            normalized[3 * i] = polygon[3 * sourceIndex];
            normalized[3 * i + 1] = polygon[3 * sourceIndex + 1];
            normalized[3 * i + 2] = polygon[3 * sourceIndex + 2];
        }
        return normalized;
    }

    private static int compareVertices(float[] polygon, int firstIndex, int secondIndex) {
        int firstOffset = 3 * firstIndex;
        int secondOffset = 3 * secondIndex;
        int xComparison = Float.compare(polygon[firstOffset], polygon[secondOffset]);
        if (xComparison != 0) {
            return xComparison;
        }
        int zComparison = Float.compare(polygon[firstOffset + 2], polygon[secondOffset + 2]);
        if (zComparison != 0) {
            return zComparison;
        }
        return Float.compare(polygon[firstOffset + 1], polygon[secondOffset + 1]);
    }
}
