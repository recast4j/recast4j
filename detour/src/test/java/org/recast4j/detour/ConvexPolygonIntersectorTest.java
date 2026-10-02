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
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.Arrays;

import org.assertj.core.data.Offset;
import org.junit.jupiter.api.Test;

public class ConvexPolygonIntersectorTest {

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
        float[] p = { 0, 0, 2, 2, 0, 2, 2, 0, 0, 0, 0, 0 };
        float[] q = { 2, 0, 4, 4, 0, 4, 4, 0, 2, 2, 0, 2 };
        float[] expected = null;
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleCollinearEdgeOverlap() {
        float[] p = { 0, 0, 3, 3, 0, 3, 3, 0, 0, 0, 0, 0 };
        float[] q = { 2, 0, 5, 5, 0, 5, 5, 0, 0, 2, 0, 0 };
        float[] expected = { 2, 0, 3, 3, 0, 3, 3, 0, 0, 2, 0, 0 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleRotatedSquareIntersection() {
        float[] p = { 0, 0, 4, 4, 0, 4, 4, 0, 0, 0, 0, 0 };
        float[] q = { -2, 0, -2, -2, 0, 6, 2, 0, 6, 2, 0, -2 };
        float[] expected = { 0, 0, 4, 2, 0, 4, 2, 0, 0, 0, 0, 0 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleNoIntersection() {
        float[] p = { 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0 };
        float[] q = { 5, 0, 6, 6, 0, 6, 6, 0, 5, 5, 0, 5 };
        float[] expected = null;
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleSamePolygonIntersectionWithDifferentStartVertex() {
        float[] p = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] q = { 3, 0, -3, -2, 0, -4, -4, 0, 0, -3, 0, 3, 2, 0, 3 };
        float[] expected = { -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4, -4, 0, 0 };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandle2IntersectingTriangles() {
        float[] p = { 0.015540654f, 0, 0.01743388f, 0.017864477f, 0, 0.02010114f, 0.010200679f, 0, 0.005310603f };
        float[] q = { 0.013191576f, 0, 0.020906897f, 0.015401214f, 0, 0.01738648f, 0.011963837f, 0, 0.0072015855f };
        float[] expected = { 0.012201321f, 0, 0.009852636f, 0.015012633f, 0, 0.016235121f, 0.013427231f, 0, 0.011537598f,
                0.012127571f, 0, 0.009029355f };
        assertResultsMatch(p, q, expected);
    }

    @Test
    public void shouldHandleNearlyCollinearVerticesOnClipEdge() {
        // Both CW. Nine of p's ten vertices lie (up to float rounding) on q's first edge.
        float[] p = { -49.487995f, 0, 541.00555f, -33.95485f, 0, 558.6862f, -32.96344f, 0, 556.24884f, -31.97203f, 0, 553.8115f,
                -30.980623f, 0, 551.37415f, -29.989214f, 0, 548.9368f, -28.997805f, 0, 546.49945f, -28.006397f, 0, 544.06213f,
                -27.014988f, 0, 541.62476f, -26.023579f, 0, 539.18744f };
        float[] q = { -2.2297719f, 0, 480.69107f, -60.726116f, 0, 456.89728f, -116.245f, 0, 593.38873f, -57.748657f, 0,
                617.18256f };
        assertThatNoException().isThrownBy(() -> ConvexPolygonIntersector.intersect(p, q));
    }

    @Test
    public void shouldRejectCCWPolygons() {
        float[] p = { 0, 0, 2, 2, 0, 2, 2, 0, 0, 0, 0, 0 };
        float[] q = { 1, 0, 1, 3, 0, 1, 3, 0, 3, 1, 0, 3 };
        assertThatThrownBy(() -> ConvexPolygonIntersector.intersect(p, q)).isInstanceOf(IllegalArgumentException.class)
                .hasMessage("Input polygons must be convex and clockwise.");
    }

    @Test
    public void shouldHandlePolygonsSharingOnlyEdges() {
        // Two CW triangles sharing the edge (10.18211, 0.3781242)-(10.11662, 0.32780614); the overlap has zero area.
        float[] p = { 10.18211f, 0, 0.3781242f, 10.11662f, 0, 0.32780614f, -6.559058f, 0, 1.817404f };
        float[] q = { 10.11662f, 0, 0.32780614f, 10.18211f, 0, 0.3781242f, 10.163076f, 0, 0.33511952f };
        assertThat(ConvexPolygonIntersector.intersect(p, q)).isNull();
    }

    @Test
    public void shouldComputePositiveAreaForClockwisePolygon() {
        float[] cw = { 0, 0, 0, 0, 0, 2, 2, 0, 2, 2, 0, 0 };
        assertThat(ConvexPolygonIntersector.areaXZ(cw, 4)).isEqualTo(4.0);
    }

    @Test
    public void shouldComputeNegativeAreaForCounterClockwisePolygon() {
        // Same square wound counter-clockwise: (0,0) -> (2,0) -> (2,2) -> (0,2).
        float[] ccw = { 0, 0, 0, 2, 0, 0, 2, 0, 2, 0, 0, 2 };
        assertThat(ConvexPolygonIntersector.areaXZ(ccw, 4)).isEqualTo(-4.0);
    }

    @Test
    public void shouldComputeAreaForClockwisePolygonWithFractionalCoordinates() {
        // Clockwise quad with fractional (X, Z) coordinates:
        // (1.3456,2.7891) -> (3.4567,4.1234) -> (5.6789,1.2345) -> (2.3456,0.5678).
        float[] cw = { 1.3456f, 0, 2.7891f, 3.4567f, 0, 4.1234f, 5.6789f, 0, 1.2345f, 2.3456f, 0, 0.5678f };
        assertThat(ConvexPolygonIntersector.areaXZ(cw, 4)).isCloseTo(8.5674, Offset.offset(0.0001));
    }

    @Test
    public void performanceTest() {
        // Regular 12-vertex polygon (large, radius 10)
        float[] p = { 8.66f, 0, -5, 5, 0, -8.66f, 0, 0, -10, -5, 0, -8.66f, -8.66f, 0, -5, -10, 0, 0, -8.66f, 0, 5, -5, 0, 8.66f,
                0, 0, 10, 5, 0, 8.66f, 8.66f, 0, 5, 10, 0, 0 };

        // 6-vertex polygon (small, with 4 vertices inside p)
        float[] q = { -5, 0, 2, -1, 0, 6, 3, 0, 5, 4, 0, 1, 2, 0, -3, -3, 0, -2 };

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
