/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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
package org.recast4j.recast;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import org.junit.jupiter.api.Test;

/**
 * Tests for Recast utility functions ported from upstream C++ tests. See:
 * https://github.com/recastnavigation/recastnavigation/blob/main/Tests/Recast/Tests_Recast.cpp
 */
public class RecastUtilTest {

    @Test
    public void testVcross() {
        // Computes cross product
        float v1[] = { 3, -3, 1 };
        float v2[] = { 4, 9, 2 };
        float result[] = new float[3];
        RecastVectors.cross(result, v1, v2);
        assertThat(result[0]).isEqualTo(-15f, offset(0.001f));
        assertThat(result[1]).isEqualTo(-2f, offset(0.001f));
        assertThat(result[2]).isEqualTo(39f, offset(0.001f));
    }

    @Test
    public void testVcrossSelfIsZero() {
        // Cross product with itself is zero
        float v1[] = { 3, -3, 1 };
        float result[] = new float[3];
        RecastVectors.cross(result, v1, v1);
        assertThat(result[0]).isEqualTo(0f, offset(0.001f));
        assertThat(result[1]).isEqualTo(0f, offset(0.001f));
        assertThat(result[2]).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVdot() {
        // Dot normalized vector with itself
        float v1[] = { 1, 0, 0 };
        float result = RecastVectors.dot(v1, v1);
        assertThat(result).isEqualTo(1f, offset(0.001f));
    }

    @Test
    public void testVdotZeroVectorIsZero() {
        // Dot zero vector with anything is zero
        float v1[] = { 1, 2, 3 };
        float v2[] = { 0, 0, 0 };
        float result = RecastVectors.dot(v1, v2);
        assertThat(result).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVadd() {
        // add two vectors
        float v1[] = { 1, 2, 3 };
        float v2[] = { 5, 6, 7 };
        float result[] = new float[3];
        RecastVectors.add(result, v1, v2, 0);
        assertThat(result[0]).isEqualTo(6f, offset(0.001f));
        assertThat(result[1]).isEqualTo(8f, offset(0.001f));
        assertThat(result[2]).isEqualTo(10f, offset(0.001f));
    }

    @Test
    public void testVsub() {
        // subtract two vectors
        float v1[] = { 5, 4, 3 };
        float v2[] = { 1, 2, 3 };
        float result[] = new float[3];
        RecastVectors.sub(result, v1, v2, 0);
        assertThat(result[0]).isEqualTo(4f, offset(0.001f));
        assertThat(result[1]).isEqualTo(2f, offset(0.001f));
        assertThat(result[2]).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVmin() {
        // selects the min component from the vectors
        float v1[] = { 5, 4, 0 };
        float v2[] = { 1, 2, 9 };
        RecastVectors.min(v1, v2, 0);
        assertThat(v1[0]).isEqualTo(1f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(2f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVminV1IsMin() {
        // v1 is min
        float v1[] = { 1, 2, 3 };
        float v2[] = { 4, 5, 6 };
        RecastVectors.min(v1, v2, 0);
        assertThat(v1[0]).isEqualTo(1f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(2f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(3f, offset(0.001f));
    }

    @Test
    public void testVminV2IsMin() {
        // v2 is min
        float v1[] = { 4, 5, 6 };
        float v2[] = { 1, 2, 3 };
        RecastVectors.min(v1, v2, 0);
        assertThat(v1[0]).isEqualTo(1f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(2f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(3f, offset(0.001f));
    }

    @Test
    public void testVmax() {
        // selects the max component from the vectors
        float v1[] = { 5, 4, 0 };
        float v2[] = { 1, 2, 9 };
        RecastVectors.max(v1, v2, 0);
        assertThat(v1[0]).isEqualTo(5f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(4f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(9f, offset(0.001f));
    }

    @Test
    public void testVmaxV2IsMax() {
        // v2 is max
        float v1[] = { 1, 2, 3 };
        float v2[] = { 4, 5, 6 };
        RecastVectors.max(v1, v2, 0);
        assertThat(v1[0]).isEqualTo(4f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(5f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(6f, offset(0.001f));
    }

    @Test
    public void testVmaxV1IsMax() {
        // v1 is max
        float v1[] = { 4, 5, 6 };
        float v2[] = { 1, 2, 3 };
        RecastVectors.max(v1, v2, 0);
        assertThat(v1[0]).isEqualTo(4f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(5f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(6f, offset(0.001f));
    }

    @Test
    public void testVcopy() {
        // copies a vector into another vector
        float v1[] = { 5, 4, 0 };
        float result[] = { 1, 2, 9 };
        RecastVectors.copy(result, v1);
        assertThat(result[0]).isEqualTo(5f, offset(0.001f));
        assertThat(result[1]).isEqualTo(4f, offset(0.001f));
        assertThat(result[2]).isEqualTo(0f, offset(0.001f));
        assertThat(v1[0]).isEqualTo(5f, offset(0.001f));
        assertThat(v1[1]).isEqualTo(4f, offset(0.001f));
        assertThat(v1[2]).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVnormalize() {
        // normalizing reduces magnitude to 1
        float v[] = { 3, 3, 3 };
        RecastVectors.normalize(v);
        float expectedVal = (float) Math.sqrt(1.0f / 3.0f);
        assertThat(v[0]).isEqualTo(expectedVal, offset(0.001f));
        assertThat(v[1]).isEqualTo(expectedVal, offset(0.001f));
        assertThat(v[2]).isEqualTo(expectedVal, offset(0.001f));
        float magnitude = (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        assertThat(magnitude).isEqualTo(1f, offset(0.001f));
    }

    @Test
    public void testCalcGridSize() {
        // computes the size of an x & z axis grid
        float verts[] = { 1, 2, 3, 0, 2, 6 };
        float bmin[] = new float[3];
        float bmax[] = new float[3];
        calcBounds(verts, 2, bmin, bmax);

        float cellSize = 1.5f;

        int[] gridSize = Recast.calcGridSize(bmin, bmax, cellSize);

        assertThat(gridSize[0]).isEqualTo(1);
        assertThat(gridSize[1]).isEqualTo(2);
    }

    @Test
    public void testMarkWalkableTriangles() {
        Telemetry ctx = new Telemetry();
        float walkableSlopeAngle = 45;
        float verts[] = { 0, 0, 0, 1, 0, 0, 0, 0, -1 };
        int nv = 3;
        int walkable_tri[] = { 0, 1, 2 };
        int unwalkable_tri[] = { 0, 2, 1 };
        int nt = 1;

        // One walkable triangle
        {
            int[] areas = Recast.markWalkableTriangles(ctx, walkableSlopeAngle, verts, walkable_tri, nt,
                    new AreaModification(1, 1));
            assertThat(areas[0]).isEqualTo(1).describedAs("One walkable triangle");
        }

        // One non-walkable triangle
        {
            int[] areas = Recast.markWalkableTriangles(ctx, walkableSlopeAngle, verts, unwalkable_tri, nt,
                    new AreaModification(0, 0));
            assertThat(areas[0]).isEqualTo(0).describedAs("One non-walkable triangle");
        }

        // Slopes equal to the max slope are considered unwalkable
        {
            walkableSlopeAngle = 0;
            int[] areas = Recast.markWalkableTriangles(ctx, walkableSlopeAngle, verts, walkable_tri, nt,
                    new AreaModification(1, 1));
            assertThat(areas[0]).isEqualTo(0).describedAs("Slopes equal to the max slope are considered unwalkable.");
        }
    }

    @Test
    public void testClearUnwalkableTriangles() {
        Telemetry ctx = new Telemetry();
        float walkableSlopeAngle = 45;
        float verts[] = { 0, 0, 0, 1, 0, 0, 0, 0, -1 };
        int nv = 3;
        int walkable_tri[] = { 0, 1, 2 };
        int unwalkable_tri[] = { 0, 2, 1 };
        int nt = 1;

        // Sets area ID of unwalkable triangle to RC_NULL_AREA
        {
            int areas[] = { 42 };
            Recast.clearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
            assertThat(areas[0]).isEqualTo(RecastConstants.RC_NULL_AREA)
                    .describedAs("Sets area ID of unwalkable triangle to RC_NULL_AREA");
        }

        // Does not modify walkable triangle area ID's
        {
            int areas[] = { 42 };
            Recast.clearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
            assertThat(areas[0]).isEqualTo(42).describedAs("Does not modify walkable triangle area ID's");
        }

        // Slopes equal to the max slope are considered unwalkable
        {
            int areas[] = { 42 };
            walkableSlopeAngle = 0;
            Recast.clearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
            assertThat(areas[0]).isEqualTo(RecastConstants.RC_NULL_AREA)
                    .describedAs("Slopes equal to the max slope are considered unwalkable.");
        }
    }

    // Helper method for calculating bounds
    private void calcBounds(float[] verts, int nv, float[] bmin, float[] bmax) {
        for (int i = 0; i < 3; i++) {
            bmin[i] = verts[i];
            bmax[i] = verts[i];
        }
        for (int i = 1; i < nv; ++i) {
            for (int j = 0; j < 3; j++) {
                bmin[j] = Math.min(bmin[j], verts[i * 3 + j]);
                bmax[j] = Math.max(bmax[j], verts[i * 3 + j]);
            }
        }
    }
}
