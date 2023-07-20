/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

public class PolygonByCircleConstraintTest {

    private final PolygonByCircleConstraint constraint = new PolygonByCircleConstraint.StrictPolygonByCircleConstraint();

    @Test
    public void shouldHandlePolygonFullyInsideCircle() {
        float[] polygon = { -2, 0, 2, 2, 0, 2, 2, 0, -2, -2, 0, -2 };
        float[] center = { 1, 0, 1 };
        float[] constrained = constraint.apply(polygon, center, 6);

        assertThat(constrained).isEqualTo(polygon);
    }

    @Test
    public void shouldHandleVerticalSegment() {
        int expectedSize = 21;
        float[] polygon = { -2, 0, 2, 2, 0, 2, 2, 0, -2, -2, 0, -2 };
        float[] center = { 2, 0, 0 };

        float[] constrained = constraint.apply(polygon, center, 3);
        assertThat(constrained).hasSize(expectedSize);
        assertThat(constrained).containsSequence(2f, 0f, 2f, 2f, 0f, -2f);
    }

    @Test
    public void shouldHandleCircleFullyInsidePolygon() {
        int expectedSize = 12 * 3;
        float[] polygon = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] center = { -1, 0, -1 };
        float[] constrained = constraint.apply(polygon, center, 2);

        assertThat(constrained).hasSize(expectedSize);

        for (int i = 0; i < expectedSize; i += 3) {
            float x = constrained[i] + 1;
            float z = constrained[i + 2] + 1;
            assertThat(x * x + z * z).isEqualTo(4, within(1e-4f));
        }
    }

    @Test
    public void shouldHandleCircleInsidePolygon() {
        int expectedSize = 9 * 3;
        float[] polygon = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] center = { -2, 0, -1 };
        float[] constrained = constraint.apply(polygon, center, 3);

        assertThat(constrained).hasSize(expectedSize);
        assertThat(constrained).containsSequence(-2f, 0f, -4f, -4f, 0f, 0f, -3.4641016f, 0.0f, 1.6076951f, -2.0f, 0.0f, 2.0f);
    }

    @Test
    public void shouldHandleCircleOutsidePolygon() {
        int expectedSize = 7 * 3;
        float[] polygon = { -4, 0, 0, -3, 0, 3, 2, 0, 3, 3, 0, -3, -2, 0, -4 };
        float[] center = { 4, 0, 0 };
        float[] constrained = constraint.apply(polygon, center, 4);

        assertThat(constrained).hasSize(expectedSize);
        assertThat(constrained).containsSequence(1.5358982f, 0f, 3f, 2f, 0f, 3f, 3f, 0f, -3f);
    }

}
