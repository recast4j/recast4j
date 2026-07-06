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
package org.recast4j.detour;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;

import org.junit.jupiter.api.Test;

/**
 * Tests for Detour common math functions ported from upstream C++ tests. See:
 * https://github.com/recastnavigation/recastnavigation/blob/main/Tests/Detour/Tests_Detour.cpp
 *
 * Note: The dtRandomPointInConvexPoly function is private in the Java port, but is tested indirectly through the public
 * APIs like findRandomPoint() which is covered in RandomPointTest.
 */
public class DetourCommonTest {

    @Test
    public void testVAdd() {
        float[] v1 = { 1, 2, 3 };
        float[] v2 = { 4, 5, 6 };
        float[] result = DetourCommon.vAdd(v1, v2);
        assertThat(result[0]).isEqualTo(5f, offset(0.001f));
        assertThat(result[1]).isEqualTo(7f, offset(0.001f));
        assertThat(result[2]).isEqualTo(9f, offset(0.001f));
    }

    @Test
    public void testVSub() {
        float[] v1 = { 5, 6, 7 };
        float[] v2 = { 1, 2, 3 };
        float[] result = DetourCommon.vSub(v1, v2);
        assertThat(result[0]).isEqualTo(4f, offset(0.001f));
        assertThat(result[1]).isEqualTo(4f, offset(0.001f));
        assertThat(result[2]).isEqualTo(4f, offset(0.001f));
    }

    @Test
    public void testVCopy() {
        float[] v1 = { 1, 2, 3 };
        float[] result = DetourCommon.vCopy(v1);
        assertThat(result[0]).isEqualTo(1f, offset(0.001f));
        assertThat(result[1]).isEqualTo(2f, offset(0.001f));
        assertThat(result[2]).isEqualTo(3f, offset(0.001f));
    }

    @Test
    public void testVMad() {
        float[] v1 = { 1, 2, 3 };
        float[] v2 = { 2, 3, 4 };
        float[] result = DetourCommon.vMad(v1, v2, 2.0f);
        assertThat(result[0]).isEqualTo(5f, offset(0.001f));
        assertThat(result[1]).isEqualTo(8f, offset(0.001f));
        assertThat(result[2]).isEqualTo(11f, offset(0.001f));
    }

    @Test
    public void testVDot2D() {
        float[] v1 = { 1, 0, 0 };
        float[] v2 = { 1, 0, 0 };
        float result = DetourCommon.vDot2D(v1, v2);
        assertThat(result).isEqualTo(1f, offset(0.001f));
    }

    @Test
    public void testVDot2DZeroVector() {
        float[] v1 = { 1, 2, 3 };
        float[] v2 = { 0, 0, 0 };
        float result = DetourCommon.vDot2D(v1, v2);
        assertThat(result).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVLen() {
        float[] v1 = { 3, 4, 0 };
        float result = DetourCommon.vLen(v1);
        assertThat(result).isEqualTo(5f, offset(0.001f));
    }

    @Test
    public void testVLenSqr() {
        float[] v1 = { 3, 4, 0 };
        float result = DetourCommon.vLenSqr(v1);
        assertThat(result).isEqualTo(25f, offset(0.001f));
    }

    @Test
    public void testVDist() {
        float[] v1 = { 0, 0, 0 };
        float[] v2 = { 3, 4, 0 };
        float result = DetourCommon.vDist(v1, v2);
        assertThat(result).isEqualTo(5f, offset(0.001f));
    }

    @Test
    public void testVDist2DSqr() {
        float[] v1 = { 0, 0, 0 };
        float[] v2 = { 3, 0, 4 };
        float result = DetourCommon.vDist2DSqr(v1, v2);
        assertThat(result).isEqualTo(25f, offset(0.001f));
    }

    @Test
    public void testVNormalize() {
        float[] v = { 3, 4, 0 };
        DetourCommon.vNormalize(v);
        assertThat(v[0]).isEqualTo(0.6f, offset(0.001f));
        assertThat(v[1]).isEqualTo(0.8f, offset(0.001f));
        assertThat(v[2]).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVLerp() {
        float[] v1 = { 0, 0, 0 };
        float[] v2 = { 10, 10, 10 };
        float[] result = DetourCommon.vLerp(v1, v2, 0.5f);
        assertThat(result[0]).isEqualTo(5f, offset(0.001f));
        assertThat(result[1]).isEqualTo(5f, offset(0.001f));
        assertThat(result[2]).isEqualTo(5f, offset(0.001f));
    }

    @Test
    public void testVLerpStart() {
        float[] v1 = { 0, 0, 0 };
        float[] v2 = { 10, 10, 10 };
        float[] result = DetourCommon.vLerp(v1, v2, 0.0f);
        assertThat(result[0]).isEqualTo(0f, offset(0.001f));
        assertThat(result[1]).isEqualTo(0f, offset(0.001f));
        assertThat(result[2]).isEqualTo(0f, offset(0.001f));
    }

    @Test
    public void testVLerpEnd() {
        float[] v1 = { 0, 0, 0 };
        float[] v2 = { 10, 10, 10 };
        float[] result = DetourCommon.vLerp(v1, v2, 1.0f);
        assertThat(result[0]).isEqualTo(10f, offset(0.001f));
        assertThat(result[1]).isEqualTo(10f, offset(0.001f));
        assertThat(result[2]).isEqualTo(10f, offset(0.001f));
    }
}
