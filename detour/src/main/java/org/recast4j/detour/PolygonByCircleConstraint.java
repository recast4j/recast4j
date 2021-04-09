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

import static org.recast4j.detour.DetourCommon.vDist2DSqr;

import java.util.Arrays;

public interface PolygonByCircleConstraint {

    float[] aply(float[] polyVerts, float[] circleCenter, float radius);

    public static PolygonByCircleConstraint noop() {
        return new NoOpPolygonByCircleConstrainer();
    }

    public static PolygonByCircleConstraint strict() {
        return new StrictPolygonByCircleConstrainer();
    }

    public static class NoOpPolygonByCircleConstrainer implements PolygonByCircleConstraint {

        @Override
        public float[] aply(float[] polyVerts, float[] circleCenter, float radius) {
            return polyVerts;
        }

    }

    /*
     * Use a simple sweep plane algorithm to approximate the intersection of a circle and a polygon
     */
    public static class StrictPolygonByCircleConstrainer implements PolygonByCircleConstraint {
        // margin on both sides to avoid the need to handle intersections with vertical segments
        private static final float SWEEP_MARGIN = 0.005f;
        private static final int APPROXIMATION_SLICES = 6;
        private static final float COLLINEAR_TOLERANCE = 0.01f;
        // private static final float EPSILON = 1e-6f;

        @Override
        public float[] aply(float[] verts, float[] center, float radius) {
            float radiusSqr = radius * radius;
            int outsideVertex = -1;
            for (int pv = 0; pv < verts.length; pv += 3) {
                if (vDist2DSqr(center, verts, pv) > radiusSqr) {
                    outsideVertex = pv;
                    break;
                }
            }
            if (outsideVertex == -1) {
                // polygon inside circle
                return verts;
            }
//            boolean circleCenterInPolygon = pointInPolygon(center, verts, verts.length / 3);
//            if (circleCenterInPolygon) {
//                boolean circleFullyInside = true;
//                for (int pv = 0; pv < verts.length; pv += 3) {
//                    Tupple2<Float, Float> distseg = distancePtSegSqr2D(center, verts, pv, (pv + 3) % verts.length);
//                    float distSqr = distseg.first + EPSILON;
//                    if (distSqr < radiusSqr) {
//                        circleFullyInside = false;
//                        break;
//                    }
//                }
//                if (circleFullyInside) {
//
//                }
//            }
            float minX = Float.POSITIVE_INFINITY;
            float maxX = Float.NEGATIVE_INFINITY;
            float midY = 0;
            for (int pv = 0; pv < verts.length; pv += 3) {
                minX = Math.min(minX, verts[pv]);
                maxX = Math.max(maxX, verts[pv]);
                midY += verts[pv + 1];
            }
            midY *= 3.0f / verts.length;
            minX = Math.max(center[0] - radius, minX);
            maxX = Math.min(center[0] + radius, maxX);
            if (minX >= maxX) {
                // disjoint
                return null;
            }
            float step = (maxX - minX) / (APPROXIMATION_SLICES - 1);
            float x = minX + step * SWEEP_MARGIN;
            step *= 1.0f - 2 * SWEEP_MARGIN;
            int count = 0;
            float[] slices = new float[APPROXIMATION_SLICES * 3];
            for (int i = 0; i < APPROXIMATION_SLICES; i++) {
                // Assume ray origin in (x, 0, center[2])
                float mx = x - center[0];
                float c = mx * mx - radiusSqr;
                float discr = -c;
                double dSqrt = Math.sqrt(discr);
                float tmin = (float) -dSqrt;
                float tmax = (float) dSqrt;
                float smin = Float.POSITIVE_INFINITY;
                float smax = Float.NEGATIVE_INFINITY;
                for (int pv = 0; pv < verts.length; pv += 3) {
                    int pw = (pv + 3) % verts.length;
                    if ((verts[pv] >= x && verts[pw] <= x) || (verts[pv] <= x && verts[pw] >= x)) {
//                        optimized version of intersectSegSeg2D
//                        float[] u = new float[] { 0, 0, 1 };
//                        float[] v = new float[] { verts[pw] - verts[pv], 0, verts[pw + 2] - verts[pv + 2] };
//                        float[] w = new float[] { x - verts[pv], 0, center[2] - verts[pv + 2] };
//                        float d = vperpXZ(u, v);
//                        float s = vperpXZ(v, w) / d;
                        float a0 = verts[pw] - verts[pv];
                        float a2 = verts[pw + 2] - verts[pv + 2];
                        float b0 = x - verts[pv];
                        float b2 = center[2] - verts[pv + 2];
                        float d = -a0;
                        float s = (a0 * b2 - a2 * b0) / d;
                        smin = Math.min(smin, s);
                        smax = Math.max(smax, s);
                    }
                }
                float minZ = Math.max(tmin, smin);
                float maxZ = Math.min(tmax, smax);
                if (minZ < maxZ) {
                    slices[count * 3] = x;
                    slices[count * 3 + 1] = maxZ;
                    slices[count * 3 + 2] = minZ;
                    count++;
                }
                x += step;
            }
            if (count > 2) {
                float[] result = new float[6 * count];
                int is = 0;
                int ir = 0;
                for (int i = 0; i < count; i++, is += 3) {
                    float z = slices[is + 1] + center[2];
                    ir = addVertex(slices, result, is, ir, i, z, midY);
                }
                is = (count - 1) * 3;
                for (int i = 0; i < count; i++, is -= 3) {
                    float z = slices[is + 2] + center[2];
                    ir = addVertex(slices, result, is, ir, i, z, midY);
                }
                return Arrays.copyOf(result, ir);
            }
            // disjoint
            return null;
        }

        private int addVertex(float[] slices, float[] result, int is, int ir, int i, float z, float midY) {
            if (i > 2 && collinear(result[ir - 6], result[ir - 6 + 2], result[ir - 3], result[ir - 3 + 2], slices[is],
                    z)) {
                ir -= 3;
            }
            result[ir] = slices[is];
            result[ir + 1] = midY;
            result[ir + 2] = z;
            ir += 3;
            return ir;
        }

        private boolean collinear(float x1, float y1, float x2, float y2, float x3, float y3) {
            return Math.abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) < COLLINEAR_TOLERANCE;
        }

    }

}
