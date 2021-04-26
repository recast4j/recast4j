/*
+recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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

import static org.recast4j.recast.RecastConstants.SPAN_MAX_HEIGHT;
import static org.recast4j.recast.RecastVectors.dot;
import static org.recast4j.recast.RecastVectors.normalize;

import java.util.function.Function;

public class RecastFilledVolumeRasterization {

    private static final float EPSILON = 0.00001f;

    public static void rasterizeSphere(Heightfield hf, float[] center, float radius, int area, int flagMergeThr, Telemetry ctx) {
        ctx.startTimer("RASTERIZE_SPHERE");
        float[] bounds = { center[0] - radius, center[1] - radius, center[2] - radius, center[0] + radius, center[1] + radius,
                center[2] + radius };
        rasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle -> intersectSphere(rectangle, center, radius * radius));
        ctx.stopTimer("RASTERIZE_SPHERE");
    }

    public static void rasterizeCapsule(Heightfield hf, float[] start, float[] end, float radius, int area, int flagMergeThr,
            Telemetry ctx) {
        ctx.startTimer("RASTERIZE_CAPSULE");
        float[] bounds = { Math.min(start[0], end[0]) - radius, Math.min(start[1], end[1]) - radius,
                Math.min(start[2], end[2]) - radius, Math.max(start[0], end[0]) + radius, Math.max(start[1], end[1]) + radius,
                Math.max(start[2], end[2]) + radius };
        float[] axis = { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
        rasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle -> intersectCapsule(rectangle, start, end, axis, radius * radius));
        ctx.stopTimer("RASTERIZE_CAPSULE");
    }

    public static void rasterizeCylinder(Heightfield hf, float[] start, float[] end, float radius, int area, int flagMergeThr,
            Telemetry ctx) {
        ctx.startTimer("RASTERIZE_CYLINDER");
        float[] bounds = { Math.min(start[0], end[0]) - radius, Math.min(start[1], end[1]) - radius,
                Math.min(start[2], end[2]) - radius, Math.max(start[0], end[0]) + radius, Math.max(start[1], end[1]) + radius,
                Math.max(start[2], end[2]) + radius };
        float[] axis = { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
        rasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle -> intersectCylinder(rectangle, start, end, axis, radius * radius));
        ctx.stopTimer("RASTERIZE_CYLINDER");
    }

    public static void rasterizeBox(Heightfield hf, float[] center, float[][] halfEdges, int area, int flagMergeThr,
            Telemetry ctx) {

        ctx.startTimer("RASTERIZE_BOX");
        float[][] normals = { { halfEdges[0][0], halfEdges[0][1], halfEdges[0][2] },
                { halfEdges[1][0], halfEdges[1][1], halfEdges[1][2] }, { halfEdges[2][0], halfEdges[2][1], halfEdges[2][2] } };
        normalize(normals[0]);
        normalize(normals[1]);
        normalize(normals[2]);

        float[] vertices = new float[8 * 3];
        float[] bounds = new float[] { Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY };
        for (int i = 0; i < 8; ++i) {
            float s0 = (i & 1) != 0 ? 1f : -1f;
            float s1 = (i & 2) != 0 ? 1f : -1f;
            float s2 = (i & 4) != 0 ? 1f : -1f;
            vertices[i * 3 + 0] = center[0] + s0 * halfEdges[0][0] + s1 * halfEdges[1][0] + s2 * halfEdges[2][0];
            vertices[i * 3 + 1] = center[1] + s0 * halfEdges[0][1] + s1 * halfEdges[1][1] + s2 * halfEdges[2][1];
            vertices[i * 3 + 2] = center[2] + s0 * halfEdges[0][2] + s1 * halfEdges[1][2] + s2 * halfEdges[2][2];
            bounds[0] = Math.min(bounds[0], vertices[i * 3 + 0]);
            bounds[1] = Math.min(bounds[1], vertices[i * 3 + 1]);
            bounds[2] = Math.min(bounds[2], vertices[i * 3 + 2]);
            bounds[3] = Math.max(bounds[3], vertices[i * 3 + 0]);
            bounds[4] = Math.max(bounds[4], vertices[i * 3 + 1]);
            bounds[5] = Math.max(bounds[5], vertices[i * 3 + 2]);
        }
        float[][] planes = new float[6][4];
        for (int i = 0; i < 6; i++) {
            float m = i < 3 ? -1 : 1;
            int vi = i < 3 ? 0 : 7;
            planes[i][0] = m * normals[i % 3][0];
            planes[i][1] = m * normals[i % 3][1];
            planes[i][2] = m * normals[i % 3][2];
            planes[i][3] = vertices[vi * 3] * planes[i][0] + vertices[vi * 3 + 1] * planes[i][1]
                    + vertices[vi * 3 + 2] * planes[i][2];
        }
        rasterizationFilledShape(hf, bounds, area, flagMergeThr, rectangle -> intersectBox(rectangle, vertices, planes));
        ctx.stopTimer("RASTERIZE_BOX");
    }

    public static void rasterizeConvex(Heightfield hf, float[] vertices, int[] triangles, int area, int flagMergeThr,
            Telemetry ctx) {

        ctx.startTimer("RASTERIZE_CONVEX");
        float[] bounds = new float[] { vertices[0], vertices[1], vertices[2], vertices[0], vertices[1], vertices[2] };
        for (int i = 0; i < vertices.length; i += 3) {
            bounds[0] = Math.min(bounds[0], vertices[i + 0]);
            bounds[1] = Math.min(bounds[1], vertices[i + 1]);
            bounds[2] = Math.min(bounds[2], vertices[i + 2]);
            bounds[3] = Math.max(bounds[3], vertices[i + 0]);
            bounds[4] = Math.max(bounds[4], vertices[i + 1]);
            bounds[5] = Math.max(bounds[5], vertices[i + 2]);
        }
        float[][] planes = new float[triangles.length][4];
        float[][] triBounds = new float[triangles.length / 3][4];
        for (int i = 0, j = 0; i < triangles.length; i += 3, j++) {
            int a = triangles[i] * 3;
            int b = triangles[i + 1] * 3;
            int c = triangles[i + 2] * 3;
            float[] ab = { vertices[b] - vertices[a], vertices[b + 1] - vertices[a + 1], vertices[b + 2] - vertices[a + 2] };
            float[] ac = { vertices[c] - vertices[a], vertices[c + 1] - vertices[a + 1], vertices[c + 2] - vertices[a + 2] };
            float[] bc = { vertices[c] - vertices[b], vertices[c + 1] - vertices[b + 1], vertices[c + 2] - vertices[b + 2] };
            float[] ca = { vertices[a] - vertices[c], vertices[a + 1] - vertices[c + 1], vertices[a + 2] - vertices[c + 2] };
            plane(planes, i, ab, ac, vertices, a);
            plane(planes, i + 1, planes[i], bc, vertices, b);
            plane(planes, i + 2, planes[i], ca, vertices, c);

            float s = 1.0f / (vertices[a] * planes[i + 1][0] + vertices[a + 1] * planes[i + 1][1]
                    + vertices[a + 2] * planes[i + 1][2] - planes[i + 1][3]);
            planes[i + 1][0] *= s;
            planes[i + 1][1] *= s;
            planes[i + 1][2] *= s;
            planes[i + 1][3] *= s;

            s = 1.0f / (vertices[b] * planes[i + 2][0] + vertices[b + 1] * planes[i + 2][1] + vertices[b + 2] * planes[i + 2][2]
                    - planes[i + 2][3]);
            planes[i + 2][0] *= s;
            planes[i + 2][1] *= s;
            planes[i + 2][2] *= s;
            planes[i + 2][3] *= s;

            triBounds[j][0] = Math.min(Math.min(vertices[a], vertices[b]), vertices[c]);
            triBounds[j][1] = Math.min(Math.min(vertices[a + 2], vertices[b + 2]), vertices[c + 2]);
            triBounds[j][2] = Math.max(Math.max(vertices[a], vertices[b]), vertices[c]);
            triBounds[j][3] = Math.max(Math.max(vertices[a + 2], vertices[b + 2]), vertices[c + 2]);

        }
        rasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle -> intersectConvex(rectangle, triangles, vertices, planes, triBounds));
        ctx.stopTimer("RASTERIZE_CONVEX");
    }

    private static void plane(float[][] planes, int p, float[] v1, float[] v2, float[] vertices, int vert) {
        RecastVectors.cross(planes[p], v1, v2);
        planes[p][3] = planes[p][0] * vertices[vert] + planes[p][1] * vertices[vert + 1] + planes[p][2] * vertices[vert + 2];
    }

    private static void rasterizationFilledShape(Heightfield hf, float[] bounds, int area, int flagMergeThr,
            Function<float[], float[]> intersection) {

        if (!overlapBounds(hf.bmin, hf.bmax, bounds)) {
            return;
        }

        bounds[3] = Math.min(bounds[3], hf.bmax[0]);
        bounds[5] = Math.min(bounds[5], hf.bmax[2]);
        bounds[0] = Math.max(bounds[0], hf.bmin[0]);
        bounds[2] = Math.max(bounds[2], hf.bmin[2]);

        if (bounds[3] <= bounds[0] || bounds[4] <= bounds[1] || bounds[5] <= bounds[2]) {
            return;
        }
        float ics = 1.0f / hf.cs;
        float ich = 1.0f / hf.ch;
        int xMin = (int) ((bounds[0] - hf.bmin[0]) * ics);
        int zMin = (int) ((bounds[2] - hf.bmin[2]) * ics);
        int xMax = Math.min(hf.width - 1, (int) ((bounds[3] - hf.bmin[0]) * ics));
        int zMax = Math.min(hf.height - 1, (int) ((bounds[5] - hf.bmin[2]) * ics));
        float[] rectangle = new float[5];
        rectangle[4] = hf.bmin[1];
        for (int x = xMin; x <= xMax; x++) {
            for (int z = zMin; z <= zMax; z++) {
                rectangle[0] = x * hf.cs + hf.bmin[0];
                rectangle[1] = z * hf.cs + hf.bmin[2];
                rectangle[2] = rectangle[0] + hf.cs;
                rectangle[3] = rectangle[1] + hf.cs;
                float[] h = intersection.apply(rectangle);
                if (h != null) {
                    int smin = (int) Math.floor((h[0] - hf.bmin[1]) * ich);
                    int smax = (int) Math.ceil((h[1] - hf.bmin[1]) * ich);
                    if (smin != smax) {
                        int ismin = RecastCommon.clamp(smin, 0, SPAN_MAX_HEIGHT);
                        int ismax = RecastCommon.clamp(smax, ismin + 1, SPAN_MAX_HEIGHT);
                        RecastRasterization.addSpan(hf, x, z, ismin, ismax, area, flagMergeThr);
                    }
                }
            }
        }
    }

    private static float[] intersectSphere(float[] rectangle, float[] center, float radiusSqr) {
        float x = Math.max(rectangle[0], Math.min(center[0], rectangle[2]));
        float y = rectangle[4];
        float z = Math.max(rectangle[1], Math.min(center[2], rectangle[3]));

        float mx = x - center[0];
        float my = y - center[1];
        float mz = z - center[2];

        float b = my; // dot(m, d) d = (0, 1, 0)
        float c = lenSqr(mx, my, mz) - radiusSqr;
        if (c > 0.0f && b > 0.0f) {
            return null;
        }
        float discr = b * b - c;
        if (discr < 0.0f) {
            return null;
        }
        float discrSqrt = (float) Math.sqrt(discr);
        float tmin = -b - discrSqrt;
        float tmax = -b + discrSqrt;

        if (tmin < 0.0f) {
            tmin = 0.0f;
        }
        return new float[] { y + tmin, y + tmax };
    }

    private static float[] intersectCapsule(float[] rectangle, float[] start, float[] end, float[] axis, float radiusSqr) {
        float[] s = mergeIntersections(intersectSphere(rectangle, start, radiusSqr), intersectSphere(rectangle, end, radiusSqr));
        float axisLen2dSqr = axis[0] * axis[0] + axis[2] * axis[2];
        if (axisLen2dSqr > EPSILON) {
            s = slabsCylinderIntersection(rectangle, start, end, axis, radiusSqr, s);
        }
        return s;
    }

    private static float[] intersectCylinder(float[] rectangle, float[] start, float[] end, float[] axis, float radiusSqr) {
        float[] s = rayCylinderIntersection(new float[] { clamp(start[0], rectangle[0], rectangle[2]), rectangle[4],
                clamp(start[2], rectangle[1], rectangle[3]) }, start, axis, radiusSqr);
        s = rayCylinderIntersection(new float[] { clamp(end[0], rectangle[0], rectangle[2]), rectangle[4],
                clamp(end[2], rectangle[1], rectangle[3]) }, start, axis, radiusSqr);
        float axisLen2dSqr = axis[0] * axis[0] + axis[2] * axis[2];
        if (axisLen2dSqr > EPSILON) {
            s = slabsCylinderIntersection(rectangle, start, end, axis, radiusSqr, s);
        }
        if (axis[1] * axis[1] > EPSILON) {
            float[][] rectangleOnStartPlane = new float[4][3];
            float[][] rectangleOnEndPlane = new float[4][3];
            float ds = dot(axis, start);
            float de = dot(axis, end);
            for (int i = 0; i < 4; i++) {
                float x = rectangle[(i + 1) & 2];
                float z = rectangle[(i & 2) + 1];
                float[] a = { x, rectangle[4], z };
                float dotAxisA = dot(axis, a);
                float t = (ds - dotAxisA) / axis[1];
                rectangleOnStartPlane[i][0] = x;
                rectangleOnStartPlane[i][1] = rectangle[4] + t;
                rectangleOnStartPlane[i][2] = z;
                t = (de - dotAxisA) / axis[1];
                rectangleOnEndPlane[i][0] = x;
                rectangleOnEndPlane[i][1] = rectangle[4] + t;
                rectangleOnEndPlane[i][2] = z;
            }
            for (int i = 0; i < 4; i++) {
                s = cylinderCapIntersection(start, radiusSqr, s, i, rectangleOnStartPlane);
                s = cylinderCapIntersection(end, radiusSqr, s, i, rectangleOnEndPlane);
            }
        }
        return s;
    }

    private static float[] cylinderCapIntersection(float[] start, float radiusSqr, float[] s, int i, float[][] rectangleOnPlane) {
        int j = (i + 1) % 4;
        // Ray against sphere intersection
        float[] m = { rectangleOnPlane[i][0] - start[0], rectangleOnPlane[i][1] - start[1], rectangleOnPlane[i][2] - start[2] };
        float[] d = { rectangleOnPlane[j][0] - rectangleOnPlane[i][0], rectangleOnPlane[j][1] - rectangleOnPlane[i][1],
                rectangleOnPlane[j][2] - rectangleOnPlane[i][2] };
        float dl = dot(d, d);
        float b = dot(m, d) / dl;
        float c = (dot(m, m) - radiusSqr) / dl;
        float discr = b * b - c;
        if (discr > EPSILON) {
            float discrSqrt = (float) Math.sqrt(discr);
            float t1 = -b - discrSqrt;
            float t2 = -b + discrSqrt;
            if (t1 <= 1 && t2 >= 0) {
                t1 = Math.max(0, t1);
                t2 = Math.min(1, t2);
                float y1 = rectangleOnPlane[i][1] + t1 * d[1];
                float y2 = rectangleOnPlane[i][1] + t2 * d[1];
                float[] y = { Math.min(y1, y2), Math.max(y1, y2) };
                s = mergeIntersections(s, y);
            }
        }
        return s;
    }

    private static float[] slabsCylinderIntersection(float[] rectangle, float[] start, float[] end, float[] axis, float radiusSqr,
            float[] s) {
        if (Math.min(start[0], end[0]) < rectangle[0]) {
            s = mergeIntersections(s, xSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[0]));
        }
        if (Math.max(start[0], end[0]) > rectangle[2]) {
            s = mergeIntersections(s, xSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[2]));
        }
        if (Math.min(start[2], end[2]) < rectangle[1]) {
            s = mergeIntersections(s, zSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[1]));
        }
        if (Math.max(start[2], end[2]) > rectangle[3]) {
            s = mergeIntersections(s, zSlabCylinderIntersection(rectangle, start, axis, radiusSqr, rectangle[3]));
        }
        return s;
    }

    private static float[] xSlabCylinderIntersection(float[] rectangle, float[] start, float[] axis, float radiusSqr, float x) {
        return rayCylinderIntersection(xSlabRayIntersection(rectangle, start, axis, x), start, axis, radiusSqr);
    }

    private static float[] xSlabRayIntersection(float[] rectangle, float[] start, float[] direction, float x) {
        // 2d intersection of plane and segment
        float t = (x - start[0]) / direction[0];
        float z = clamp(start[2] + t * direction[2], rectangle[1], rectangle[3]);
        return new float[] { x, rectangle[4], z };
    }

    private static float[] zSlabCylinderIntersection(float[] rectangle, float[] start, float[] axis, float radiusSqr, float z) {
        return rayCylinderIntersection(zSlabRayIntersection(rectangle, start, axis, z), start, axis, radiusSqr);
    }

    private static float[] zSlabRayIntersection(float[] rectangle, float[] start, float[] direction, float z) {
        // 2d intersection of plane and segment
        float t = (z - start[2]) / direction[2];
        float x = clamp(start[0] + t * direction[0], rectangle[0], rectangle[2]);
        return new float[] { x, rectangle[4], z };
    }

    // Based on Christer Ericsons's "Real-Time Collision Detection"
    private static float[] rayCylinderIntersection(float[] point, float[] start, float[] axis, float radiusSqr) {
        float[] d = axis;
        float[] m = { point[0] - start[0], point[1] - start[1], point[2] - start[2] };
        // float[] n = { 0, 1, 0 };
        float md = dot(m, d);
        // float nd = dot(n, d);
        float nd = axis[1];
        float dd = dot(d, d);

        // float nn = dot(n, n);
        float nn = 1;
        // float mn = dot(m, n);
        float mn = m[1];
        // float a = dd * nn - nd * nd;
        float a = dd - nd * nd;
        float k = dot(m, m) - radiusSqr;
        float c = dd * k - md * md;
        if (Math.abs(a) < EPSILON) {
            // Segment runs parallel to cylinder axis
            if (c > 0.0f) {
                return null; // ’a’ and thus the segment lie outside cylinder
            }
            // Now known that segment intersects cylinder; figure out how it intersects
            float t1 = -mn / nn; // Intersect segment against ’p’ endcap
            float t2 = (nd - mn) / nn; // Intersect segment against ’q’ endcap
            return new float[] { point[1] + Math.min(t1, t2), point[1] + Math.max(t1, t2) };
        }
        float b = dd * mn - nd * md;
        float discr = b * b - a * c;
        if (discr < 0.0f) {
            return null; // No real roots; no intersection
        }
        float discSqrt = (float) Math.sqrt(discr);
        float t1 = (-b - discSqrt) / a;
        float t2 = (-b + discSqrt) / a;

        if (md + t1 * nd < 0.0f) {
            // Intersection outside cylinder on ’p’ side
            t1 = -md / nd;
            if (k + t1 * (2 * mn + t1 * nn) > 0.0f) {
                return null;
            }
        } else if (md + t1 * nd > dd) {
            // Intersection outside cylinder on ’q’ side
            t1 = (dd - md) / nd;
            if (k + dd - 2 * md + t1 * (2 * (mn - nd) + t1 * nn) > 0.0f) {
                return null;
            }
        }
        if (md + t2 * nd < 0.0f) {
            // Intersection outside cylinder on ’p’ side
            t2 = -md / nd;
            if (k + t2 * (2 * mn + t2 * nn) > 0.0f) {
                return null;
            }
        } else if (md + t2 * nd > dd) {
            // Intersection outside cylinder on ’q’ side
            t2 = (dd - md) / nd;
            if (k + dd - 2 * md + t2 * (2 * (mn - nd) + t2 * nn) > 0.0f) {
                return null;
            }
        }
        return new float[] { point[1] + Math.min(t1, t2), point[1] + Math.max(t1, t2) };
    }

    private static float[] intersectBox(float[] rectangle, float[] vertices, float[][] planes) {
        float yMin = Float.POSITIVE_INFINITY;
        float yMax = Float.NEGATIVE_INFINITY;
        // check intersection with rays starting in box vertices first
        for (int i = 0; i < 8; i++) {
            int vi = i * 3;
            if (vertices[vi] >= rectangle[0] && vertices[vi] < rectangle[2] && vertices[vi + 2] >= rectangle[1]
                    && vertices[vi + 2] < rectangle[3]) {
                yMin = Math.min(yMin, vertices[vi + 1]);
                yMax = Math.max(yMax, vertices[vi + 1]);
            }
        }

        // check intersection with rays starting in rectangle vertices first
        float[] point = new float[] { 0, rectangle[1], 0 };
        for (int i = 0; i < 4; i++) {
            point[0] = ((i & 1) == 0) ? rectangle[0] : rectangle[2];
            point[2] = ((i & 2) == 0) ? rectangle[1] : rectangle[3];
            for (int j = 0; j < 6; j++) {
                if (Math.abs(planes[j][1]) > EPSILON) {
                    float dotNormalPoint = dot(planes[j], point);
                    float t = (planes[j][3] - dotNormalPoint) / planes[j][1];
                    float y = point[1] + t;
                    boolean valid = true;
                    for (int k = 0; k < 6; k++) {
                        if (k != j) {
                            if (point[0] * planes[k][0] + y * planes[k][1] + point[2] * planes[k][2] > planes[k][3]) {
                                valid = false;
                                break;
                            }
                        }
                    }
                    if (valid) {
                        yMin = Math.min(yMin, y);
                        yMax = Math.max(yMax, y);
                    }
                }
            }
        }
        if (yMin <= yMax) {
            return new float[] { yMin, yMax };
        }
        return null;
    }

    private static float[] intersectConvex(float[] rectangle, int[] triangles, float[] verts, float[][] planes,
            float[][] triBounds) {
        float imin = Float.POSITIVE_INFINITY;
        float imax = Float.NEGATIVE_INFINITY;
        for (int tr = 0, tri = 0; tri < triangles.length; tr++, tri += 3) {
            if (triBounds[tr][0] > rectangle[2] || triBounds[tr][2] < rectangle[0] || triBounds[tr][1] > rectangle[3]
                    || triBounds[tr][3] < rectangle[1]) {
                continue;
            }
            if (Math.abs(planes[tri][1]) < EPSILON) {
                continue;
            }
            for (int i = 0; i < 3; i++) {
                int vi = triangles[tri + i] * 3;
                int vj = triangles[tri + (i + 1) % 3] * 3;
                float x = verts[vi];
                float z = verts[vi + 2];
                // triangle vertex
                if (x >= rectangle[0] && x <= rectangle[2] && z >= rectangle[1] && z <= rectangle[3]) {
                    imin = Math.min(imin, verts[vi + 1]);
                    imax = Math.max(imax, verts[vi + 1]);
                }
                // triangle slab intersection
                float y = verts[vi + 1];
                float dx = verts[vj] - x;
                float dy = verts[vj + 1] - y;
                float dz = verts[vj + 2] - z;
                if (Math.abs(dx) > EPSILON) {
                    Float iy = xSlabTriangleIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[0]);
                    if (iy != null) {
                        imin = Math.min(imin, iy);
                        imax = Math.max(imax, iy);
                    }
                    iy = xSlabTriangleIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[2]);
                    if (iy != null) {
                        imin = Math.min(imin, iy);
                        imax = Math.max(imax, iy);
                    }
                }
                if (Math.abs(dz) > EPSILON) {
                    Float iy = zSlabTriangleIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[1]);
                    if (iy != null) {
                        imin = Math.min(imin, iy);
                        imax = Math.max(imax, iy);
                    }
                    iy = zSlabTriangleIntersection(rectangle, x, y, z, dx, dy, dz, rectangle[3]);
                    if (iy != null) {
                        imin = Math.min(imin, iy);
                        imax = Math.max(imax, iy);
                    }
                }
            }
            // rectangle vertex
            float[] point = new float[] { 0, rectangle[1], 0 };
            for (int i = 0; i < 4; i++) {
                point[0] = ((i & 1) == 0) ? rectangle[0] : rectangle[2];
                point[2] = ((i & 2) == 0) ? rectangle[1] : rectangle[3];
                Float y = rayTriangleIntersection(point, tri, planes);
                if (y != null) {
                    imin = Math.min(imin, y);
                    imax = Math.max(imax, y);
                }
            }
        }
        if (imin < imax) {
            return new float[] { imin, imax };
        }
        return null;
    }

    private static Float xSlabTriangleIntersection(float[] rectangle, float x, float y, float z, float dx, float dy, float dz,
            float slabX) {
        float x2 = x + dx;
        if ((x < slabX && x2 > slabX) || (x > slabX && x2 < slabX)) {
            float t = (slabX - x) / dx;
            float iz = z + dz * t;
            if (iz >= rectangle[1] && iz <= rectangle[3]) {
                return y + dy * t;
            }
        }
        return null;
    }

    private static Float zSlabTriangleIntersection(float[] rectangle, float x, float y, float z, float dx, float dy, float dz,
            float slabZ) {
        float z2 = z + dz;
        if ((z < slabZ && z2 > slabZ) || (z > slabZ && z2 < slabZ)) {
            float t = (slabZ - z) / dz;
            float ix = x + dx * t;
            if (ix >= rectangle[0] && ix <= rectangle[2]) {
                return y + dy * t;
            }
        }
        return null;
    }

    private static Float rayTriangleIntersection(float[] point, int plane, float[][] planes) {
        float t = (planes[plane][3] - dot(planes[plane], point)) / planes[plane][1];
        float[] s = { point[0], point[1] + t, point[2] };
        float u = dot(s, planes[plane + 1]) - planes[plane + 1][3];
        if (u < 0.0f || u > 1.0f) {
            return null;
        }
        float v = dot(s, planes[plane + 2]) - planes[plane + 2][3];
        if (v < 0.0f) {
            return null;
        }
        float w = 1f - u - v;
        if (w < 0.0f) {
            return null;
        }
        return s[1];
    }

    private static float[] mergeIntersections(float[] s1, float[] s2) {
        if (s1 == null && s2 == null) {
            return null;
        }
        if (s1 == null) {
            return s2;
        }
        if (s2 == null) {
            return s1;
        }
        return new float[] { Math.min(s1[0], s2[0]), Math.max(s1[1], s2[1]) };
    }

    private static float lenSqr(float dx, float dy, float dz) {
        return dx * dx + dy * dy + dz * dz;
    }

    public static float clamp(float v, float min, float max) {
        return Math.max(Math.min(max, v), min);
    }

    private static boolean overlapBounds(float[] amin, float[] amax, float[] bounds) {
        boolean overlap = true;
        overlap = (amin[0] > bounds[3] || amax[0] < bounds[0]) ? false : overlap;
        overlap = (amin[1] > bounds[4]) ? false : overlap;
        overlap = (amin[2] > bounds[5] || amax[2] < bounds[2]) ? false : overlap;
        return overlap;
    }

}
