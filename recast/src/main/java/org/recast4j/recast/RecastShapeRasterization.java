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

package org.recast4j.recast;

import static org.recast4j.recast.RecastConstants.SPAN_MAX_HEIGHT;

import java.util.function.Function;

public class RecastShapeRasterization {

    private static final float EPSILON = 0.00001f;

    public static void rasterizeSphere(Heightfield hf, float[] center, float radius, int area, int flagMergeThr, Telemetry ctx) {
        ctx.startTimer("RASTERIZE_SPHERE");
        float[] bounds = new float[] { center[0] - radius, center[1] - radius, center[2] - radius, center[0] + radius,
                center[1] + radius, center[2] + radius };
        rasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle -> intersectSphere(rectangle, center, radius * radius));
        ctx.stopTimer("RASTERIZE_SPHERE");
    }

    public static void rasterizeCapsule(Heightfield hf, float[] start, float[] end, float radius, int area, int flagMergeThr,
            Telemetry ctx) {
        ctx.startTimer("RASTERIZE_CAPSULE");
        float[] bounds = new float[] { Math.min(start[0], end[0]) - radius, Math.min(start[1], end[1]) - radius,
                Math.min(start[2], end[2]) - radius, Math.max(start[0], end[0]) + radius, Math.max(start[1], end[1]) + radius,
                Math.max(start[2], end[2]) + radius };
        float[] axis = new float[] { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
        float axisLenSqr = lenSqr(axis);
        float[] normal = getUprightNormal(axis);
        rasterizationFilledShape(hf, bounds, area, flagMergeThr,
                rectangle -> intersectCapsule(rectangle, start, end, axis, normal, axisLenSqr, radius, radius * radius));
        ctx.stopTimer("RASTERIZE_CAPSULE");
    }

    public static void rasterizeBox(Heightfield hf, float[] center, float[] extent, float[] up, int area, int flagMergeThr,
            Telemetry ctx) {

        RecastVectors.normalize(up);
        float[][] normals = new float[][] { new float[3], up, new float[3] };
        if (Math.abs(up[2]) < 0.9f) {
            normals[0][0] = 0;
            normals[0][1] = 0;
            normals[0][2] = -1;
        } else {
            normals[0][0] = 0;
            normals[0][1] = -1;
            normals[0][2] = 0;
        }
        RecastVectors.cross(normals[2], normals[0], up);
        RecastVectors.normalize(normals[2]);
        RecastVectors.cross(normals[0], up, normals[2]);
        RecastVectors.normalize(normals[0]);

        float[] trX = new float[] { normals[2][0], up[0], normals[0][0] };
        float[] trY = new float[] { normals[2][1], up[1], normals[0][1] };
        float[] trZ = new float[] { normals[2][2], up[2], normals[0][2] };

        float[] vertices = new float[8 * 3];
        float[] bounds = new float[] { Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY };
        for (int i = 0; i < 8; ++i) {
            float[] pt = new float[] { ((i & 1) != 0) ? extent[0] : -extent[0], ((i & 2) != 0) ? extent[1] : -extent[1],
                    ((i & 4) != 0) ? extent[2] : -extent[2] };
            vertices[i * 3 + 0] = dot(pt, trX) + center[0];
            vertices[i * 3 + 1] = dot(pt, trY) + center[1];
            vertices[i * 3 + 2] = dot(pt, trZ) + center[2];
            bounds[0] = Math.min(bounds[0], vertices[i * 3 + 0]);
            bounds[1] = Math.min(bounds[1], vertices[i * 3 + 1]);
            bounds[2] = Math.min(bounds[2], vertices[i * 3 + 2]);
            bounds[3] = Math.max(bounds[3], vertices[i * 3 + 0]);
            bounds[4] = Math.max(bounds[4], vertices[i * 3 + 1]);
            bounds[5] = Math.max(bounds[5], vertices[i * 3 + 2]);
        }
        float[][] planes = new float[6][];
        for (int i = 0; i < 6; i++) {
            planes[i] = new float[4];
            float m = i < 3 ? -1 : 1;
            int vi = i < 3 ? 0 : 7;
            planes[i][0] = m * normals[i % 3][0];
            planes[i][1] = m * normals[i % 3][1];
            planes[i][2] = m * normals[i % 3][2];
            planes[i][3] = vertices[vi * 3] * planes[i][0] + vertices[vi * 3 + 1] * planes[i][1]
                    + vertices[vi * 3 + 2] * planes[i][2];
        }
        rasterizationFilledShape(hf, bounds, area, flagMergeThr, rectangle -> intersectBox(rectangle, vertices, planes));
    }

    public static void main(String[] args) {
        rasterizeBox(null, new float[] { 10, 0, -3 }, new float[] { 5, 3, 6 }, new float[] { 0, 1, 0 }, 0, 0, null);
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
        float tmin = (float) (-b - Math.sqrt(discr));
        float tmax = (float) (-b + Math.sqrt(discr));

        if (tmin < 0.0f) {
            tmin = 0.0f;
        }
        return new float[] { y + tmin, y + tmax };
    }

    private static float[] getUprightNormal(float[] axis) {
        // cross product of side vector & axis, side = cross product of (0,1,0) and axis
        // side = cross(up, axis)
        // side.x = up.y * axis.z - up.z * axis.y
        // side.y = up.z * axis.x - up.x * axis.z
        // side.z = up.x * axis.y - up.y * axis.x
        // side[0] = axis[2]
        // side[1] = 0
        // side[2] = -axis[0]
        // normal = cross(axis, side)
        // normal.x = axis.y * side.z - axis.z * side.y
        // normal.y = axis.z * side.x - axis.x * side.z
        // normal.z = axis.x * side.y - axis.y * side.x
        // normal[0] = - axis[1] * axis[0]
        // normal[1] = axis[2] * axis[2] + axis[0] * axis[0]
        // normal[2] = -axis[1] * axis[2]
        float[] normal = new float[] { -axis[1] * axis[0], axis[2] * axis[2] + axis[0] * axis[0], -axis[1] * axis[2] };
        float l = (float) Math.sqrt(lenSqr(normal));
        normal[0] /= l;
        normal[1] /= l;
        normal[2] /= l;
        return normal;
    }

    private static float[] intersectCapsule(float[] rectangle, float[] start, float[] end, float[] axis, float[] normal,
            float axisLenSqr, float radius, float radiusSqr) {
        float[] s = mergeIntersections(intersectSphere(rectangle, start, radiusSqr), intersectSphere(rectangle, end, radiusSqr));
        boolean slabIntersection = false;
        float axisLen2dSqr = axis[0] * axis[0] + axis[2] * axis[2];
        if (axisLen2dSqr > EPSILON) {
            // check slabs (face planes)
            for (int i = 0; i < 4; i++) {
                float planeD = rectangle[i];
                // 0: min x plane, 1: min z plane, 2: max x plane, 3: max z plane
                // cylinder intersection is simplified to intersection with 2 planes ("top" and "bottom" since the ray
                // is vertical)
                float planeDotAxis = ((i & 1) == 0) ? axis[0] : axis[2];
                float planeDotA = ((i & 1) == 0) ? start[0] : start[2];
                float t = (planeD - planeDotA) / planeDotAxis;
                float[] intersection = new float[] { start[0] + t * axis[0], rectangle[1], start[2] + t * axis[2] };
                if (((i == 0 || i == 2) && intersection[2] >= rectangle[1] && intersection[2] <= rectangle[3])
                        || ((i == 1 || i == 3) && intersection[0] >= rectangle[0] && intersection[0] <= rectangle[2])) {
                    s = mergeIntersections(s, intersectCylinder(intersection, start, axis, normal, radius, axisLenSqr));
                    slabIntersection = true;
                }
            }
            if (!slabIntersection) {
                // check vertices
                for (int i = 0; i < 4; i++) {
                    float x = ((i & 1) == 0) ? rectangle[0] : rectangle[2];
                    float z = ((i & 2) == 0) ? rectangle[1] : rectangle[3];
                    // distance to axis
                    float[] p = projectPointOnLine2d(x, z, start, axis, axisLen2dSqr);
                    float dx = x - p[0];
                    float dz = z - p[2];
                    float dSqr = dx * dx + dz * dz;
                    // scale radius based on distance
                    float rSqr = radiusSqr - dSqr;
                    if (rSqr >= 0.0f) {
                        float r = (float) Math.sqrt(rSqr);
                        float[] point = new float[] { x, rectangle[1], z };
                        s = mergeIntersections(s, intersectCylinder(point, start, axis, normal, r, axisLenSqr));
                    }
                }
            }
        }
        return s;
    }

    private static float[] intersectCylinder(float[] point, float[] start, float[] axis, float[] normal, float radius,
            float axisLenSqr) {
        // top plane intersection
        float nrx = normal[0] * radius;
        float nry = normal[1] * radius;
        float nrz = normal[2] * radius;
        float topX = start[0] + nrx;
        float topY = start[1] + nry;
        float topZ = start[2] + nrz;
        float topPlaneD = normal[0] * topX + normal[1] * topY + normal[2] * topZ;
        float dotNormalPoint = dot(normal, point);
        float t = (topPlaneD - dotNormalPoint) / normal[1];
        float upperY = point[1] + t;
        // check if within the top segment
        float vx = point[0] - topX;
        float vy = upperY - topY;
        float vz = point[2] - topZ;
        float dot = vx * axis[0] + vy * axis[1] + vz * axis[2];
        float proj = dot / axisLenSqr;
        boolean topHit = proj >= 0.0f && proj <= 1.0f;
        if (proj < 0.0f) {
            upperY = start[1];
        } else if (proj > 1.0f) {
            upperY = start[1] + axis[1];
        }
        // bottom plane intersection
        float bottomX = start[0] - nrx;
        float bottomY = start[1] - nry;
        float bottomZ = start[2] - nrz;
        float bottomPlaneD = normal[0] * bottomX + normal[1] * bottomY + normal[2] * bottomZ;
        t = (bottomPlaneD - dotNormalPoint) / normal[1];
        float lowerY = point[1] + t;
        // check if within the bottom segment
        vx = point[0] - bottomX;
        vy = lowerY - bottomY;
        vz = point[2] - bottomZ;
        dot = vx * axis[0] + vy * axis[1] + vz * axis[2];
        proj = dot / axisLenSqr;
        boolean bottomHit = proj >= 0.0f && proj <= 1.0f;
        if (proj < 0.0f) {
            lowerY = start[1];
        } else if (proj > 1.0f) {
            lowerY = start[1] + axis[1];
        }
        if (topHit || bottomHit) {
            return new float[] { lowerY, upperY };
        }
        return null;
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

    private static float[] projectPointOnLine2d(float px, float py, float[] a, float[] ab, float abLenSqr) {
        float vx = px - a[0];
        float vz = py - a[2];
        float dot = vx * ab[0] + vz * ab[2];
        float t = dot / abLenSqr;
        return new float[] { a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2] };
    }

    private static float lenSqr(float[] v) {
        return lenSqr(v[0], v[1], v[2]);
    }

    private static float lenSqr(float dx, float dy, float dz) {
        return dx * dx + dy * dy + dz * dz;
    }

    public static float clamp(float v, float min, float max) {
        return Math.max(Math.min(max, v), min);
    }

    public static float dot(float[] v1, float[] v2) {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }

    private static boolean overlapBounds(float[] amin, float[] amax, float[] bounds) {
        boolean overlap = true;
        overlap = (amin[0] > bounds[3] || amax[0] < bounds[0]) ? false : overlap;
        overlap = (amin[1] > bounds[4]) ? false : overlap;
        overlap = (amin[2] > bounds[5] || amax[2] < bounds[2]) ? false : overlap;
        return overlap;
    }

}
