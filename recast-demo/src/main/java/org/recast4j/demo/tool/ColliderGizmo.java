package org.recast4j.demo.tool;

import static org.recast4j.demo.tool.GizmoHelper.generateCylindricalTriangles;
import static org.recast4j.demo.tool.GizmoHelper.generateCylindricalVertices;
import static org.recast4j.demo.tool.GizmoHelper.generateSphericalTriangles;
import static org.recast4j.demo.tool.GizmoHelper.generateSphericalVertices;
import static org.recast4j.detour.DetourCommon.clamp;
import static org.recast4j.detour.DetourCommon.vLen;
import static org.recast4j.recast.RecastVectors.cross;
import static org.recast4j.recast.RecastVectors.normalize;

import java.util.Arrays;

import org.recast4j.demo.draw.DebugDraw;
import org.recast4j.demo.draw.DebugDrawPrimitives;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.dynamic.collider.BoxCollider;
import org.recast4j.recast.RecastVectors;

public interface ColliderGizmo {

    void render(RecastDebugDraw debugDraw);

    public static ColliderGizmo box(float[] center, float[][] halfEdges) {
        return new BoxGizmo(center, halfEdges);
    }

    public static ColliderGizmo sphere(float[] center, float radius) {
        return new SphereGizmo(center, radius);
    }

    public static ColliderGizmo capsule(float[] start, float[] end, float radius) {
        return new CapsuleGizmo(start, end, radius);
    }

    public static ColliderGizmo cylinder(float[] start, float[] end, float radius) {
        return new CylinderGizmo(start, end, radius);
    }

    public static ColliderGizmo trimesh(float[] verts, int[] faces) {
        return new TrimeshGizmo(verts, faces);
    }

    public static ColliderGizmo composite(ColliderGizmo... gizmos) {
        return new CompositeGizmo(gizmos);
    }

    public static class BoxGizmo implements ColliderGizmo {
        private static final int[] TRIANLGES = { 0, 1, 2, 0, 2, 3, 4, 7, 6, 4, 6, 5, 0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2,
                2, 6, 7, 2, 7, 3, 4, 0, 3, 4, 3, 7 };
        private static final float[][] VERTS = { { -1, -1, -1 }, { 1, -1, -1 }, { 1, -1, 1 }, { -1, -1, 1 },
                { -1, 1, -1 }, { 1, 1, -1 }, { 1, 1, 1 }, { -1, 1, 1 } };

        private final float[] vertices = new float[8 * 3];
        private final float[] center;
        private final float[][] halfEdges;

        public BoxGizmo(float[] center, float[] extent, float[] forward, float[] up) {
            this(center, BoxCollider.getHalfEdges(up, forward, extent));
        }

        public BoxGizmo(float[] center, float[][] halfEdges) {
            this.center = center;
            this.halfEdges = halfEdges;
            for (int i = 0; i < 8; ++i) {
                float s0 = (i & 1) != 0 ? 1f : -1f;
                float s1 = (i & 2) != 0 ? 1f : -1f;
                float s2 = (i & 4) != 0 ? 1f : -1f;
                vertices[i * 3 + 0] = center[0] + s0 * halfEdges[0][0] + s1 * halfEdges[1][0] + s2 * halfEdges[2][0];
                vertices[i * 3 + 1] = center[1] + s0 * halfEdges[0][1] + s1 * halfEdges[1][1] + s2 * halfEdges[2][1];
                vertices[i * 3 + 2] = center[2] + s0 * halfEdges[0][2] + s1 * halfEdges[1][2] + s2 * halfEdges[2][2];
            }
        }

        @Override
        public void render(RecastDebugDraw debugDraw) {
            float[] trX = new float[] { halfEdges[0][0], halfEdges[1][0], halfEdges[2][0] };
            float[] trY = new float[] { halfEdges[0][1], halfEdges[1][1], halfEdges[2][1] };
            float[] trZ = new float[] { halfEdges[0][2], halfEdges[1][2], halfEdges[2][2] };
            float[] vertices = new float[8 * 3];
            for (int i = 0; i < 8; i++) {
                vertices[i * 3 + 0] = RecastVectors.dot(VERTS[i], trX) + center[0];
                vertices[i * 3 + 1] = RecastVectors.dot(VERTS[i], trY) + center[1];
                vertices[i * 3 + 2] = RecastVectors.dot(VERTS[i], trZ) + center[2];
            }
            debugDraw.begin(DebugDrawPrimitives.TRIS);
            for (int i = 0; i < 12; i++) {
                int col = DebugDraw.duRGBA(200, 200, 50, 160);
                if (i == 4 || i == 5 || i == 8 || i == 9) {
                    col = DebugDraw.duRGBA(160, 160, 40, 160);
                } else if (i > 4) {
                    col = DebugDraw.duRGBA(120, 120, 30, 160);
                }
                for (int j = 0; j < 3; j++) {
                    int v = TRIANLGES[i * 3 + j] * 3;
                    debugDraw.vertex(vertices[v], vertices[v + 1], vertices[v + 2], col);
                }
            }
            debugDraw.end();
        }
    }

    public static class SphereGizmo implements ColliderGizmo {
        private final float[] vertices;
        private final int[] triangles;
        private final float radius;
        private final float[] center;

        public SphereGizmo(float[] center, float radius) {
            this.center = center;
            this.radius = radius;
            vertices = generateSphericalVertices();
            triangles = generateSphericalTriangles();
        }

        @Override
        public void render(RecastDebugDraw debugDraw) {
            debugDraw.begin(DebugDrawPrimitives.TRIS);
            for (int i = 0; i < triangles.length; i += 3) {
                for (int j = 0; j < 3; j++) {
                    int v = triangles[i + j] * 3;
                    float c = clamp(0.57735026f * (vertices[v] + vertices[v + 1] + vertices[v + 2]), -1, 1);
                    int col = DebugDraw.duLerpCol(DebugDraw.duRGBA(32, 32, 0, 160), DebugDraw.duRGBA(220, 220, 0, 160),
                            (int) (127 * (1 + c)));
                    debugDraw.vertex(radius * vertices[v] + center[0], radius * vertices[v + 1] + center[1],
                            radius * vertices[v + 2] + center[2], col);
                }
            }
            debugDraw.end();
        }

    }

    public static class CapsuleGizmo implements ColliderGizmo {
        private final float[] vertices;
        private final int[] triangles;
        private final float[] center;
        private final float[] gradient;

        public CapsuleGizmo(float[] start, float[] end, float radius) {
            center = new float[] { 0.5f * (start[0] + end[0]), 0.5f * (start[1] + end[1]),
                    0.5f * (start[2] + end[2]) };
            float[] axis = new float[] { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
            float[][] normals = new float[3][];
            normals[1] = new float[] { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
            normalize(normals[1]);
            normals[0] = getSideVector(axis);
            normals[2] = new float[3];
            cross(normals[2], normals[0], normals[1]);
            normalize(normals[2]);
            triangles = generateSphericalTriangles();
            float[] trX = new float[] { normals[0][0], normals[1][0], normals[2][0] };
            float[] trY = new float[] { normals[0][1], normals[1][1], normals[2][1] };
            float[] trZ = new float[] { normals[0][2], normals[1][2], normals[2][2] };
            float[] spVertices = generateSphericalVertices();
            float halfLength = 0.5f * vLen(axis);
            vertices = new float[spVertices.length];
            gradient = new float[spVertices.length / 3];
            float[] v = new float[3];
            for (int i = 0; i < spVertices.length; i += 3) {
                float offset = (i >= spVertices.length / 2) ? -halfLength : halfLength;
                float x = radius * spVertices[i];
                float y = radius * spVertices[i + 1] + offset;
                float z = radius * spVertices[i + 2];
                vertices[i] = x * trX[0] + y * trX[1] + z * trX[2] + center[0];
                vertices[i + 1] = x * trY[0] + y * trY[1] + z * trY[2] + center[1];
                vertices[i + 2] = x * trZ[0] + y * trZ[1] + z * trZ[2] + center[2];
                v[0] = vertices[i] - center[0];
                v[1] = vertices[i + 1] - center[1];
                v[2] = vertices[i + 2] - center[2];
                normalize(v);
                gradient[i / 3] = clamp(0.57735026f * (v[0] + v[1] + v[2]), -1, 1);
            }

        }

        private float[] getSideVector(float[] axis) {
            float[] side = { 1, 0, 0 };
            if (axis[0] > 0.8) {
                side = new float[] { 0, 0, 1 };
            }
            float[] forward = new float[3];
            cross(forward, side, axis);
            cross(side, axis, forward);
            normalize(side);
            return side;
        }

        @Override
        public void render(RecastDebugDraw debugDraw) {

            debugDraw.begin(DebugDrawPrimitives.TRIS);
            for (int i = 0; i < triangles.length; i += 3) {
                for (int j = 0; j < 3; j++) {
                    int v = triangles[i + j] * 3;
                    float c = gradient[triangles[i + j]];
                    int col = DebugDraw.duLerpCol(DebugDraw.duRGBA(32, 32, 0, 160), DebugDraw.duRGBA(220, 220, 0, 160),
                            (int) (127 * (1 + c)));
                    debugDraw.vertex(vertices[v], vertices[v + 1], vertices[v + 2], col);
                }
            }
            debugDraw.end();
        }

    }

    public static class CylinderGizmo implements ColliderGizmo {
        private final float[] vertices;
        private final int[] triangles;
        private final float[] center;
        private final float[] gradient;

        public CylinderGizmo(float[] start, float[] end, float radius) {
            center = new float[] { 0.5f * (start[0] + end[0]), 0.5f * (start[1] + end[1]),
                    0.5f * (start[2] + end[2]) };
            float[] axis = new float[] { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
            float[][] normals = new float[3][];
            normals[1] = new float[] { end[0] - start[0], end[1] - start[1], end[2] - start[2] };
            normalize(normals[1]);
            normals[0] = getSideVector(axis);
            normals[2] = new float[3];
            cross(normals[2], normals[0], normals[1]);
            normalize(normals[2]);
            triangles = generateCylindricalTriangles();
            float[] trX = new float[] { normals[0][0], normals[1][0], normals[2][0] };
            float[] trY = new float[] { normals[0][1], normals[1][1], normals[2][1] };
            float[] trZ = new float[] { normals[0][2], normals[1][2], normals[2][2] };
            vertices = generateCylindricalVertices();
            float halfLength = 0.5f * vLen(axis);
            gradient = new float[vertices.length / 3];
            float[] v = new float[3];
            for (int i = 0; i < vertices.length; i += 3) {
                float offset = (i >= vertices.length / 2) ? -halfLength : halfLength;
                float x = radius * vertices[i];
                float y = vertices[i + 1] + offset;
                float z = radius * vertices[i + 2];
                vertices[i] = x * trX[0] + y * trX[1] + z * trX[2] + center[0];
                vertices[i + 1] = x * trY[0] + y * trY[1] + z * trY[2] + center[1];
                vertices[i + 2] = x * trZ[0] + y * trZ[1] + z * trZ[2] + center[2];
                if (i < vertices.length / 4 || i >= 3 * vertices.length / 4) {
                    gradient[i / 3] = 1;
                } else {
                    v[0] = vertices[i] - center[0];
                    v[1] = vertices[i + 1] - center[1];
                    v[2] = vertices[i + 2] - center[2];
                    normalize(v);
                    gradient[i / 3] = clamp(0.57735026f * (v[0] + v[1] + v[2]), -1, 1);
                }
            }
        }

        private float[] getSideVector(float[] axis) {
            float[] side = { 1, 0, 0 };
            if (axis[0] > 0.8) {
                side = new float[] { 0, 0, 1 };
            }
            float[] forward = new float[3];
            cross(forward, side, axis);
            cross(side, axis, forward);
            normalize(side);
            return side;
        }

        @Override
        public void render(RecastDebugDraw debugDraw) {

            debugDraw.begin(DebugDrawPrimitives.TRIS);
            for (int i = 0; i < triangles.length; i += 3) {
                for (int j = 0; j < 3; j++) {
                    int v = triangles[i + j] * 3;
                    float c = gradient[triangles[i + j]];
                    int col = DebugDraw.duLerpCol(DebugDraw.duRGBA(32, 32, 0, 160), DebugDraw.duRGBA(220, 220, 0, 160),
                            (int) (127 * (1 + c)));
                    debugDraw.vertex(vertices[v], vertices[v + 1], vertices[v + 2], col);
                }
            }
            debugDraw.end();
        }

    }

    public static class TrimeshGizmo implements ColliderGizmo {
        private final float[] vertices;
        private final int[] triangles;

        public TrimeshGizmo(float[] vertices, int[] triangles) {
            this.vertices = vertices;
            this.triangles = triangles;
        }

        @Override
        public void render(RecastDebugDraw debugDraw) {
            debugDraw.begin(DebugDrawPrimitives.TRIS);
            for (int i = 0; i < triangles.length; i += 3) {
                int v0 = 3 * triangles[i];
                int v1 = 3 * triangles[i + 1];
                int v2 = 3 * triangles[i + 2];
                int col = GizmoHelper.getColorByNormal(vertices, v0, v1, v2);
                debugDraw.vertex(vertices[v0], vertices[v0 + 1], vertices[v0 + 2], col);
                debugDraw.vertex(vertices[v1], vertices[v1 + 1], vertices[v1 + 2], col);
                debugDraw.vertex(vertices[v2], vertices[v2 + 1], vertices[v2 + 2], col);
            }
            debugDraw.end();
        }

    }

    public static class CompositeGizmo implements ColliderGizmo {

        private final ColliderGizmo[] gizmos;

        public CompositeGizmo(ColliderGizmo... gizmos) {
            this.gizmos = gizmos;
        }

        @Override
        public void render(RecastDebugDraw debugDraw) {
            Arrays.stream(gizmos).forEach(g -> g.render(debugDraw));
        }
    }
}
