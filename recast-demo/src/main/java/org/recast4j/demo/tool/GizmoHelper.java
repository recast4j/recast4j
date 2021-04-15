package org.recast4j.demo.tool;

import static org.recast4j.detour.DetourCommon.clamp;

import org.recast4j.demo.draw.DebugDraw;
import org.recast4j.recast.RecastVectors;

public class GizmoHelper {

    private static final int SEGMENTS = 16;
    private static final int RINGS = 8;

    private static float[] sphericalVertices;

    static float[] generateSphericalVertices() {
        if (sphericalVertices == null) {
            sphericalVertices = generateSphericalVertices(SEGMENTS, RINGS);
        }
        return sphericalVertices;
    }

    private static float[] generateSphericalVertices(int segments, int rings) {
        float[] vertices = new float[6 + 3 * (segments + 1) * (rings + 1)];
        // top
        int vi = 0;
        vertices[vi++] = 0;
        vertices[vi++] = 1;
        vertices[vi++] = 0;
        for (int r = 0; r <= rings; r++) {
            double theta = Math.PI * (r + 1) / (rings + 2);
            double cosTheta = Math.cos(theta);
            double sinTheta = Math.sin(theta);
            for (int p = 0; p <= segments; p++) {
                double phi = 2 * Math.PI * p / segments;
                double cosPhi = Math.cos(phi);
                double sinPhi = Math.sin(phi);
                vertices[vi++] = (float) (sinTheta * cosPhi);
                vertices[vi++] = (float) cosTheta;
                vertices[vi++] = (float) (sinTheta * sinPhi);
            }
        }
        // bottom
        vertices[vi++] = 0;
        vertices[vi++] = -1;
        vertices[vi++] = 0;
        return vertices;
    }

    static int[] generateSphericalTriangles() {
        return generateSphericalTriangles(SEGMENTS, RINGS);
    }

    private static int[] generateSphericalTriangles(int segments, int rings) {
        int[] triangles = new int[6 * (segments + rings * (segments + 1))];
        int ti = 0;
        for (int p = 0; p < segments; p++) {
            triangles[ti++] = p + 2;
            triangles[ti++] = p + 1;
            triangles[ti++] = 0;
        }
        for (int r = 0; r < rings; r++) {
            for (int p = 0; p < segments; p++) {
                int current = p + r * (segments + 1) + 1;
                int next = p + 1 + r * (segments + 1) + 1;
                int currentBottom = p + (r + 1) * (segments + 1) + 1;
                int nextBottom = p + 1 + (r + 1) * (segments + 1) + 1;
                triangles[ti++] = current;
                triangles[ti++] = next;
                triangles[ti++] = nextBottom;
                triangles[ti++] = current;
                triangles[ti++] = nextBottom;
                triangles[ti++] = currentBottom;
            }
        }
        int lastVertex = 1 + (segments + 1) * (rings + 1);
        for (int p = 0; p < segments; p++) {
            triangles[ti++] = lastVertex;
            triangles[ti++] = lastVertex - (p + 2);
            triangles[ti++] = lastVertex - (p + 1);
        }
        return triangles;
    }

    static int getColorByNormal(float[] vertices, int v0, int v1, int v2) {
        float[] e0 = new float[3], e1 = new float[3];
        float[] normal = new float[3];
        for (int j = 0; j < 3; ++j) {
            e0[j] = vertices[v1 + j] - vertices[v0 + j];
            e1[j] = vertices[v2 + j] - vertices[v0 + j];
        }
        normal[0] = e0[1] * e1[2] - e0[2] * e1[1];
        normal[1] = e0[2] * e1[0] - e0[0] * e1[2];
        normal[2] = e0[0] * e1[1] - e0[1] * e1[0];
        RecastVectors.normalize(normal);
        float c = clamp(0.57735026f * (normal[0] + normal[1] + normal[2]), -1, 1);
        int col = DebugDraw.duLerpCol(DebugDraw.duRGBA(32, 32, 0, 160), DebugDraw.duRGBA(220, 220, 0, 160),
                (int) (127 * (1 + c)));
        return col;
    }

}
