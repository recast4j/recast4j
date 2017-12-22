package org.recast4j.recast.geom;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.recast4j.recast.AreaModification;
import org.recast4j.recast.ConvexVolume;
import org.recast4j.recast.RecastVectors;

public class SimpleInputGeomProvider implements InputGeomProvider {

    final float[] vertices;
    final int[] faces;
    final float[] bmin;
    final float[] bmax;
    final List<ConvexVolume> volumes = new ArrayList<>();

    public SimpleInputGeomProvider(List<Float> vertexPositions, List<Integer> meshFaces) {
        this(mapVertices(vertexPositions), mapFaces(meshFaces));
    }

    private static int[] mapFaces(List<Integer> meshFaces) {
        int[] faces = new int[meshFaces.size()];
        for (int i = 0; i < faces.length; i++) {
            faces[i] = meshFaces.get(i);
        }
        return faces;
    }

    private static float[] mapVertices(List<Float> vertexPositions) {
        float[] vertices = new float[vertexPositions.size()];
        for (int i = 0; i < vertices.length; i++) {
            vertices[i] = vertexPositions.get(i);
        }
        return vertices;
    }

    public SimpleInputGeomProvider(float[] vertices, int[] faces) {
        this.vertices = vertices;
        this.faces = faces;
        bmin = new float[3];
        bmax = new float[3];
        RecastVectors.copy(bmin, vertices, 0);
        RecastVectors.copy(bmax, vertices, 0);
        for (int i = 1; i < vertices.length / 3; i++) {
            RecastVectors.min(bmin, vertices, i * 3);
            RecastVectors.max(bmax, vertices, i * 3);
        }
    }

    @Override
    public float[] getMeshBoundsMin() {
        return bmin;
    }

    @Override
    public float[] getMeshBoundsMax() {
        return bmax;
    }

    @Override
    public List<ConvexVolume> getConvexVolumes() {
        return new ArrayList<>();
    }

    public void addConvexVolume(float[] verts, float minh, float maxh, AreaModification areaMod) {
        ConvexVolume vol = new ConvexVolume();
        vol.hmin = minh;
        vol.hmax = maxh;
        vol.verts = verts;
        vol.areaMod = areaMod;
        volumes.add(vol);
    }

    @Override
    public Iterable<TriMesh> meshes() {
        return Collections.singletonList(new TriMesh(vertices, faces));
    }
}
