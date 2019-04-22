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
package org.recast4j.recast.geom;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.recast4j.recast.AreaModification;
import org.recast4j.recast.ConvexVolume;
import org.recast4j.recast.RecastVectors;

public class SimpleInputGeomProvider implements InputGeomProvider {

    public final float[] vertices;
    public final int[] faces;
    public final float[] normals;
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
        normals = new float[faces.length];
        calculateNormals();
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
        return volumes;
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

    public void calculateNormals() {
        for (int i = 0; i < faces.length; i += 3) {
            int v0 = faces[i] * 3;
            int v1 = faces[i + 1] * 3;
            int v2 = faces[i + 2] * 3;
            float[] e0 = new float[3], e1 = new float[3];
            for (int j = 0; j < 3; ++j) {
                e0[j] = vertices[v1 + j] - vertices[v0 + j];
                e1[j] = vertices[v2 + j] - vertices[v0 + j];
            }
            normals[i] = e0[1] * e1[2] - e0[2] * e1[1];
            normals[i + 1] = e0[2] * e1[0] - e0[0] * e1[2];
            normals[i + 2] = e0[0] * e1[1] - e0[1] * e1[0];
            float d = (float) Math
                    .sqrt(normals[i] * normals[i] + normals[i + 1] * normals[i + 1] + normals[i + 2] * normals[i + 2]);
            if (d > 0) {
                d = 1.0f / d;
                normals[i] *= d;
                normals[i + 1] *= d;
                normals[i + 2] *= d;
            }
        }
    }
}
