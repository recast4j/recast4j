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

package org.recast4j.recast.geom;

import java.util.Collections;
import java.util.List;

import org.recast4j.recast.ConvexVolume;
import org.recast4j.recast.RecastVectors;

public class SingleTrimeshInputGeomProvider implements InputGeomProvider {

    private final float[] bmin;
    private final float[] bmax;
    private final List<TriMesh> meshes;

    public SingleTrimeshInputGeomProvider(float[] vertices, int[] faces) {
        bmin = new float[3];
        bmax = new float[3];
        RecastVectors.copy(bmin, vertices, 0);
        RecastVectors.copy(bmax, vertices, 0);
        for (int i = 1; i < vertices.length / 3; i++) {
            RecastVectors.min(bmin, vertices, i * 3);
            RecastVectors.max(bmax, vertices, i * 3);
        }
        meshes = Collections.singletonList(new TriMesh(vertices, faces));
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
    public Iterable<TriMesh> meshes() {
        return meshes;
    }

    @Override
    public Iterable<ConvexVolume> convexVolumes() {
        return Collections.emptyList();
    }

}
