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

package org.recast4j.demo.geom;

import java.util.List;
import java.util.Optional;

import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;

public class PolyMeshRaycast {

    public static Optional<Float> raycast(List<RecastBuilderResult> results, float[] src, float[] dst) {
        for (RecastBuilderResult result : results) {
            if (result.getMeshDetail() != null) {
                Optional<Float> intersection = raycast(result.getMesh(), result.getMeshDetail(), src, dst);
                if (intersection.isPresent()) {
                    return intersection;
                }
            }
        }
        return Optional.empty();
    }

    private static Optional<Float> raycast(PolyMesh poly, PolyMeshDetail meshDetail, float[] sp, float[] sq) {
        if (meshDetail != null) {
            for (int i = 0; i < meshDetail.nmeshes; ++i) {
                int m = i * 4;
                int bverts = meshDetail.meshes[m];
                int btris = meshDetail.meshes[m + 2];
                int ntris = meshDetail.meshes[m + 3];
                int verts = bverts * 3;
                int tris = btris * 4;
                for (int j = 0; j < ntris; ++j) {
                    float vs[][] = new float[3][3];
                    for (int k = 0; k < 3; ++k) {
                        vs[k][0] = meshDetail.verts[verts + meshDetail.tris[tris + j * 4 + k] * 3];
                        vs[k][1] = meshDetail.verts[verts + meshDetail.tris[tris + j * 4 + k] * 3 + 1];
                        vs[k][2] = meshDetail.verts[verts + meshDetail.tris[tris + j * 4 + k] * 3 + 2];
                    }
                    Optional<Float> intersection = Intersections.intersectSegmentTriangle(sp, sq, vs[0], vs[1], vs[2]);
                    if (intersection.isPresent()) {
                        return intersection;
                    }
                }
            }
        } else {
            // TODO: check PolyMesh instead
        }
        return Optional.empty();
    }
}
