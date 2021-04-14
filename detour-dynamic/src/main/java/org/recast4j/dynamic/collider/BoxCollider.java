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

package org.recast4j.dynamic.collider;

import org.recast4j.recast.Heightfield;
import org.recast4j.recast.RecastShapeRasterization;
import org.recast4j.recast.RecastVectors;
import org.recast4j.recast.Telemetry;

public class BoxCollider extends AbstractCollider {

    private final float[] center;
    private final float[] extent;
    private final float[] forward;
    private final float[] up;

    public BoxCollider(float[] center, float[] extent, float[] forward, float[] up, int area, float flagMergeThreshold) {
        super(area, flagMergeThreshold, bounds(center, extent, forward, up));
        this.center = center;
        this.extent = extent;
        this.forward = forward;
        this.up = up;
    }

    private static float[] bounds(float[] center, float[] extent, float[] forward, float[] up) {
        RecastVectors.normalize(up);
        float[][] normals = new float[][] { new float[3], up, new float[3] };
        RecastVectors.cross(normals[2], forward, up);
        RecastVectors.normalize(normals[2]);
        RecastVectors.cross(normals[0], up, normals[2]);
        RecastVectors.normalize(normals[0]);

        float[] trX = new float[] { normals[2][0], up[0], normals[0][0] };
        float[] trY = new float[] { normals[2][1], up[1], normals[0][1] };
        float[] trZ = new float[] { normals[2][2], up[2], normals[0][2] };

        float[] bounds = new float[] { Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY };
        for (int i = 0; i < 8; ++i) {
            float[] pt = new float[] { ((i & 1) != 0) ? extent[0] : -extent[0], ((i & 2) != 0) ? extent[1] : -extent[1],
                    ((i & 4) != 0) ? extent[2] : -extent[2] };
            float vx = RecastVectors.dot(pt, trX) + center[0];
            float vy = RecastVectors.dot(pt, trY) + center[1];
            float vz = RecastVectors.dot(pt, trZ) + center[2];
            bounds[0] = Math.min(bounds[0], vx);
            bounds[1] = Math.min(bounds[1], vy);
            bounds[2] = Math.min(bounds[2], vz);
            bounds[3] = Math.max(bounds[3], vx);
            bounds[4] = Math.max(bounds[4], vy);
            bounds[5] = Math.max(bounds[5], vz);
        }
        return bounds;
    }

    @Override
    public void rasterize(Heightfield hf, Telemetry telemetry) {
        RecastShapeRasterization.rasterizeBox(hf, center, extent, forward, up, area, (int) Math.floor(flagMergeThreshold / hf.ch),
                telemetry);

    }
}
