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
import org.recast4j.recast.RecastRasterization;
import org.recast4j.recast.Telemetry;

public class TrimeshCollider extends AbstractCollider {

    private final float[] vertices;
    private final int[] triangles;

    public TrimeshCollider(float[] vertices, int[] triangles, float[] bounds, int area, float flagMergeThreshold) {
        super(area, flagMergeThreshold, bounds);
        this.vertices = vertices;
        this.triangles = triangles;
    }

    @Override
    public void rasterize(Heightfield hf, Telemetry telemetry) {
        for (int i = 0; i < triangles.length; i += 3) {
            RecastRasterization.rasterizeTriangle(hf, vertices, triangles[i], triangles[i + 1], triangles[i + 2], area,
                    (int) Math.floor(flagMergeThreshold / hf.ch), telemetry);
        }
    }

}
