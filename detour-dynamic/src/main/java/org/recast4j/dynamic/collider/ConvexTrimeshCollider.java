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

import static org.recast4j.dynamic.collider.TrimeshCollider.computeBounds;

import org.recast4j.recast.Heightfield;
import org.recast4j.recast.RecastFilledVolumeRasterization;
import org.recast4j.recast.Telemetry;

public class ConvexTrimeshCollider extends AbstractCollider {

    private final float[] vertices;
    private final int[] triangles;

    public ConvexTrimeshCollider(float[] vertices, int[] triangles, int area, float flagMergeThreshold) {
        super(area, flagMergeThreshold, computeBounds(vertices));
        this.vertices = vertices;
        this.triangles = triangles;
    }

    public ConvexTrimeshCollider(float[] vertices, int[] triangles, float[] bounds, int area, float flagMergeThreshold) {
        super(area, flagMergeThreshold, bounds);
        this.vertices = vertices;
        this.triangles = triangles;
    }

    @Override
    public void rasterize(Heightfield hf, Telemetry telemetry) {
        RecastFilledVolumeRasterization.rasterizeConvex(hf, vertices, triangles, area,
                (int) Math.floor(flagMergeThreshold / hf.ch), telemetry);
    }

}
