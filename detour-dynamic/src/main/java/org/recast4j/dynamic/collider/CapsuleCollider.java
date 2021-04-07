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
import org.recast4j.recast.Telemetry;

public class CapsuleCollider extends AbstractCollider {

    private final float[] start;
    private final float[] end;
    private final float radius;

    public CapsuleCollider(float[] start, float[] end, float radius, int area, float flagMergeThreshold) {
        super(area, flagMergeThreshold, bounds(start, end, radius));
        this.start = start;
        this.end = end;
        this.radius = radius;
    }

    @Override
    public void rasterize(Heightfield hf, Telemetry telemetry) {
        RecastShapeRasterization.rasterizeCapsule(hf, start, end, radius, area, (int) Math.floor(flagMergeThreshold / hf.ch),
                telemetry);
    }

    private static float[] bounds(float[] start, float[] end, float radius) {
        return new float[] { Math.min(start[0], end[0]) - radius, Math.min(start[1], end[1]) - radius,
                Math.min(start[2], end[2]) - radius, Math.max(start[0], end[0]) + radius, Math.max(start[1], end[1]) + radius,
                Math.max(start[2], end[2]) + radius };
    }

}
