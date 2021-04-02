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

package org.recast4j.dynamic;

import org.recast4j.recast.AreaModification;
import org.recast4j.recast.RecastConstants.PartitionType;

public class DynamicNavMeshConfig {

    public final boolean useTiles;
    public final int tileSizeX;
    public final int tileSizeZ;
    public final float cellSize;
    public PartitionType partitionType = PartitionType.WATERSHED;
    public AreaModification walkableAreaModification = new AreaModification(1);
    public float walkableHeight;
    public float walkableSlopeAngle;
    public float walkableRadius;
    public float walkableClimb;
    public float minRegionArea;
    public float regionMergeArea;
    public float maxEdgeLen;
    public float maxSimplificationError;
    public int vertsPerPoly;
    public boolean buildDetailMesh;
    public float detailSampleDistance;
    public float detailSampleMaxError;
    public boolean filterLowHangingObstacles = true;
    public boolean filterLedgeSpans = true;
    public boolean filterWalkableLowHeightSpans = true;
    public boolean enableCheckpoints = true;
    public boolean keepIntermediateResults = false;

    DynamicNavMeshConfig(boolean useTiles, int tileSizeX, int tileSizeZ, float cellSize) {
        this.useTiles = useTiles;
        this.tileSizeX = tileSizeX;
        this.tileSizeZ = tileSizeZ;
        this.cellSize = cellSize;
    }

}
