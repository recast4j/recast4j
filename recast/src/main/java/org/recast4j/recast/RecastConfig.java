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
package org.recast4j.recast;

import org.recast4j.recast.RecastConstants.PartitionType;

public class RecastConfig {
    public final PartitionType partitionType;

    /** The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx] **/
    public final int tileSize;

    /** The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] **/
    public final float cs;

    /** The y-axis cell size to use for fields. [Limit: > 0] [Units: wu] **/
    public final float ch;

    /** The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] **/
    public final float walkableSlopeAngle;

    /**
     * Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3]
     * [Units: vx]
     **/
    public final int walkableHeight;

    /** Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] **/
    public final int walkableClimb;

    /**
     * The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: >=0] [Units:
     * vx]
     **/
    public final int walkableRadius;

    /** The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] **/
    public final int maxEdgeLen;

    /**
     * The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0]
     * [Units: vx]
     **/
    public final float maxSimplificationError;

    /** The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] **/
    public final int minRegionArea;

    /**
     * Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit:
     * >=0] [Units: vx]
     **/
    public final int mergeRegionArea;

    /**
     * The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
     * [Limit: >= 3]
     **/
    public final int maxVertsPerPoly;

    /**
     * Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >=
     * 0.9] [Units: wu]
     **/
    public final float detailSampleDist;

    /**
     * The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.)
     * [Limit: >=0] [Units: wu]
     **/
    public final float detailSampleMaxError;

    public final AreaModification walkableAreaMod;
    public final boolean filterLowHangingObstacles;
    public final boolean filterLedgeSpans;
    public final boolean filterWalkableLowHeightSpans;

    public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight,
            float agentRadius, float agentMaxClimb, float agentMaxSlope, int regionMinSize, int regionMergeSize,
            float edgeMaxLen, float edgeMaxError, int vertsPerPoly, float detailSampleDist, float detailSampleMaxError,
            int tileSize, AreaModification walkableAreaMod) {
        this(partitionType, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, regionMinSize,
                regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError,
                tileSize, walkableAreaMod, true, true, true);
    }

    public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight,
            float agentRadius, float agentMaxClimb, float agentMaxSlope, int regionMinSize, int regionMergeSize,
            float edgeMaxLen, float edgeMaxError, int vertsPerPoly, float detailSampleDist, float detailSampleMaxError,
            int tileSize, AreaModification walkableAreaMod, boolean filterLowHangingObstacles, boolean filterLedgeSpans,
            boolean filterWalkableLowHeightSpans) {
        this.partitionType = partitionType;
        cs = cellSize;
        ch = cellHeight;
        walkableSlopeAngle = agentMaxSlope;
        walkableHeight = (int) Math.ceil(agentHeight / ch);
        walkableClimb = (int) Math.floor(agentMaxClimb / ch);
        walkableRadius = (int) Math.ceil(agentRadius / cs);
        maxEdgeLen = (int) (edgeMaxLen / cellSize);
        maxSimplificationError = edgeMaxError;
        minRegionArea = regionMinSize * regionMinSize; // Note: area = size*size
        mergeRegionArea = regionMergeSize * regionMergeSize; // Note: area = size*size
        maxVertsPerPoly = vertsPerPoly;
        this.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
        this.detailSampleMaxError = cellHeight * detailSampleMaxError;
        this.tileSize = tileSize;
        this.walkableAreaMod = walkableAreaMod;
        this.filterLowHangingObstacles = filterLowHangingObstacles;
        this.filterLedgeSpans = filterLedgeSpans;
        this.filterWalkableLowHeightSpans = filterWalkableLowHeightSpans;
    }

}
