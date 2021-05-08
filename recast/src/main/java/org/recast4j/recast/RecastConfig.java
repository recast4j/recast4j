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

    public final boolean useTiles;
    /** The width/depth size of tile's on the xz-plane. [Limit: &gt;= 0] [Units: vx] **/
    public final int tileSizeX;
    public final int tileSizeZ;

    /** The xz-plane cell size to use for fields. [Limit: &gt; 0] [Units: wu] **/
    public final float cs;

    /** The y-axis cell size to use for fields. [Limit: &gt; 0] [Units: wu] **/
    public final float ch;

    /** The maximum slope that is considered walkable. [Limits: 0 &lt;= value &lt; 90] [Units: Degrees] **/
    public final float walkableSlopeAngle;

    /**
     * Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: &gt;= 3]
     * [Units: vx]
     **/
    public final int walkableHeight;

    /** Maximum ledge height that is considered to still be traversable. [Limit: &gt;=0] [Units: vx] **/
    public final int walkableClimb;

    /**
     * The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: &gt;=0] [Units:
     * vx]
     **/
    public final int walkableRadius;

    /** The maximum allowed length for contour edges along the border of the mesh. [Limit: &gt;=0] [Units: vx] **/
    public final int maxEdgeLen;

    /**
     * The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: &gt;=0]
     * [Units: vx]
     **/
    public final float maxSimplificationError;

    /** The minimum number of cells allowed to form isolated island areas. [Limit: &gt;=0] [Units: vx] **/
    public final int minRegionArea;

    /**
     * Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit:
     * &gt;=0] [Units: vx]
     **/
    public final int mergeRegionArea;

    /**
     * The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
     * [Limit: &gt;= 3]
     **/
    public final int maxVertsPerPoly;

    /**
     * Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >=
     * 0.9] [Units: wu]
     **/
    public final float detailSampleDist;

    /**
     * The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.)
     * [Limit: &gt;=0] [Units: wu]
     **/
    public final float detailSampleMaxError;

    public final AreaModification walkableAreaMod;
    public final boolean filterLowHangingObstacles;
    public final boolean filterLedgeSpans;
    public final boolean filterWalkableLowHeightSpans;
    /** Set to false to disable building detailed mesh **/
    public final boolean buildMeshDetail;
    /** The size of the non-navigable border around the heightfield. [Limit: &gt;=0] [Units: vx] **/
    public final int borderSize;
    /** Set of original settings passed in world units */
    public final float minRegionAreaWorld;
    public final float mergeRegionAreaWorld;
    public final float walkableHeightWorld;
    public final float walkableClimbWorld;
    public final float walkableRadiusWorld;
    public final float maxEdgeLenWorld;

    /**
     * Non-tiled build configuration
     */
    public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight, float agentRadius,
            float agentMaxClimb, float agentMaxSlope, int regionMinSize, int regionMergeSize, float edgeMaxLen,
            float edgeMaxError, int vertsPerPoly, float detailSampleDist, float detailSampleMaxError,
            AreaModification walkableAreaMod) {
        this(partitionType, cellSize, cellHeight, agentMaxSlope, true, true, true, agentHeight, agentRadius, agentMaxClimb,
                regionMinSize, regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError,
                walkableAreaMod, true);
    }

    /**
     * Non-tiled build configuration
     */
    public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentMaxSlope,
            boolean filterLowHangingObstacles, boolean filterLedgeSpans, boolean filterWalkableLowHeightSpans, float agentHeight,
            float agentRadius, float agentMaxClimb, int regionMinSize, int regionMergeSize, float edgeMaxLen, float edgeMaxError,
            int vertsPerPoly, float detailSampleDist, float detailSampleMaxError, AreaModification walkableAreaMod,
            boolean buildMeshDetail) {
        // Note: area = size*size in [Units: wu]
        this(false, 0, 0, 0, partitionType, cellSize, cellHeight, agentMaxSlope, filterLowHangingObstacles, filterLedgeSpans,
                filterWalkableLowHeightSpans, agentHeight, agentRadius, agentMaxClimb,
                regionMinSize * regionMinSize * cellSize * cellSize, regionMergeSize * regionMergeSize * cellSize * cellSize,
                edgeMaxLen, edgeMaxError, vertsPerPoly, buildMeshDetail, detailSampleDist, detailSampleMaxError, walkableAreaMod);
    }

    public RecastConfig(boolean useTiles, int tileSizeX, int tileSizeZ, int borderSize, PartitionType partitionType,
            float cellSize, float cellHeight, float agentMaxSlope, boolean filterLowHangingObstacles, boolean filterLedgeSpans,
            boolean filterWalkableLowHeightSpans, float agentHeight, float agentRadius, float agentMaxClimb, float minRegionArea,
            float mergeRegionArea, float edgeMaxLen, float edgeMaxError, int vertsPerPoly, boolean buildMeshDetail,
            float detailSampleDist, float detailSampleMaxError, AreaModification walkableAreaMod) {
        this.useTiles = useTiles;
        this.tileSizeX = tileSizeX;
        this.tileSizeZ = tileSizeZ;
        this.borderSize = borderSize;
        this.partitionType = partitionType;
        cs = cellSize;
        ch = cellHeight;
        walkableSlopeAngle = agentMaxSlope;
        walkableHeight = (int) Math.ceil(agentHeight / ch);
        walkableHeightWorld = agentHeight;
        walkableClimb = (int) Math.floor(agentMaxClimb / ch);
        walkableClimbWorld = agentMaxClimb;
        walkableRadius = (int) Math.ceil(agentRadius / cs);
        walkableRadiusWorld = agentRadius;
        this.minRegionArea = Math.round(minRegionArea / (cs * cs));
        minRegionAreaWorld = minRegionArea;
        this.mergeRegionArea = Math.round(mergeRegionArea / (cs * cs));
        mergeRegionAreaWorld = mergeRegionArea;
        maxEdgeLen = (int) (edgeMaxLen / cellSize);
        maxEdgeLenWorld = edgeMaxLen;
        maxSimplificationError = edgeMaxError;
        maxVertsPerPoly = vertsPerPoly;
        this.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
        this.detailSampleMaxError = cellHeight * detailSampleMaxError;
        this.walkableAreaMod = walkableAreaMod;
        this.filterLowHangingObstacles = filterLowHangingObstacles;
        this.filterLedgeSpans = filterLedgeSpans;
        this.filterWalkableLowHeightSpans = filterWalkableLowHeightSpans;
        this.buildMeshDetail = buildMeshDetail;
    }

    public static int calcBorder(float agentRadius, float cs) {
        return 3 + (int) Math.ceil(agentRadius / cs);
    }
}
