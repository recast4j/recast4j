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

package org.recast4j.dynamic.io;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

import org.recast4j.dynamic.DynamicNavMesh;
import org.recast4j.dynamic.DynamicNavMeshConfig;
import org.recast4j.recast.AreaModification;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;

public class VoxelFile {

    public static final ByteOrder PREFERRED_BYTE_ORDER = ByteOrder.BIG_ENDIAN;
    public static final int MAGIC = 'V' << 24 | 'O' << 16 | 'X' << 8 | 'L';
    public static final int VERSION_EXPORTER_MASK = 0xF000;
    public static final int VERSION_COMPRESSION_MASK = 0x0F00;
    public int version;
    public PartitionType partitionType = PartitionType.WATERSHED;
    public boolean filterLowHangingObstacles = true;
    public boolean filterLedgeSpans = true;
    public boolean filterWalkableLowHeightSpans = true;
    public float walkableRadius;
    public float walkableHeight;
    public float walkableClimb;
    public float walkableSlopeAngle;
    public float cellSize;
    public float maxSimplificationError;
    public float maxEdgeLen;
    public float minRegionArea;
    public float regionMergeArea;
    public int vertsPerPoly;
    public boolean buildMeshDetail;
    public float detailSampleDistance;
    public float detailSampleMaxError;
    public boolean useTiles;
    public int tileSizeX;
    public int tileSizeZ;
    public float[] rotation = new float[3];
    public float[] bounds = new float[6];
    public int tileCount;
    public final List<VoxelTile> tiles = new ArrayList<>();

    public void addTile(VoxelTile tile) {
        tiles.add(tile);
    }

    public RecastConfig getConfig(VoxelTile tile, PartitionType partitionType, int maxPolyVerts, int regionMergeSize,
            boolean filterLowHangingObstacles, boolean filterLedgeSpans, boolean filterWalkableLowHeightSpans,
            AreaModification walkbableAreaMod, boolean buildMeshDetail, float detailSampleDist, float detailSampleMaxError) {
        return new RecastConfig(useTiles, tileSizeX, tileSizeZ, tile.borderSize, partitionType, cellSize, tile.cellHeight,
                walkableSlopeAngle, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, walkableHeight,
                walkableRadius, walkableClimb, minRegionArea, regionMergeArea, maxEdgeLen, maxSimplificationError, maxPolyVerts,
                buildMeshDetail, detailSampleDist, detailSampleMaxError, walkbableAreaMod);
    }

    public static VoxelFile from(RecastConfig config, List<RecastBuilderResult> results) {
        VoxelFile f = new VoxelFile();
        f.version = 1;
        f.partitionType = config.partitionType;
        f.filterLowHangingObstacles = config.filterLowHangingObstacles;
        f.filterLedgeSpans = config.filterLedgeSpans;
        f.filterWalkableLowHeightSpans = config.filterWalkableLowHeightSpans;
        f.walkableRadius = config.walkableRadiusWorld;
        f.walkableHeight = config.walkableHeightWorld;
        f.walkableClimb = config.walkableClimbWorld;
        f.walkableSlopeAngle = config.walkableSlopeAngle;
        f.cellSize = config.cs;
        f.maxSimplificationError = config.maxSimplificationError;
        f.maxEdgeLen = config.maxEdgeLenWorld;
        f.minRegionArea = config.minRegionAreaWorld;
        f.regionMergeArea = config.mergeRegionAreaWorld;
        f.vertsPerPoly = config.maxVertsPerPoly;
        f.buildMeshDetail = config.buildMeshDetail;
        f.detailSampleDistance = config.detailSampleDist;
        f.detailSampleMaxError = config.detailSampleMaxError;
        f.useTiles = config.useTiles;
        f.tileSizeX = config.tileSizeX;
        f.tileSizeZ = config.tileSizeZ;
        f.bounds = new float[] { Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY };
        f.tileCount = results.size();
        for (RecastBuilderResult r : results) {
            f.tiles.add(new VoxelTile(r.tileX, r.tileZ, r.getSolidHeightfield()));
            f.bounds[0] = Math.min(f.bounds[0], r.getSolidHeightfield().bmin[0]);
            f.bounds[1] = Math.min(f.bounds[1], r.getSolidHeightfield().bmin[1]);
            f.bounds[2] = Math.min(f.bounds[2], r.getSolidHeightfield().bmin[2]);
            f.bounds[3] = Math.max(f.bounds[3], r.getSolidHeightfield().bmax[0]);
            f.bounds[4] = Math.max(f.bounds[4], r.getSolidHeightfield().bmax[1]);
            f.bounds[5] = Math.max(f.bounds[5], r.getSolidHeightfield().bmax[2]);
        }
        return f;
    }

    public static VoxelFile from(DynamicNavMesh mesh) {
        VoxelFile f = new VoxelFile();
        f.version = 1;
        DynamicNavMeshConfig config = mesh.config;
        f.partitionType = config.partitionType;
        f.filterLowHangingObstacles = config.filterLowHangingObstacles;
        f.filterLedgeSpans = config.filterLedgeSpans;
        f.filterWalkableLowHeightSpans = config.filterWalkableLowHeightSpans;
        f.walkableRadius = config.walkableRadius;
        f.walkableHeight = config.walkableHeight;
        f.walkableClimb = config.walkableClimb;
        f.walkableSlopeAngle = config.walkableSlopeAngle;
        f.cellSize = config.cellSize;
        f.maxSimplificationError = config.maxSimplificationError;
        f.maxEdgeLen = config.maxEdgeLen;
        f.minRegionArea = config.minRegionArea;
        f.regionMergeArea = config.regionMergeArea;
        f.vertsPerPoly = config.vertsPerPoly;
        f.buildMeshDetail = config.buildDetailMesh;
        f.detailSampleDistance = config.detailSampleDistance;
        f.detailSampleMaxError = config.detailSampleMaxError;
        f.useTiles = config.useTiles;
        f.tileSizeX = config.tileSizeX;
        f.tileSizeZ = config.tileSizeZ;
        f.bounds = new float[] { Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY };
        List<RecastBuilderResult> results = mesh.recastResults();
        f.tileCount = results.size();
        for (RecastBuilderResult r : results) {
            f.tiles.add(new VoxelTile(r.tileX, r.tileZ, r.getSolidHeightfield()));
            f.bounds[0] = Math.min(f.bounds[0], r.getSolidHeightfield().bmin[0]);
            f.bounds[1] = Math.min(f.bounds[1], r.getSolidHeightfield().bmin[1]);
            f.bounds[2] = Math.min(f.bounds[2], r.getSolidHeightfield().bmin[2]);
            f.bounds[3] = Math.max(f.bounds[3], r.getSolidHeightfield().bmax[0]);
            f.bounds[4] = Math.max(f.bounds[4], r.getSolidHeightfield().bmax[1]);
            f.bounds[5] = Math.max(f.bounds[5], r.getSolidHeightfield().bmax[2]);
        }
        return f;
    }

}
