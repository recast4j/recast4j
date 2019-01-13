/*
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.demo.builder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.detour.DetourCommon;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.NavMeshParams;
import org.recast4j.detour.Tupple2;
import org.recast4j.recast.Recast;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;

public class TileNavMeshBuilder extends AbstractNavMeshBuilder {

    public Tupple2<List<RecastBuilderResult>, NavMesh> build(DemoInputGeomProvider m_geom,
            PartitionType m_partitionType, float m_cellSize, float m_cellHeight, float m_agentHeight,
            float m_agentRadius, float m_agentMaxClimb, float m_agentMaxSlope, int m_regionMinSize,
            int m_regionMergeSize, float m_edgeMaxLen, float m_edgeMaxError, int m_vertsPerPoly,
            float m_detailSampleDist, float m_detailSampleMaxError, boolean filterLowHangingObstacles,
            boolean filterLedgeSpans, boolean filterWalkableLowHeightSpans, int tileSize) {

        RecastBuilderResult[][] rcResult = buildRecastResult(m_geom, m_partitionType, m_cellSize, m_cellHeight,
                m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize,
                m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError,
                filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, tileSize);
        return new Tupple2<>(toList(rcResult), buildNavMesh(m_geom, buildMeshData(m_geom, m_cellSize, m_cellHeight,
                m_agentHeight, m_agentRadius, m_agentMaxClimb, rcResult), m_cellSize, tileSize, m_vertsPerPoly));
    }

    private List<RecastBuilderResult> toList(RecastBuilderResult[][] rcResult) {
        return Arrays.stream(rcResult).flatMap(Arrays::stream).collect(Collectors.toList());
    }

    private RecastBuilderResult[][] buildRecastResult(DemoInputGeomProvider m_geom, PartitionType m_partitionType,
            float m_cellSize, float m_cellHeight, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb,
            float m_agentMaxSlope, int m_regionMinSize, int m_regionMergeSize, float m_edgeMaxLen, float m_edgeMaxError,
            int m_vertsPerPoly, float m_detailSampleDist, float m_detailSampleMaxError,
            boolean filterLowHangingObstacles, boolean filterLedgeSpans, boolean filterWalkableLowHeightSpans,
            int tileSize) {
        RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, tileSize,
                SampleAreaModifications.SAMPLE_AREAMOD_WALKABLE, filterLowHangingObstacles, filterLedgeSpans,
                filterWalkableLowHeightSpans);
        RecastBuilder rcBuilder = new RecastBuilder();
        return rcBuilder.buildTiles(m_geom, cfg, 1);
    }

    private NavMesh buildNavMesh(DemoInputGeomProvider geom, List<MeshData> meshData, float cellSize, int tileSize,
            int vertsPerPoly) {
        NavMeshParams navMeshParams = new NavMeshParams();
        navMeshParams.orig[0] = geom.getMeshBoundsMin()[0];
        navMeshParams.orig[1] = geom.getMeshBoundsMin()[1];
        navMeshParams.orig[2] = geom.getMeshBoundsMin()[2];
        navMeshParams.tileWidth = tileSize * cellSize;
        navMeshParams.tileHeight = tileSize * cellSize;

        // snprintf(text, 64, "Tiles %d x %d", tw, th);

        navMeshParams.maxTiles = getMaxTiles(geom, cellSize, tileSize);
        navMeshParams.maxPolys = getMaxPolysPerTile(geom, cellSize, tileSize);
        NavMesh navMesh = new NavMesh(navMeshParams, vertsPerPoly);
        meshData.forEach(md -> navMesh.addTile(md, 0, 0));
        return navMesh;
    }

    public int getMaxTiles(DemoInputGeomProvider geom, float cellSize, int tileSize) {
        int tileBits = getTileBits(geom, cellSize, tileSize);
        return 1 << tileBits;
    }

    public int getMaxPolysPerTile(DemoInputGeomProvider geom, float cellSize, int tileSize) {
        int polyBits = 22 - getTileBits(geom, cellSize, tileSize);
        return 1 << polyBits;
    }

    private int getTileBits(DemoInputGeomProvider geom, float cellSize, int tileSize) {
        int[] wh = Recast.calcGridSize(geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), cellSize);
        int tw = (wh[0] + tileSize - 1) / tileSize;
        int th = (wh[1] + tileSize - 1) / tileSize;
        int tileBits = Math.min(DetourCommon.ilog2(DetourCommon.nextPow2(tw * th)), 14);
        return tileBits;
    }

    public int[] getTiles(DemoInputGeomProvider geom, float cellSize, int tileSize) {
        int[] wh = Recast.calcGridSize(geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), cellSize);
        int tw = (wh[0] + tileSize - 1) / tileSize;
        int th = (wh[1] + tileSize - 1) / tileSize;
        return new int[] { tw, th };
    }

    private List<MeshData> buildMeshData(DemoInputGeomProvider m_geom, float m_cellSize, float m_cellHeight,
            float m_agentHeight, float m_agentRadius, float m_agentMaxClimb, RecastBuilderResult[][] rcResult) {

        // Add tiles to nav mesh
        int tw = rcResult.length;
        int th = rcResult[0].length;
        List<MeshData> meshData = new ArrayList<>();
        for (int y = 0; y < th; y++) {
            for (int x = 0; x < tw; x++) {
                NavMeshDataCreateParams params = getNavMeshCreateParams(m_geom, m_cellSize, m_cellHeight, m_agentHeight,
                        m_agentRadius, m_agentMaxClimb, rcResult[x][y]);
                params.tileX = x;
                params.tileY = y;
                MeshData md = NavMeshBuilder.createNavMeshData(params);
                if (md != null) {
                    meshData.add(updateAreaAndFlags(md));
                }
            }
        }
        return meshData;
    }

}
