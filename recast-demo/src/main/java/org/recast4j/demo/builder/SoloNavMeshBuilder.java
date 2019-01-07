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

import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.geom.DemoOffMeshConnection;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.Tupple2;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;

public class SoloNavMeshBuilder {

    public Tupple2<RecastBuilderResult, MeshData> build(DemoInputGeomProvider m_geom, PartitionType m_partitionType, float m_cellSize,
            float m_cellHeight, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb, float m_agentMaxSlope, int m_regionMinSize,
            int m_regionMergeSize, float m_edgeMaxLen, float m_edgeMaxError, int m_vertsPerPoly, float m_detailSampleDist,
            float m_detailSampleMaxError, boolean filterLowHangingObstacles, boolean filterLedgeSpans,
            boolean filterWalkableLowHeightSpans) {

        RecastBuilderResult rcResult = buildRecastResult(m_geom, m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly,
                m_detailSampleDist, m_detailSampleMaxError, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans);
        return new Tupple2<>(rcResult,
                buildMeshData(m_geom, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, rcResult));
    }

    private RecastBuilderResult buildRecastResult(DemoInputGeomProvider m_geom, PartitionType m_partitionType, float m_cellSize,
            float m_cellHeight, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb, float m_agentMaxSlope, int m_regionMinSize,
            int m_regionMergeSize, float m_edgeMaxLen, float m_edgeMaxError, int m_vertsPerPoly, float m_detailSampleDist,
            float m_detailSampleMaxError, boolean filterLowHangingObstacles, boolean filterLedgeSpans,
            boolean filterWalkableLowHeightSpans) {
        RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb,
                m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist,
                m_detailSampleMaxError, 0, SampleAreaModifications.SAMPLE_AREAMOD_WALKABLE, filterLowHangingObstacles, filterLedgeSpans,
                filterWalkableLowHeightSpans);
        RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, m_geom.getMeshBoundsMin(), m_geom.getMeshBoundsMax());
        RecastBuilder rcBuilder = new RecastBuilder();
        return rcBuilder.build(m_geom, bcfg);
    }

    private MeshData buildMeshData(DemoInputGeomProvider m_geom, float m_cellSize, float m_cellHeight, float m_agentHeight,
            float m_agentRadius, float m_agentMaxClimb, RecastBuilderResult rcResult) {
        PolyMesh m_pmesh = rcResult.getMesh();
        for (int i = 0; i < m_pmesh.npolys; ++i) {
            m_pmesh.flags[i] = 1;
        }
        PolyMeshDetail m_dmesh = rcResult.getMeshDetail();
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        params.verts = m_pmesh.verts;
        params.vertCount = m_pmesh.nverts;
        params.polys = m_pmesh.polys;
        params.polyAreas = m_pmesh.areas;
        params.polyFlags = m_pmesh.flags;
        params.polyCount = m_pmesh.npolys;
        params.nvp = m_pmesh.nvp;
        if (m_dmesh != null) {
            params.detailMeshes = m_dmesh.meshes;
            params.detailVerts = m_dmesh.verts;
            params.detailVertsCount = m_dmesh.nverts;
            params.detailTris = m_dmesh.tris;
            params.detailTriCount = m_dmesh.ntris;
        }
        params.walkableHeight = m_agentHeight;
        params.walkableRadius = m_agentRadius;
        params.walkableClimb = m_agentMaxClimb;
        params.bmin = m_pmesh.bmin;
        params.bmax = m_pmesh.bmax;
        params.cs = m_cellSize;
        params.ch = m_cellHeight;
        params.buildBvTree = true;

        params.offMeshConCount = m_geom.getOffMeshConnections().size();
        params.offMeshConVerts = new float[params.offMeshConCount * 6];
        params.offMeshConRad = new float[params.offMeshConCount];
        params.offMeshConDir = new int[params.offMeshConCount];
        params.offMeshConAreas = new int[params.offMeshConCount];
        params.offMeshConFlags = new int[params.offMeshConCount];
        params.offMeshConUserID = new int[params.offMeshConCount];
        for (int i = 0; i < params.offMeshConCount; i++) {
            DemoOffMeshConnection offMeshCon = m_geom.getOffMeshConnections().get(i);
            for (int j = 0; j < 6; j++) {
                params.offMeshConVerts[6 * i + j] = offMeshCon.verts[j];
            }
            params.offMeshConRad[i] = offMeshCon.radius;
            params.offMeshConDir[i] = offMeshCon.bidir ? 1 : 0;
            params.offMeshConAreas[i] = offMeshCon.area;
            params.offMeshConFlags[i] = offMeshCon.flags;
        }
        return NavMeshBuilder.createNavMeshData(params);
    }

}
