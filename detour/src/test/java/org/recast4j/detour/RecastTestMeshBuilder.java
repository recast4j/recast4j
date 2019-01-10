/*
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
package org.recast4j.detour;

import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.geom.InputGeomProvider;

public class RecastTestMeshBuilder {

    private final MeshData meshData;
    private final static float m_cellSize = 0.3f;
    private final static float m_cellHeight = 0.2f;
    private final static float m_agentHeight = 2.0f;
    private final static float m_agentRadius = 0.6f;
    private final static float m_agentMaxClimb = 0.9f;
    private final static float m_agentMaxSlope = 45.0f;
    private final static int m_regionMinSize = 8;
    private final static int m_regionMergeSize = 20;
    private final static float m_edgeMaxLen = 12.0f;
    private final static float m_edgeMaxError = 1.3f;
    private final static int m_vertsPerPoly = 6;
    private final static float m_detailSampleDist = 6.0f;
    private final static float m_detailSampleMaxError = 1.0f;

    public RecastTestMeshBuilder() {
        this(new ObjImporter().load(ObjImporter.class.getResourceAsStream("dungeon.obj")), PartitionType.WATERSHED,
                m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope,
                m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist,
                m_detailSampleMaxError);
    }

    public RecastTestMeshBuilder(InputGeomProvider m_geom, PartitionType m_partitionType, float m_cellSize,
            float m_cellHeight, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb, float m_agentMaxSlope,
            int m_regionMinSize, int m_regionMergeSize, float m_edgeMaxLen, float m_edgeMaxError, int m_vertsPerPoly,
            float m_detailSampleDist, float m_detailSampleMaxError) {
        RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, 0,
                SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
        RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, m_geom.getMeshBoundsMin(), m_geom.getMeshBoundsMax());
        RecastBuilder rcBuilder = new RecastBuilder();
        RecastBuilderResult rcResult = rcBuilder.build(m_geom, bcfg);
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
        params.detailMeshes = m_dmesh.meshes;
        params.detailVerts = m_dmesh.verts;
        params.detailVertsCount = m_dmesh.nverts;
        params.detailTris = m_dmesh.tris;
        params.detailTriCount = m_dmesh.ntris;
        params.walkableHeight = m_agentHeight;
        params.walkableRadius = m_agentRadius;
        params.walkableClimb = m_agentMaxClimb;
        params.bmin = m_pmesh.bmin;
        params.bmax = m_pmesh.bmax;
        params.cs = m_cellSize;
        params.ch = m_cellHeight;
        params.buildBvTree = true;

        params.offMeshConVerts = new float[6];
        params.offMeshConVerts[0] = 0.1f;
        params.offMeshConVerts[1] = 0.2f;
        params.offMeshConVerts[2] = 0.3f;
        params.offMeshConVerts[3] = 0.4f;
        params.offMeshConVerts[4] = 0.5f;
        params.offMeshConVerts[5] = 0.6f;
        params.offMeshConRad = new float[1];
        params.offMeshConRad[0] = 0.1f;
        params.offMeshConDir = new int[1];
        params.offMeshConDir[0] = 1;
        params.offMeshConAreas = new int[1];
        params.offMeshConAreas[0] = 2;
        params.offMeshConFlags = new int[1];
        params.offMeshConFlags[0] = 12;
        params.offMeshConUserID = new int[1];
        params.offMeshConUserID[0] = 0x4567;
        params.offMeshConCount = 1;
        meshData = NavMeshBuilder.createNavMeshData(params);
    }

    public MeshData getMeshData() {
        return meshData;
    }
}
