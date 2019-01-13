package org.recast4j.demo.builder;

import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.geom.DemoOffMeshConnection;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;

public abstract class AbstractNavMeshBuilder {

    protected NavMeshDataCreateParams getNavMeshCreateParams(DemoInputGeomProvider m_geom, float m_cellSize,
            float m_cellHeight, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb,
            RecastBuilderResult rcResult) {
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
        return params;
    }

    protected MeshData updateAreaAndFlags(MeshData meshData) {
        // Update poly flags from areas.
        for (int i = 0; i < meshData.polys.length; ++i) {
            if (meshData.polys[i].getArea() == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WALKABLE) {
                meshData.polys[i].setArea(SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GROUND);
            }
            if (meshData.polys[i].getArea() == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GROUND
                    || meshData.polys[i].getArea() == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GRASS
                    || meshData.polys[i].getArea() == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD) {
                meshData.polys[i].flags = SampleAreaModifications.SAMPLE_POLYFLAGS_WALK;
            } else if (meshData.polys[i].getArea() == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER) {
                meshData.polys[i].flags = SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM;
            } else if (meshData.polys[i].getArea() == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_DOOR) {
                meshData.polys[i].flags = SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR;
            }
        }
        return meshData;
    }
}
