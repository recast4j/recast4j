package org.recast4j.detour;

import org.recast4j.recast.InputGeom;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastConfig;

public class DetourBuilder {

	private final InputGeom geom;
	
	public DetourBuilder(InputGeom geom) {
		this.geom = geom;

	}

	public MeshData build(RecastConfig config, float agentHeight, float agentRadius, float agentMaxClimb, int x, int y) {
		RecastBuilder rcBuilder = new RecastBuilder(geom);
		rcBuilder.build(config);
		PolyMesh pmesh = rcBuilder.getMesh();
		PolyMeshDetail dmesh = rcBuilder.getMeshDetail();
		MeshData data = null;
		if (pmesh != null && dmesh != null) {
			NavMeshCreateParams params = new NavMeshCreateParams();
			params.verts = pmesh.verts;
			params.vertCount = pmesh.nverts;
			params.polys = pmesh.polys;
			params.polyAreas = pmesh.areas;
			params.polyFlags = pmesh.flags;
			params.polyCount = pmesh.npolys;
			params.nvp = pmesh.nvp;
			params.detailMeshes = dmesh.meshes;
			params.detailVerts = dmesh.verts;
			params.detailVertsCount = dmesh.nverts;
			params.detailTris = dmesh.tris;
			params.detailTriCount = dmesh.ntris;
			params.walkableHeight = agentHeight;
			params.walkableRadius = agentRadius;
			params.walkableClimb = agentMaxClimb;
			params.bmin = pmesh.bmin;
			params.bmax = pmesh.bmax;
			params.cs = config.cs;
			params.ch = config.ch;
			params.buildBvTree = true;
			/*
			params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
			params.offMeshConRad = m_geom->getOffMeshConnectionRads();
			params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
			params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
			params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
			params.offMeshConUserID = m_geom->getOffMeshConnectionId();
			params.offMeshConCount = m_geom->getOffMeshConnectionCount();	
			*/	
			data = NavMeshBuilder.createNavMeshData(params);
			data.header.x = x;
			data.header.y = y;
		}
		return data;
	}
}
