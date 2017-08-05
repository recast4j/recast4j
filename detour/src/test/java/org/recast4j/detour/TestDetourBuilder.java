package org.recast4j.detour;

import org.recast4j.recast.InputGeom;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;

public class TestDetourBuilder extends DetourBuilder {

	public MeshData build(InputGeom geom, RecastBuilderConfig rcConfig, float agentHeight, float agentRadius,
			float agentMaxClimb, int x, int y, boolean applyRecastDemoFlags) {
		RecastBuilder rcBuilder = new RecastBuilder();
		RecastBuilderResult rcResult = rcBuilder.build(geom, rcConfig);
		PolyMesh pmesh = rcResult.getMesh();

		if (applyRecastDemoFlags) {
			// Update poly flags from areas.
			for (int i = 0; i < pmesh.npolys; ++i) {
				if (pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GROUND
						|| pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GRASS
						|| pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD) {
					pmesh.flags[i] = SampleAreaModifications.SAMPLE_POLYFLAGS_WALK;
				} else if (pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER) {
					pmesh.flags[i] = SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM;
				} else if (pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_DOOR) {
					pmesh.flags[i] = SampleAreaModifications.SAMPLE_POLYFLAGS_WALK
							| SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR;
				}
				if (pmesh.areas[i] > 0) {
					pmesh.areas[i]--;
				}
			}
		}
		PolyMeshDetail dmesh = rcResult.getMeshDetail();
		NavMeshDataCreateParams params = getNavMeshCreateParams(rcConfig.cfg, pmesh, dmesh, agentHeight, agentRadius,
				agentMaxClimb);
		return build(params, x, y);
	}

	public NavMeshDataCreateParams getNavMeshCreateParams(RecastConfig rcConfig, PolyMesh pmesh, PolyMeshDetail dmesh,
			float agentHeight, float agentRadius, float agentMaxClimb) {
		NavMeshDataCreateParams params = new NavMeshDataCreateParams();
		params.verts = pmesh.verts;
		params.vertCount = pmesh.nverts;
		params.polys = pmesh.polys;
		params.polyAreas = pmesh.areas;
		params.polyFlags = pmesh.flags;
		params.polyCount = pmesh.npolys;
		params.nvp = pmesh.nvp;
		if (dmesh != null) {
			params.detailMeshes = dmesh.meshes;
			params.detailVerts = dmesh.verts;
			params.detailVertsCount = dmesh.nverts;
			params.detailTris = dmesh.tris;
			params.detailTriCount = dmesh.ntris;
		}
		params.walkableHeight = agentHeight;
		params.walkableRadius = agentRadius;
		params.walkableClimb = agentMaxClimb;
		params.bmin = pmesh.bmin;
		params.bmax = pmesh.bmax;
		params.cs = rcConfig.cs;
		params.ch = rcConfig.ch;
		params.buildBvTree = true;
		/*
		 * params.offMeshConVerts = m_geom->getOffMeshConnectionVerts(); params.offMeshConRad =
		 * m_geom->getOffMeshConnectionRads(); params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		 * params.offMeshConAreas = m_geom->getOffMeshConnectionAreas(); params.offMeshConFlags =
		 * m_geom->getOffMeshConnectionFlags(); params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		 * params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		 */
		return params;

	}
}
