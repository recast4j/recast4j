package org.recast4j.detour;

import org.junit.Before;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;

public abstract class AbstractDetourTest {

	protected final long[] startRefs = { 281474976710696L, 281474976710773L, 281474976710680L, 281474976710753L,
			281474976710733L };

	protected final long[] endRefs = { 281474976710721L, 281474976710767L, 281474976710758L, 281474976710731L,
			281474976710772L };

	protected final float[][] startPoss = { { 22.60652f, 10.197294f, -45.918674f },
			{ 22.331268f, 10.197294f, -1.0401875f }, { 18.694363f, 15.803535f, -73.090416f },
			{ 0.7453353f, 10.197294f, -5.94005f }, { -20.651257f, 5.904126f, -13.712508f } };

	protected final float[][] endPoss = { { 6.4576626f, 10.197294f, -18.33406f }, { -5.8023443f, 0.19729415f, 3.008419f },
			{ 38.423977f, 10.197294f, -0.116066754f }, { 0.8635526f, 10.197294f, -10.31032f },
			{ 18.784092f, 10.197294f, 3.0543678f } };

	protected RecastBuilder rcBuilder = new RecastBuilder();
	protected float m_cellSize;
	protected float m_cellHeight;
	protected float m_agentHeight;
	protected float m_agentRadius;
	protected float m_agentMaxClimb;
	protected float m_agentMaxSlope;
	protected NavMeshData nmd;
	protected NavMeshQuery query;
	protected NavMesh navmesh;

	@Before
	public void setUp() {
		rcBuilder.build();
		m_cellSize = 0.3f;
		m_cellHeight = 0.2f;
		m_agentHeight = 2.0f;
		m_agentRadius = 0.6f;
		m_agentMaxClimb = 0.9f;
		m_agentMaxSlope = 45.0f;

		PolyMesh m_pmesh = rcBuilder.getMesh();
		for (int i = 0; i < m_pmesh.npolys; ++i) {
			m_pmesh.flags[i] = 1;
		}
		PolyMeshDetail m_dmesh = rcBuilder.getMeshDetail();
		NavMeshCreateParams params = new NavMeshCreateParams();
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
		// params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		// params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		// params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		// params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		// params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		// params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		// params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.bmin = m_pmesh.bmin;
		params.bmax = m_pmesh.bmax;
		params.cs = m_cellSize;
		params.ch = m_cellHeight;
		params.buildBvTree = true;

		nmd = NavMeshBuilder.createNavMeshData(params);
		navmesh = new NavMesh();
		navmesh.init(nmd, 0);
		query = new NavMeshQuery();
		query.init(navmesh, 0);

		for (int i = 0; i < startRefs.length; i++) {
			System.out.println("//---");
			System.out.println("m_startRef = " + startRefs[i] + ";");
			System.out.println("m_endRef = " + endRefs[i] + ";");
			System.out.println("m_spos[0] = " + startPoss[i][0] + ";");
			System.out.println("m_spos[1] = " + startPoss[i][1] + ";");
			System.out.println("m_spos[2] = " + startPoss[i][2] + ";");
			System.out.println("m_epos[0] = " + endPoss[i][0] + ";");
			System.out.println("m_epos[1] = " + endPoss[i][1] + ";");
			System.out.println("m_epos[2] = " + endPoss[i][2] + ";");
		}

	}

}
