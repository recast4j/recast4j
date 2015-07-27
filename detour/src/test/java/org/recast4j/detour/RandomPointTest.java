/*
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

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

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.recast4j.detour.NavMeshQuery.FRand;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;

public class RandomPointTest {

	private RecastBuilder rcBuilder = new RecastBuilder();
	private float m_cellSize;
	private float m_cellHeight;
	private float m_agentHeight;
	private float m_agentRadius;
	private float m_agentMaxClimb;
	private float m_agentMaxSlope;
	private NavMeshData nmd;
	private NavMeshQuery query;
	private NavMesh navmesh;

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
	}

	@Test
	public void testRandom() {
		Assert.assertEquals(223, nmd.navVerts.length / 3);
		Assert.assertEquals(118, nmd.navPolys.length);
		Assert.assertEquals(453, nmd.header.maxLinkCount);
		Assert.assertEquals(59, nmd.navDVerts.length / 3);
		Assert.assertEquals(289, nmd.navDTris.length / 4);
		Assert.assertEquals(118, nmd.navDMeshes.length);
		Assert.assertEquals(0, nmd.offMeshCons.length);
		Assert.assertEquals(118, nmd.header.offMeshBase);
		Assert.assertEquals(236, nmd.navBvtree.length);
		FRand f = new FRand();
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < 1000; i++) {
			FindRandomPointResult point = query.findRandomPoint(filter, f);
			Assert.assertEquals(Status.SUCCSESS, point.status);
			Tupple2<MeshTile,Poly> tileAndPoly = navmesh.getTileAndPolyByRef(point.randomRef);
			float[] bmin = new float[2];
			float[] bmax = new float[2];
			for (int j = 0; j < tileAndPoly.second.vertCount; j++) {
				int v = tileAndPoly.second.verts[j] * 3;
				bmin[0] = j == 0 ? tileAndPoly.first.verts[v] : Math.min(bmin[0], tileAndPoly.first.verts[v]);
				bmax[0] = j == 0 ? tileAndPoly.first.verts[v] : Math.max(bmax[0], tileAndPoly.first.verts[v]);
				bmin[1] = j == 0 ? tileAndPoly.first.verts[v + 2] : Math.min(bmin[1], tileAndPoly.first.verts[v + 2]);
				bmax[1] = j == 0 ? tileAndPoly.first.verts[v + 2] : Math.max(bmax[1], tileAndPoly.first.verts[v + 2]);
			}
			Assert.assertTrue(point.randomPt[0] >= bmin[0]);
			Assert.assertTrue(point.randomPt[0] <= bmax[0]);
			Assert.assertTrue(point.randomPt[2] >= bmin[1]);
			Assert.assertTrue(point.randomPt[2] <= bmax[1]);
		}
	}

	@Test
	public void testRandomInCircle() {
		FRand f = new FRand();
		QueryFilter filter = new QueryFilter();
		FindRandomPointResult point = query.findRandomPoint(filter, f);
		for (int i = 0; i < 1000; i++) {
			point = query.findRandomPointAroundCircle(point.randomRef, point.randomPt, 5f, filter, f);
			Assert.assertEquals(Status.SUCCSESS, point.status);
			Tupple2<MeshTile, Poly> tileAndPoly = navmesh.getTileAndPolyByRef(point.randomRef);
			float[] bmin = new float[2];
			float[] bmax = new float[2];
			for (int j = 0; j < tileAndPoly.second.vertCount; j++) {
				int v = tileAndPoly.second.verts[j] * 3;
				bmin[0] = j == 0 ? tileAndPoly.first.verts[v] : Math.min(bmin[0], tileAndPoly.first.verts[v]);
				bmax[0] = j == 0 ? tileAndPoly.first.verts[v] : Math.max(bmax[0], tileAndPoly.first.verts[v]);
				bmin[1] = j == 0 ? tileAndPoly.first.verts[v + 2] : Math.min(bmin[1], tileAndPoly.first.verts[v + 2]);
				bmax[1] = j == 0 ? tileAndPoly.first.verts[v + 2] : Math.max(bmax[1], tileAndPoly.first.verts[v + 2]);
			}
			Assert.assertTrue(point.randomPt[0] >= bmin[0]);
			Assert.assertTrue(point.randomPt[0] <= bmax[0]);
			Assert.assertTrue(point.randomPt[2] >= bmin[1]);
			Assert.assertTrue(point.randomPt[2] <= bmax[1]);
		}
	}
}
