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

import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.recast4j.detour.NavMeshQuery.StraightPathItem;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;

public class FindPathTest {

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

	long[] startRefs = new long[] { 281474976710696L, 281474976710773L, 281474976710680L, 281474976710753L,
			281474976710733L };

	long[] endRefs = new long[] { 281474976710721L, 281474976710767L, 281474976710758L, 281474976710731L,
			281474976710772L };

	float[][] startPoss = new float[][] { new float[] { 22.60652f, 10.197294f, -45.918674f },
		{ 22.331268f, 10.197294f, -1.0401875f }, { 18.694363f, 15.803535f, -73.090416f },
		{ 0.7453353f, 10.197294f, -5.94005f }, {-20.651257f, 5.904126f, -13.712508f} };

	float[][] endPoss = new float[][] { { 6.4576626f, 10.197294f, -18.33406f },
		{ -5.8023443f, 0.19729415f, 3.008419f }, { 38.423977f, 10.197294f, -0.116066754f },
		{ 0.8635526f, 10.197294f, -10.31032f }, {18.784092f, 10.197294f, 3.0543678f} };


	Status[] statuses = new Status[] { Status.SUCCSESS, Status.PARTIAL_RESULT, Status.SUCCSESS, Status.SUCCSESS, Status.SUCCSESS };
	long[][] results = new long[][] {
		{ 281474976710696L, 281474976710695L, 281474976710694L, 281474976710703L, 281474976710706L,
				281474976710705L, 281474976710702L, 281474976710701L, 281474976710714L, 281474976710713L,
				281474976710712L, 281474976710727L, 281474976710730L, 281474976710717L, 281474976710721L },
		{ 281474976710773L, 281474976710772L, 281474976710768L, 281474976710754L, 281474976710755L,
				281474976710753L, 281474976710748L, 281474976710752L, 281474976710731L, 281474976710729L,
				281474976710717L, 281474976710724L, 281474976710728L, 281474976710737L, 281474976710738L,
				281474976710736L, 281474976710733L, 281474976710735L, 281474976710742L, 281474976710740L,
				281474976710746L, 281474976710745L, 281474976710744L },
		{281474976710680L, 281474976710684L, 281474976710688L, 281474976710687L, 281474976710686L, 281474976710697L, 281474976710695L, 281474976710694L, 281474976710703L, 281474976710706L, 281474976710705L, 281474976710702L, 281474976710701L, 281474976710714L, 281474976710713L, 281474976710712L, 281474976710727L, 281474976710730L, 281474976710717L, 281474976710729L, 281474976710731L, 281474976710752L, 281474976710748L, 281474976710753L, 281474976710755L, 281474976710754L, 281474976710768L, 281474976710772L, 281474976710773L, 281474976710770L, 281474976710757L, 281474976710761L, 281474976710758L},
		{281474976710753L, 281474976710748L, 281474976710752L, 281474976710731L}, 
		{281474976710733L, 281474976710736L, 281474976710738L, 281474976710737L, 281474976710728L, 281474976710724L, 281474976710717L, 281474976710729L, 281474976710731L, 281474976710752L, 281474976710748L, 281474976710753L, 281474976710755L, 281474976710754L, 281474976710768L, 281474976710772L}
		};
	
	StraightPathItem[][] straightPaths = new StraightPathItem[][] {
		{new StraightPathItem(new float[]{0, 0, 0}, 0, 1L), new StraightPathItem(new float[]{0, 0, 0}, 0, 1L), new StraightPathItem(new float[]{0, 0, 0}, 0, 1L),new StraightPathItem(new float[]{0, 0, 0}, 0, 1L),new StraightPathItem(new float[]{0, 0, 0}, 0, 1L),new StraightPathItem(new float[]{0, 0, 0}, 0, 1L)},
		{}
	};
		
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

	@Test
	public void testFindPath() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			long startRef = startRefs[i];
			long endRef = endRefs[i];
			float[] startPos = startPoss[i];
			float[] endPos = endPoss[i];
			Tupple2<Status, List<Long>> path = query.findPath(startRef, endRef, startPos, endPos, filter);
			Assert.assertEquals(statuses[i], path.first);
			Assert.assertEquals(results[i].length, path.second.size());
			for (int j = 0; j < results[i].length; j++) {
				Assert.assertEquals(results[i][j], path.second.get(j).longValue());
			}
		}
	}

	@Test
	public void testFindPathSliced() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			long startRef = startRefs[i];
			long endRef = endRefs[i];
			float[] startPos = startPoss[i];
			float[] endPos = endPoss[i];
			query.initSlicedFindPath(startRef, endRef, startPos, endPos, filter, NavMeshQuery.DT_FINDPATH_ANY_ANGLE);
			Status status = Status.IN_PROGRESS;
			while (status == Status.IN_PROGRESS) { 
				Tupple2<Status, Integer> res = query.updateSlicedFindPath(10);
				status = res.first;
			}
			Tupple2<Status, List<Long>> path = query.finalizeSlicedFindPath();
			Assert.assertEquals(statuses[i], path.first);
			Assert.assertEquals(results[i].length, path.second.size());
			for (int j = 0; j < results[i].length; j++) {
				Assert.assertEquals(results[i][j], path.second.get(j).longValue());
			}

		}
	}

	@Test
	public void testFindPathStraight() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			long startRef = startRefs[i];
			long endRef = endRefs[i];
			float[] startPos = startPoss[i];
			float[] endPos = endPoss[i];
			Tupple2<Status, List<Long>> path = query.findPath(startRef, endRef, startPos, endPos, filter);
			List<StraightPathItem> straightPath = query.findStraightPath(startPos, endPos, path.second, 0);
			System.out.println(straightPath.size());
		}
	}

}
