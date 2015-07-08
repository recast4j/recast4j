/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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

import static org.recast4j.detour.DetourCommon.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.recast4j.detour.DetourCommon.IntersectResult;

public class NavMeshQuery {

	public static final int DT_FINDPATH_LOW_QUALITY_FAR = 0x01; /// < [provisional] trade quality for performance far
																/// from the origin. The idea is that by then a new
																/// query will be issued
	public static final int DT_FINDPATH_ANY_ANGLE = 0x02; /// < use raycasts during pathfind to "shortcut" (raycast
															/// still consider costs)

	public static final int DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01;	///< Add a vertex at every polygon edge crossing where area changes.
	public static final int DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02;	///< Add a vertex at every polygon edge crossing.

	public static final int DT_RAYCAST_USE_COSTS = 0x01;		///< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
	
	static float H_SCALE = 0.999f; // Search heuristic scale.
	static int MAX_NEIS = 32;

	private NavMesh m_nav;
	private NodePool m_nodePool;
	private NodePool m_tinyNodePool;
	private QueryData m_query; /// < Sliced query state.
	private NodeQueue m_openList;

	public NavMeshQuery() {
		m_nodePool = new NodePool();
		m_tinyNodePool = new NodePool();
		m_openList = new NodeQueue();
	}

	public void init(NavMesh nav, int maxNodes) {
		m_nav = nav;
		m_nodePool.clear();
		m_tinyNodePool.clear();
		m_openList.clear();
	}

	public static class FRand {
		Random r = new Random();

		public float frand() {
			return r.nextFloat();
		}
	}

	public class RandomPointResult {
		public final Status status;
		public final long randomRef;
		public final float[] randomPt;

		public RandomPointResult(Status status, long randomRef, float[] randomPt) {
			this.status = status;
			this.randomRef = randomRef;
			this.randomPt = randomPt;
		}
	}

	public RandomPointResult findRandomPoint(QueryFilter filter, FRand frand) {
		// Randomly pick one tile. Assume that all tiles cover roughly the same area.
		MeshTile tile = null;
		float tsum = 0.0f;
		for (int i = 0; i < m_nav.getMaxTiles(); i++) {
			MeshTile t = m_nav.getTile(i);
			if (t == null || t.header == null)
				continue;

			// Choose random tile using reservoi sampling.
			float area = 1.0f; // Could be tile area too.
			tsum += area;
			float u = frand.frand();
			if (u * tsum <= area)
				tile = t;
		}
		if (tile == null)
			return new RandomPointResult(Status.FAILURE, 0, null);

		// Randomly pick one polygon weighted by polygon area.
		Poly poly = null;
		long polyRef = 0;
		long base = m_nav.getPolyRefBase(tile);

		float areaSum = 0.0f;
		for (int i = 0; i < tile.header.polyCount; ++i) {
			Poly p = tile.polys[i];
			// Do not return off-mesh connection polygons.
			if (p.getType() != Poly.DT_POLYTYPE_GROUND)
				continue;
			// Must pass filter
			long ref = base | i;
			if (!filter.passFilter(ref, tile, p))
				continue;

			// Calc area of the polygon.
			float polyArea = 0.0f;
			for (int j = 2; j < p.vertCount; ++j) {
				int va = p.verts[0] * 3;
				int vb = p.verts[j - 1] * 3;
				int vc = p.verts[j] * 3;
				polyArea += triArea2D(tile.verts, va, vb, vc);
			}

			// Choose random polygon weighted by area, using reservoi sampling.
			areaSum += polyArea;
			float u = frand.frand();
			if (u * areaSum <= polyArea) {
				poly = p;
				polyRef = ref;
			}
		}

		if (poly == null)
			return new RandomPointResult(Status.FAILURE, 0, null);

		// Randomly pick point on polygon.
		float[] verts = new float[3 * NavMesh.DT_VERTS_PER_POLYGON];
		float[] areas = new float[NavMesh.DT_VERTS_PER_POLYGON];
		System.arraycopy(tile.verts, poly.verts[0] * 3, verts, 0, 3);
		for (int j = 1; j < poly.vertCount; ++j) {
			System.arraycopy(tile.verts, poly.verts[j] * 3, verts, j * 3, 3);
		}

		float s = frand.frand();
		float t = frand.frand();

		float[] pt = randomPointInConvexPoly(verts, poly.vertCount, areas, s, t);

		Tupple2<Status, Float> height = getPolyHeight(polyRef, new VectorPtr(pt, 0));
		if (height.first.isFailed())
			return new RandomPointResult(height.first, 0, null);
		pt[1] = height.second;

		return new RandomPointResult(Status.SUCCSESS, polyRef, pt);
	}

	public RandomPointResult findRandomPointAroundCircle(long startRef, float[] centerPos, float maxRadius,
			QueryFilter filter, FRand frand) {

		// Validate input
		if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			throw new IllegalArgumentException("Invalid start ref");

		Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(startRef);
		MeshTile startTile = tileAndPoly.first;
		Poly startPoly = tileAndPoly.second;
		if (!filter.passFilter(startRef, startTile, startPoly))
			throw new IllegalArgumentException("Invalid start");

		m_nodePool.clear();
		m_openList.clear();

		Node startNode = m_nodePool.getNode(startRef);
		vCopy(startNode.pos, centerPos);
		startNode.pidx = 0;
		startNode.cost = 0;
		startNode.total = 0;
		startNode.id = startRef;
		startNode.flags = Node.DT_NODE_OPEN;
		m_openList.push(startNode);

		float radiusSqr = maxRadius * maxRadius;
		float areaSum = 0.0f;

		MeshTile randomTile = null;
		Poly randomPoly = null;
		long randomPolyRef = 0;

		while (!m_openList.isEmpty()) {
			Node bestNode = m_openList.pop();
			bestNode.flags &= ~Node.DT_NODE_OPEN;
			bestNode.flags |= Node.DT_NODE_CLOSED;
			// Get poly and tile.
			// The API input has been cheked already, skip checking internal data.
			long bestRef = bestNode.id;
			Tupple2<MeshTile, Poly> bestTilePoly = m_nav.getTileAndPolyByRefUnsafe(bestRef);
			MeshTile bestTile = bestTilePoly.first;
			Poly bestPoly = bestTilePoly.second;

			// Place random locations on on ground.
			if (bestPoly.getType() == Poly.DT_POLYTYPE_GROUND) {
				// Calc area of the polygon.
				float polyArea = 0.0f;
				for (int j = 2; j < bestPoly.vertCount; ++j) {
					int va = bestPoly.verts[0] * 3;
					int vb = bestPoly.verts[j - 1] * 3;
					int vc = bestPoly.verts[j] * 3;
					polyArea += triArea2D(bestTile.verts, va, vb, vc);
				}
				// Choose random polygon weighted by area, using reservoi sampling.
				areaSum += polyArea;
				float u = frand.frand();
				if (u * areaSum <= polyArea) {
					randomTile = bestTile;
					randomPoly = bestPoly;
					randomPolyRef = bestRef;
				}
			}

			// Get parent poly and tile.
			long parentRef = 0;
			if (bestNode.pidx != 0)
				parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
			if (parentRef != 0) {
				Tupple2<MeshTile, Poly> parentTilePoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
				MeshTile parentTile = parentTilePoly.first;
				Poly parentPoly = parentTilePoly.second;
			}

			for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
				Link link = bestTile.links.get(i);
				long neighbourRef = link.ref;
				// Skip invalid neighbours and do not follow back to parent.
				if (neighbourRef == 0 || neighbourRef == parentRef)
					continue;

				// Expand to neighbour
				Tupple2<MeshTile, Poly> neighbourTilePoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
				MeshTile neighbourTile = neighbourTilePoly.first;
				Poly neighbourPoly = neighbourTilePoly.second;

				// Do not advance if the polygon is excluded by the filter.
				if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;

				// Find edge and calc distance to the edge.
				PortalResult portalpoints = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
						neighbourTile, 0, 0);
				if (portalpoints.status.isFailed())
					continue;
				float[] va = portalpoints.left;
				float[] vb = portalpoints.right;

				// If the circle is not touching the next polygon, skip it.
				float[] distseg = distancePtSegSqr2D(centerPos, va, vb);
				float distSqr = distseg[0];
				float tseg = distseg[1];
				if (distSqr > radiusSqr)
					continue;

				Node neighbourNode = m_nodePool.getNode(neighbourRef);

				if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
					continue;

				// Cost
				if (neighbourNode.flags == 0)
					neighbourNode.pos = vLerp(va, vb, 0.5f);

				float total = bestNode.total + vDist(bestNode.pos, neighbourNode.pos);

				// The node is already in open list and the new result is worse, skip.
				if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					continue;

				neighbourNode.id = neighbourRef;
				neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
				neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
				neighbourNode.total = total;

				if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0) {
					m_openList.modify(neighbourNode);
				} else {
					neighbourNode.flags = Node.DT_NODE_OPEN;
					m_openList.push(neighbourNode);
				}
			}
		}

		if (randomPoly == null)
			return new RandomPointResult(Status.FAILURE, 0, null);

		// Randomly pick point on polygon.
		float[] verts = new float[3 * NavMesh.DT_VERTS_PER_POLYGON];
		float[] areas = new float[NavMesh.DT_VERTS_PER_POLYGON];
		System.arraycopy(randomTile.verts, randomPoly.verts[0] * 3, verts, 0, 3);
		for (int j = 1; j < randomPoly.vertCount; ++j) {
			System.arraycopy(randomTile.verts, randomPoly.verts[j] * 3, verts, j * 3, 3);
		}

		float s = frand.frand();
		float t = frand.frand();

		float[] pt = randomPointInConvexPoly(verts, randomPoly.vertCount, areas, s, t);

		Tupple2<Status, Float> height = getPolyHeight(randomPolyRef, new VectorPtr(pt, 0));
		if (height.first.isFailed())
			return new RandomPointResult(height.first, 0, null);
		pt[1] = height.second;

		return new RandomPointResult(Status.SUCCSESS, randomPolyRef, pt);
	}

	//////////////////////////////////////////////////////////////////////////////////////////
	/// @par
	///
	/// Uses the detail polygons to find the surface height. (Most accurate.)
	///
	/// @p pos does not have to be within the bounds of the polygon or navigation mesh.
	///
	/// See closestPointOnPolyBoundary() for a limited but faster option.
	///
	Tupple3<Status, Boolean, float[]> closestPointOnPoly(long ref, float[] pos) {
		Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
		MeshTile tile = tileAndPoly.first;
		Poly poly = tileAndPoly.second;

		// Off-mesh connections don't have detail polygons.
		if (poly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
			int v0 = poly.verts[0] * 3;
			int v1 = poly.verts[1] * 3;
			float d0 = vDist(pos, tile.verts, v0);
			float d1 = vDist(pos, tile.verts, v1);
			float u = d0 / (d0 + d1);
			float[] closest = vLerp(tile.verts, v0, v1, u);
			return new Tupple3<>(Status.SUCCSESS, false, closest);
		}

		int ip = poly.index;
		PolyDetail pd = tile.detailMeshes[ip];

		// Clamp point to be inside the polygon.
		float[] verts = new float[NavMesh.DT_VERTS_PER_POLYGON * 3];
		float[] edged = new float[NavMesh.DT_VERTS_PER_POLYGON];
		float[] edget = new float[NavMesh.DT_VERTS_PER_POLYGON];
		int nv = poly.vertCount;
		for (int i = 0; i < nv; ++i)
			System.arraycopy(tile.verts, poly.verts[i] * 3, verts, i * 3, 3);

		boolean posOverPoly = false;
		float[] closest = new float[3];
		vCopy(closest, pos);
		if (!dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget)) {
			// Point is outside the polygon, dtClamp to nearest edge.
			float dmin = Float.MAX_VALUE;
			int imin = -1;
			for (int i = 0; i < nv; ++i) {
				if (edged[i] < dmin) {
					dmin = edged[i];
					imin = i;
				}
			}
			int va = imin * 3;
			int vb = ((imin + 1) % nv) * 3;
			closest = vLerp(verts, va, vb, edget[imin]);
			posOverPoly = false;
		} else {
			posOverPoly = true;
		}
		VectorPtr posV = new VectorPtr(pos);
		// Find height at the location.
		for (int j = 0; j < pd.triCount; ++j) {
			int t = (pd.triBase + j) * 4;
			VectorPtr[] v = new VectorPtr[3];
			for (int k = 0; k < 3; ++k) {
				if (tile.detailTris[t + k] < poly.vertCount)
					v[k] = new VectorPtr(tile.verts, poly.verts[tile.detailTris[t + k]] * 3);
				else
					v[k] = new VectorPtr(tile.detailVerts,
							(pd.vertBase + (tile.detailTris[t + k] - poly.vertCount)) * 3);
			}
			Tupple2<Boolean, Float> clp = closestHeightPointTriangle(posV, v[0], v[1], v[2]);
			if (clp.first) {
				closest[1] = clp.second;
				break;
			}
		}
		return new Tupple3<Status, Boolean, float[]>(Status.SUCCSESS, posOverPoly, closest);
	}

	/// @par
	///
	/// Much faster than closestPointOnPoly().
	///
	/// If the provided position lies within the polygon's xz-bounds (above or below),
	/// then @p pos and @p closest will be equal.
	///
	/// The height of @p closest will be the polygon boundary. The height detail is not used.
	///
	/// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
	///
	Tupple2<Status, float[]> closestPointOnPolyBoundary(long ref, float[] pos) {

		Tupple2<MeshTile,Poly> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
		MeshTile tile = tileAndPoly.first;
		Poly poly = tileAndPoly.second;

		// Collect vertices.
		float[] verts = new float[NavMesh.DT_VERTS_PER_POLYGON * 3];
		float[] edged = new float[NavMesh.DT_VERTS_PER_POLYGON];
		float[] edget = new float[NavMesh.DT_VERTS_PER_POLYGON];
		int nv = poly.vertCount;
		for (int i = 0; i < nv; ++i)
			System.arraycopy(tile.verts, poly.verts[i] * 3, verts, i * 3, 3);

		float[] closest = new float[3];
		if (dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget)) {
			vCopy(closest, pos);
		} else {
			// Point is outside the polygon, dtClamp to nearest edge.
			float dmin = Float.MAX_VALUE;
			int imin = -1;
			for (int i = 0; i < nv; ++i) {
				if (edged[i] < dmin) {
					dmin = edged[i];
					imin = i;
				}
			}
			int va = imin * 3;
			int vb = ((imin + 1) % nv) * 3;
			closest = vLerp(verts, va, vb, edget[imin]);
		}

		return new Tupple2<>(Status.SUCCSESS, closest);
	}

	/// @par
	///
	/// Will return #DT_FAILURE if the provided position is outside the xz-bounds
	/// of the polygon.
	///
	Tupple2<Status, Float> getPolyHeight(long ref, VectorPtr pos) {
		Tupple2<MeshTile,Poly> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
		MeshTile tile = tileAndPoly.first;
		Poly poly = tileAndPoly.second;
		if (poly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
			VectorPtr v0 = new VectorPtr(tile.verts, poly.verts[0] * 3);
			VectorPtr v1 = new VectorPtr(tile.verts, poly.verts[1] * 3);
			float d0 = vDist2D(pos, v0);
			float d1 = vDist2D(pos, v1);
			float u = d0 / (d0 + d1);
			return new Tupple2<>(Status.SUCCSESS, v0.get(1) + (v1.get(1) - v0.get(1)) * u);
		} else {
			int ip = poly.index;
			PolyDetail pd = tile.detailMeshes[ip];
			for (int j = 0; j < pd.triCount; ++j) {
				int t = (pd.triBase + j) * 4;
				VectorPtr[] v = new VectorPtr[3];
				for (int k = 0; k < 3; ++k) {
					if (tile.detailTris[t + k] < poly.vertCount)
						v[k] = new VectorPtr(tile.verts, poly.verts[tile.detailTris[t + k]] * 3);
					else
						v[k] = new VectorPtr(tile.detailVerts,
								(pd.vertBase + (tile.detailTris[t + k] - poly.vertCount)) * 3);
				}
				Tupple2<Boolean, Float> heightResult = closestHeightPointTriangle(pos, v[0], v[1], v[2]);
				if (heightResult.first) {
					return new Tupple2<>(Status.SUCCSESS, heightResult.second);
				}
			}
		}
		throw new IllegalArgumentException("Invalid ref");
	}

	/// @par
	///
	/// @note If the search box does not intersect any polygons the search will
	/// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check
	/// @p nearestRef before using @p nearestPt.
	///
	/// @warning This function is not suitable for large area searches. If the search
	/// extents overlaps more than MAX_SEARCH (128) polygons it may return an invalid result.
	///
	public Tupple3<Status, Long, float[]> findNearestPoly(float[] center, float[] extents, QueryFilter filter) {

		float[] nearestPt = null;

		// Get nearby polygons from proximity grid.
		int MAX_SEARCH = 128;
		List<Long> polys = queryPolygons(center, extents, filter, MAX_SEARCH);

		// Find nearest polygon amongst the nearby polygons.
		long nearest = 0;
		float nearestDistanceSqr = Float.MAX_VALUE;
		for (int i = 0; i < polys.size(); ++i) {
			long ref = polys.get(i);
			Tupple3<Status, Boolean, float[]> closest = closestPointOnPoly(ref, center);
			float[] closestPtPoly = closest.third;
			boolean posOverPoly = closest.second;

			// If a point is directly over a polygon and closer than
			// climb height, favor that instead of straight line nearest point.
			float d = 0;
			float[] diff = vSub(center, closestPtPoly);
			if (posOverPoly) {
				Tupple2<MeshTile, Poly> tilaAndPoly = m_nav.getTileAndPolyByRefUnsafe(polys.get(i));
				MeshTile tile = tilaAndPoly.first;
				d = Math.abs(diff[1]) - tile.header.walkableClimb;
				d = d > 0 ? d * d : 0;
			} else {
				d = vLenSqr(diff);
			}

			if (d < nearestDistanceSqr) {
				nearestPt = closestPtPoly;
				nearestDistanceSqr = d;
				nearest = ref;
			}
		}

		return new Tupple3<Status, Long, float[]>(Status.SUCCSESS, nearest, nearestPt);
	}

	// FIXME: duplicate
	List<Long> queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax, QueryFilter filter, int maxPolys) {
		List<Long> polys = new ArrayList<>();
		if (tile.bvTree != null) {
			int nodeIndex = 0;
			float[] tbmin = tile.header.bmin;
			float[] tbmax = tile.header.bmax;
			float qfac = tile.header.bvQuantFactor;
			// Calculate quantized box
			int[] bmin = new int[3];
			int[] bmax = new int[3];
			// dtClamp query box to world box.
			float minx = clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
			float miny = clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
			float minz = clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
			float maxx = clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
			float maxy = clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
			float maxz = clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
			// Quantize
			bmin[0] = (int) (qfac * minx) & 0xfffe;
			bmin[1] = (int) (qfac * miny) & 0xfffe;
			bmin[2] = (int) (qfac * minz) & 0xfffe;
			bmax[0] = (int) (qfac * maxx + 1) | 1;
			bmax[1] = (int) (qfac * maxy + 1) | 1;
			bmax[2] = (int) (qfac * maxz + 1) | 1;

			// Traverse tree
			long base = m_nav.getPolyRefBase(tile);
			int end = tile.header.bvNodeCount;
			while (nodeIndex < end) {
				BVNode node = tile.bvTree[nodeIndex];
				boolean overlap = dtOverlapQuantBounds(bmin, bmax, node.bmin, node.bmax);
				boolean isLeafNode = node.i >= 0;

				if (isLeafNode && overlap) {
					long ref = base | node.i;
					if (filter.passFilter(ref, tile, tile.polys[node.i])) {
						if (polys.size() < maxPolys)
							polys.add(ref);
					}
				}

				if (overlap || isLeafNode)
					nodeIndex++;
				else {
					int escapeIndex = -node.i;
					nodeIndex += escapeIndex;
				}
			}
			return polys;
		} else {
			float[] bmin = new float[3];
			float[] bmax = new float[3];
			long base = m_nav.getPolyRefBase(tile);
			for (int i = 0; i < tile.header.polyCount; ++i) {
				Poly p = tile.polys[i];
				// Do not return off-mesh connection polygons.
				if (p.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;
				long ref = base | i;
				if (!filter.passFilter(ref, tile, p))
					continue;
				// Calc polygon bounds.
				int v = p.verts[0] * 3;
				vCopy(bmin, tile.verts, v);
				vCopy(bmax, tile.verts, v);
				for (int j = 1; j < p.vertCount; ++j) {
					v = p.verts[j] * 3;
					vMin(bmin, tile.verts, v);
					vMax(bmax, tile.verts, v);
				}
				if (dtOverlapBounds(qmin, qmax, bmin, bmax)) {
					if (polys.size() < maxPolys)
						polys.add(ref);
				}
			}
			return polys;
		}
	}

	/// @par
	///
	/// If no polygons are found, the function will return #DT_SUCCESS with a
	/// @p polyCount of zero.
	///
	/// If @p polys is too small to hold the entire result set, then the array will
	/// be filled to capacity. The method of choosing which polygons from the
	/// full set are included in the partial result set is undefined.
	///
	List<Long> queryPolygons(float[] center, float[] extents, QueryFilter filter, int maxPolys) {
		float[] bmin = vSub(center, extents);
		float[] bmax = vAdd(center, extents);

		// Find tiles the query touches.
		int[] minxy = m_nav.calcTileLoc(bmin);
		int minx = minxy[0];
		int miny = minxy[1];
		int[] maxxy = m_nav.calcTileLoc(bmax);
		int maxx = maxxy[0];
		int maxy = maxxy[1];
		List<Long> polys = new ArrayList<>();
		for (int y = miny; y <= maxy; ++y) {
			for (int x = minx; x <= maxx; ++x) {
				List<MeshTile> neis = m_nav.getTilesAt(x, y);
				for (int j = 0; j < neis.size(); ++j) {
					List<Long> polysInTile = queryPolygonsInTile(neis.get(j), bmin, bmax, filter,
							maxPolys - polys.size());
					polys.addAll(polysInTile);
					if (polys.size() >= maxPolys) {
						return polys;
					}
				}
			}
		}

		return polys;
	}

	/// @par
	///
	/// If the end polygon cannot be reached through the navigation graph,
	/// the last polygon in the path will be the nearest the end polygon.
	///
	/// If the path array is to small to hold the full result, it will be filled as
	/// far as possible from the start polygon toward the end polygon.
	///
	/// The start and end positions are used to calculate traversal costs.
	/// (The y-values impact the result.)
	///
	public Tupple2<Status, List<Long>> findPath(long startRef, long endRef, float[] startPos, float[] endPos,
			QueryFilter filter) {
		if (startRef == 0 || endRef == 0)
			throw new IllegalArgumentException("Start or end ref = 0");

		// Validate input
		if (!m_nav.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef))
			throw new IllegalArgumentException("Invalid start or end ref");

		List<Long> path = new ArrayList<>(64);
		if (startRef == endRef) {
			path.add(startRef);
			return new Tupple2<>(Status.SUCCSESS, path);
		}

		m_nodePool.clear();
		m_openList.clear();

		Node startNode = m_nodePool.getNode(startRef);
		vCopy(startNode.pos, startPos);
		startNode.pidx = 0;
		startNode.cost = 0;
		startNode.total = vDist(startPos, endPos) * H_SCALE;
		startNode.id = startRef;
		startNode.flags = Node.DT_NODE_OPEN;
		m_openList.push(startNode);

		Node lastBestNode = startNode;
		float lastBestNodeCost = startNode.total;

		Status status = Status.SUCCSESS;

		while (!m_openList.isEmpty()) {
			// Remove node from open list and put it in closed list.
			Node bestNode = m_openList.pop();
			bestNode.flags &= ~Node.DT_NODE_OPEN;
			bestNode.flags |= Node.DT_NODE_CLOSED;

			// Reached the goal, stop searching.
			if (bestNode.id == endRef) {
				lastBestNode = bestNode;
				break;
			}

			// Get current poly and tile.
			// The API input has been cheked already, skip checking internal data.
			long bestRef = bestNode.id;
			Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(bestRef);
			MeshTile bestTile = tileAndPoly.first;
			Poly bestPoly = tileAndPoly.second;

			// Get parent poly and tile.
			long parentRef = 0;
			MeshTile parentTile = null;
			Poly parentPoly = null;
			if (bestNode.pidx != 0)
				parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
			if (parentRef != 0) {
				tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
				parentTile = tileAndPoly.first;
				parentPoly = tileAndPoly.second;
			}

			for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
				long neighbourRef = bestTile.links.get(i).ref;

				// Skip invalid ids and do not expand back to where we came from.
				if (neighbourRef == 0 || neighbourRef == parentRef)
					continue;

				// Get neighbour poly and tile.
				// The API input has been cheked already, skip checking internal data.
				tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
				MeshTile neighbourTile = tileAndPoly.first;
				Poly neighbourPoly = tileAndPoly.second;

				if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;

				// deal explicitly with crossing tile boundaries
				int crossSide = 0;
				if (bestTile.links.get(i).side != 0xff)
					crossSide = bestTile.links.get(i).side >> 1;

				// get the node
				Node neighbourNode = m_nodePool.getNode(neighbourRef, crossSide);
				if (neighbourNode == null) {
					// status |= DT_OUT_OF_NODES;
					continue;
				}

				// If the node is visited the first time, calculate node position.
				if (neighbourNode.flags == 0) {
					Tupple2<Status, float[]> midPoint = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef,
							neighbourPoly, neighbourTile);
					neighbourNode.pos = midPoint.second;
				}

				// Calculate cost and heuristic.
				float cost = 0;
				float heuristic = 0;

				// Special case for last node.
				if (neighbourRef == endRef) {
					// Cost
					float curCost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly,
							bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
					float endCost = filter.getCost(neighbourNode.pos, endPos, bestRef, bestTile, bestPoly, neighbourRef,
							neighbourTile, neighbourPoly, 0L, null, null);

					cost = bestNode.cost + curCost + endCost;
					heuristic = 0;
				} else {
					// Cost
					float curCost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly,
							bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
					cost = bestNode.cost + curCost;
					heuristic = vDist(neighbourNode.pos, endPos) * H_SCALE;
				}

				float total = cost + heuristic;

				// The node is already in open list and the new result is worse, skip.
				if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					continue;
				// The node is already visited and process, and the new result is worse, skip.
				if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0 && total >= neighbourNode.total)
					continue;

				// Add or update the node.
				neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
				neighbourNode.id = neighbourRef;
				neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
				neighbourNode.cost = cost;
				neighbourNode.total = total;

				if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0) {
					// Already in open, update node location.
					m_openList.modify(neighbourNode);
				} else {
					// Put the node in open list.
					neighbourNode.flags |= Node.DT_NODE_OPEN;
					m_openList.push(neighbourNode);
				}

				// Update nearest node to target so far.
				if (heuristic < lastBestNodeCost) {
					lastBestNodeCost = heuristic;
					lastBestNode = neighbourNode;
				}
			}
		}

		if (lastBestNode.id != endRef)
			status = Status.PARTIAL_RESULT;

		// Reverse the path.
		Node prev = null;
		Node node = lastBestNode;
		do {
			Node next = m_nodePool.getNodeAtIdx(node.pidx);
			node.pidx = m_nodePool.getNodeIdx(prev);
			prev = node;
			node = next;
		} while (node != null);

		// Store path
		node = prev;
		do {
			path.add(node.id);
			node = m_nodePool.getNodeAtIdx(node.pidx);
		} while (node != null);

		return new Tupple2<>(status, path);
	}

	/// @par
	///
	/// @warning Calling any non-slice methods before calling finalizeSlicedFindPath()
	/// or finalizeSlicedFindPathPartial() may result in corrupted data!
	///
	/// The @p filter pointer is stored and used for the duration of the sliced
	/// path query.
	///
	public Status initSlicedFindPath(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter,
			int options) {
		// Init path state.
		m_query = new QueryData();
		m_query.status = Status.FAILURE;
		m_query.startRef = startRef;
		m_query.endRef = endRef;
		vCopy(m_query.startPos, startPos);
		vCopy(m_query.endPos, endPos);
		m_query.filter = filter;
		m_query.options = options;
		m_query.raycastLimitSqr = Float.MAX_VALUE;

		if (startRef == 0 || endRef == 0)
			throw new IllegalArgumentException("Start or end ref = 0");

		// Validate input
		if (!m_nav.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef))
			throw new IllegalArgumentException("Invalid start or end ref");

		// trade quality with performance?
		if ((options & DT_FINDPATH_ANY_ANGLE) != 0) {
			// limiting to several times the character radius yields nice results. It is not sensitive
			// so it is enough to compute it from the first tile.
			MeshTile tile = m_nav.getTileByRef(startRef);
			float agentRadius = tile.header.walkableRadius;
			m_query.raycastLimitSqr = sqr(agentRadius * NavMesh.DT_RAY_CAST_LIMIT_PROPORTIONS);
		}

		if (startRef == endRef) {
			m_query.status = Status.SUCCSESS;
			return Status.SUCCSESS;
		}

		m_nodePool.clear();
		m_openList.clear();

		Node startNode = m_nodePool.getNode(startRef);
		vCopy(startNode.pos, startPos);
		startNode.pidx = 0;
		startNode.cost = 0;
		startNode.total = vDist(startPos, endPos) * H_SCALE;
		startNode.id = startRef;
		startNode.flags = Node.DT_NODE_OPEN;
		m_openList.push(startNode);

		m_query.status = Status.IN_PROGRESS;
		m_query.lastBestNode = startNode;
		m_query.lastBestNodeCost = startNode.total;

		return m_query.status;
	}

	public Tupple2<Status, Integer> updateSlicedFindPath(int maxIter) {
		if (!m_query.status.isInProgress())
			return new Tupple2<>(m_query.status, 0);

		// Make sure the request is still valid.
		if (!m_nav.isValidPolyRef(m_query.startRef) || !m_nav.isValidPolyRef(m_query.endRef)) {
			m_query.status = Status.FAILURE;
			return new Tupple2<>(m_query.status, 0);
		}

		int iter = 0;
		while (iter < maxIter && !m_openList.isEmpty()) {
			iter++;

			// Remove node from open list and put it in closed list.
			Node bestNode = m_openList.pop();
			bestNode.flags &= ~Node.DT_NODE_OPEN;
			bestNode.flags |= Node.DT_NODE_CLOSED;

			// Reached the goal, stop searching.
			if (bestNode.id == m_query.endRef) {
				m_query.lastBestNode = bestNode;
				m_query.status = Status.SUCCSESS;
				return new Tupple2<>(m_query.status, iter);
			}

			// Get current poly and tile.
			// The API input has been cheked already, skip checking internal
			// data.
			long bestRef = bestNode.id;
			Tupple2<MeshTile, Poly> tileAndPoly;
			try {
				tileAndPoly = m_nav.getTileAndPolyByRef(bestRef);
			} catch (IllegalArgumentException e) {
				m_query.status = Status.FAILURE;
				// The polygon has disappeared during the sliced query, fail.
				return new Tupple2<>(m_query.status, iter);
			}
			MeshTile bestTile = tileAndPoly.first;
			Poly bestPoly = tileAndPoly.second;
			// Get parent and grand parent poly and tile.
			long parentRef = 0, grandpaRef = 0;
			MeshTile parentTile = null;
			Poly parentPoly = null;
			Node parentNode = null;
			if (bestNode.pidx != 0) {
				parentNode = m_nodePool.getNodeAtIdx(bestNode.pidx);
				parentRef = parentNode.id;
				if (parentNode.pidx != 0)
					grandpaRef = m_nodePool.getNodeAtIdx(parentNode.pidx).id;
			}
			if (parentRef != 0) {
				boolean invalidParent = false;
				try {
					tileAndPoly = m_nav.getTileAndPolyByRef(parentRef);
					parentTile = tileAndPoly.first;
					parentPoly = tileAndPoly.second;
				} catch (IllegalArgumentException e) {
					invalidParent = true;
				}
				if (invalidParent || (grandpaRef != 0 && !m_nav.isValidPolyRef(grandpaRef))) {
					// The polygon has disappeared during the sliced query,
					// fail.
					m_query.status = Status.FAILURE;
					return new Tupple2<>(m_query.status, iter);
				}
			}

			// decide whether to test raycast to previous nodes
			boolean tryLOS = false;
			if ((m_query.options & DT_FINDPATH_ANY_ANGLE) != 0) {
				if ((parentRef != 0) && (vDistSqr(parentNode.pos, bestNode.pos) < m_query.raycastLimitSqr))
					tryLOS = true;
			}

			for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
				long neighbourRef = bestTile.links.get(i).ref;

				// Skip invalid ids and do not expand back to where we came
				// from.
				if (neighbourRef == 0 || neighbourRef == parentRef)
					continue;

				// Get neighbour poly and tile.
				// The API input has been cheked already, skip checking internal
				// data.
				Tupple2<MeshTile, Poly> tileAndPolyUns = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
				MeshTile neighbourTile = tileAndPolyUns.first;
				Poly neighbourPoly = tileAndPolyUns.second;

				if (!m_query.filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;

				// get the neighbor node
				Node neighbourNode = m_nodePool.getNode(neighbourRef, 0);
				if (neighbourNode == null) {
					m_query.status = Status.PARTIAL_RESULT;
					continue;
				}

				// do not expand to nodes that were already visited from the
				// same parent
				if (neighbourNode.pidx != 0 && neighbourNode.pidx == bestNode.pidx)
					continue;

				// If the node is visited the first time, calculate node
				// position.
				if (neighbourNode.flags == 0) {
					Tupple2<Status, float[]> midPoint = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef,
							neighbourPoly, neighbourTile);
					neighbourNode.pos = midPoint.second;
				}

				// Calculate cost and heuristic.
				float cost = 0;
				float heuristic = 0;

				// raycast parent
				boolean foundShortCut = false;
				if (tryLOS) {
					RaycastHit rayHit = raycast(parentRef, parentNode.pos, neighbourNode.pos, m_query.filter,
							DT_RAYCAST_USE_COSTS, grandpaRef);
					foundShortCut = rayHit.t >= 1.0f;
					if (foundShortCut) {
						// shortcut found using raycast. Using shorter cost
						// instead
						cost = parentNode.cost + rayHit.pathCost;
					}
				}

				// update move cost
				if (!foundShortCut) {
					// No shortcut found.
					float curCost = m_query.filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile,
							parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
					cost = bestNode.cost + curCost;
				}

				// Special case for last node.
				if (neighbourRef == m_query.endRef) {
					float endCost = m_query.filter.getCost(neighbourNode.pos, m_query.endPos, bestRef, bestTile,
							bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0, null, null);

					cost = cost + endCost;
					heuristic = 0;
				} else {
					heuristic = vDist(neighbourNode.pos, m_query.endPos) * H_SCALE;
				}

				float total = cost + heuristic;

				// The node is already in open list and the new result is worse,
				// skip.
				if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					continue;
				// The node is already visited and process, and the new result
				// is worse, skip.
				if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0 && total >= neighbourNode.total)
					continue;

				// Add or update the node.
				neighbourNode.pidx = foundShortCut ? bestNode.pidx : m_nodePool.getNodeIdx(bestNode);
				neighbourNode.id = neighbourRef;
				neighbourNode.flags = (neighbourNode.flags & ~(Node.DT_NODE_CLOSED | Node.DT_NODE_PARENT_DETACHED));
				neighbourNode.cost = cost;
				neighbourNode.total = total;
				if (foundShortCut)
					neighbourNode.flags = (neighbourNode.flags | Node.DT_NODE_PARENT_DETACHED);

				if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0) {
					// Already in open, update node location.
					m_openList.modify(neighbourNode);
				} else {
					// Put the node in open list.
					neighbourNode.flags |= Node.DT_NODE_OPEN;
					m_openList.push(neighbourNode);
				}

				// Update nearest node to target so far.
				if (heuristic < m_query.lastBestNodeCost) {
					m_query.lastBestNodeCost = heuristic;
					m_query.lastBestNode = neighbourNode;
				}
			}
		}

		// Exhausted all nodes, but could not find path.
		if (m_openList.isEmpty()) {
			m_query.status = Status.PARTIAL_RESULT;
		}

		return new Tupple2<>(m_query.status, iter);
	}
	
	public Tupple2<Status, List<Long>> finalizeSlicedFindPath() {

		List<Long> path = new ArrayList<>(64);
		if (m_query.status.isFailed()) {
			// Reset query.
			m_query = new QueryData();
			return new Tupple2<Status, List<Long>>(Status.FAILURE, path);
		}

		if (m_query.startRef == m_query.endRef) {
			// Special case: the search starts and ends at same poly.
			path.add(m_query.startRef);
		} else {
			// Reverse the path.
			if (m_query.lastBestNode.id != m_query.endRef)
				m_query.status = Status.PARTIAL_RESULT;

			Node prev = null;
			Node node = m_query.lastBestNode;
			int prevRay = 0;
			do {
				Node next = m_nodePool.getNodeAtIdx(node.pidx);
				node.pidx = m_nodePool.getNodeIdx(prev);
				prev = node;
				int nextRay = node.flags & Node.DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
				node.flags = (node.flags & ~Node.DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
				prevRay = nextRay;
				node = next;
			} while (node != null);

			// Store path
			node = prev;
			do {
				Node next = m_nodePool.getNodeAtIdx(node.pidx);
				if ((node.flags & Node.DT_NODE_PARENT_DETACHED) != 0) {
					RaycastHit iresult = raycast(node.id, node.pos, next.pos, m_query.filter, 0, 0);
					path.addAll(iresult.path);
					// raycast ends on poly boundary and the path might include the next poly boundary.
					if (path.get(path.size() - 1) == next.id)
						path.remove(path.size() - 1); // remove to avoid duplicates
				} else {
					path.add(node.id);
				}

				node = next;
			} while (node != null);
		}

		Status status = m_query.status;
		// Reset query.
		m_query = new QueryData();

		return new Tupple2<Status, List<Long>>(status, path);
	}

	public Tupple2<Status, List<Long>> finalizeSlicedFindPathPartial(List<Long> existing) {

		List<Long> path = new ArrayList<>(64);
		if (existing.size() == 0) {
			return new Tupple2<Status, List<Long>>(Status.FAILURE, path);
		}
		if (m_query.status.isFailed()) {
			// Reset query.
			m_query = new QueryData();
			return new Tupple2<Status, List<Long>>(Status.FAILURE, path);
		}
		if (m_query.startRef == m_query.endRef) {
			// Special case: the search starts and ends at same poly.
			path.add(m_query.startRef);
		} else {
			// Find furthest existing node that was visited.
			Node prev = null;
			Node node = null;
			for (int i = existing.size()-1; i >= 0; --i)
			{
				node = m_nodePool.findNode(existing.get(i));
				if (node != null)
					break;
			}
				
			if (node == null)
			{
				m_query.status = Status.PARTIAL_RESULT;
				node = m_query.lastBestNode;
			}
				
			// Reverse the path.
			int prevRay = 0;
			do {
				Node next = m_nodePool.getNodeAtIdx(node.pidx);
				node.pidx = m_nodePool.getNodeIdx(prev);
				prev = node;
				int nextRay = node.flags & Node.DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
				node.flags = (node.flags & ~Node.DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
				prevRay = nextRay;
				node = next;
			} while (node != null);
				
			// Store path
			node = prev;
			do {
				Node next = m_nodePool.getNodeAtIdx(node.pidx);
				if ((node.flags & Node.DT_NODE_PARENT_DETACHED) != 0) {
					RaycastHit iresult = raycast(node.id, node.pos, next.pos, m_query.filter, 0, 0);
					path.addAll(iresult.path);
					// raycast ends on poly boundary and the path might include the next poly boundary.
					if (path.get(path.size() - 1) == next.id)
						path.remove(path.size() - 1); // remove to avoid duplicates
				} else {
					path.add(node.id);
				}

				node = next;
			} while (node != null);
		}
		Status status = m_query.status;
		// Reset query.
		m_query = new QueryData();

		return new Tupple2<Status, List<Long>>(status, path);
	}	
	/*
	
	dtStatus dtNavMeshQuery::appendVertex(const float* pos, const unsigned char flags, const dtPolyRef ref,
										  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
										  int* straightPathCount, const int maxStraightPath) const
	{
		if ((*straightPathCount) > 0 && dtVequal(&straightPath[((*straightPathCount)-1)*3], pos))
		{
			// The vertices are equal, update flags and poly.
			if (straightPathFlags)
				straightPathFlags[(*straightPathCount)-1] = flags;
			if (straightPathRefs)
				straightPathRefs[(*straightPathCount)-1] = ref;
		}
		else
		{
			// Append new vertex.
			dtVcopy(&straightPath[(*straightPathCount)*3], pos);
			if (straightPathFlags)
				straightPathFlags[(*straightPathCount)] = flags;
			if (straightPathRefs)
				straightPathRefs[(*straightPathCount)] = ref;
			(*straightPathCount)++;
			// If reached end of path or there is no space to append more vertices, return.
			if (flags == DT_STRAIGHTPATH_END || (*straightPathCount) >= maxStraightPath)
			{
				return DT_SUCCESS | (((*straightPathCount) >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
			}
		}
		return DT_IN_PROGRESS;
	}
	
	dtStatus dtNavMeshQuery::appendPortals(const int startIdx, const int endIdx, const float* endPos, const dtPolyRef* path,
										  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
										  int* straightPathCount, const int maxStraightPath, const int options) const
	{
		const float* startPos = &straightPath[(*straightPathCount-1)*3];
		// Append or update last vertex
		dtStatus stat = 0;
		for (int i = startIdx; i < endIdx; i++)
		{
			// Calculate portal
			const dtPolyRef from = path[i];
			const dtMeshTile* fromTile = 0;
			const dtPoly* fromPoly = 0;
			if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
				return DT_FAILURE | DT_INVALID_PARAM;
			
			const dtPolyRef to = path[i+1];
			const dtMeshTile* toTile = 0;
			const dtPoly* toPoly = 0;
			if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
				return DT_FAILURE | DT_INVALID_PARAM;
			
			float left[3], right[3];
			if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
				break;
		
			if (options & DT_STRAIGHTPATH_AREA_CROSSINGS)
			{
				// Skip intersection if only area crossings are requested.
				if (fromPoly->getArea() == toPoly->getArea())
					continue;
			}
			
			// Append intersection
			float s,t;
			if (dtIntersectSegSeg2D(startPos, endPos, left, right, s, t))
			{
				float pt[3];
				dtVlerp(pt, left,right, t);
	
				stat = appendVertex(pt, 0, path[i+1],
									straightPath, straightPathFlags, straightPathRefs,
									straightPathCount, maxStraightPath);
				if (stat != DT_IN_PROGRESS)
					return stat;
			}
		}
		return DT_IN_PROGRESS;
	}
	
	/// @par
	/// 
	/// This method peforms what is often called 'string pulling'.
	///
	/// The start position is clamped to the first polygon in the path, and the 
	/// end position is clamped to the last. So the start and end positions should 
	/// normally be within or very near the first and last polygons respectively.
	///
	/// The returned polygon references represent the reference id of the polygon 
	/// that is entered at the associated path position. The reference id associated 
	/// with the end point will always be zero.  This allows, for example, matching 
	/// off-mesh link points to their representative polygons.
	///
	/// If the provided result buffers are too small for the entire result set, 
	/// they will be filled as far as possible from the start toward the end 
	/// position.
	///
	dtStatus dtNavMeshQuery::findStraightPath(const float* startPos, const float* endPos,
											  const dtPolyRef* path, const int pathSize,
											  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
											  int* straightPathCount, const int maxStraightPath, const int options) const
	{
		dtAssert(m_nav);
		
		*straightPathCount = 0;
		
		if (!maxStraightPath)
			return DT_FAILURE | DT_INVALID_PARAM;
		
		if (!path[0])
			return DT_FAILURE | DT_INVALID_PARAM;
		
		dtStatus stat = 0;
		
		// TODO: Should this be callers responsibility?
		float closestStartPos[3];
		if (dtStatusFailed(closestPointOnPolyBoundary(path[0], startPos, closestStartPos)))
			return DT_FAILURE | DT_INVALID_PARAM;
	
		float closestEndPos[3];
		if (dtStatusFailed(closestPointOnPolyBoundary(path[pathSize-1], endPos, closestEndPos)))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		// Add start point.
		stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path[0],
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath);
		if (stat != DT_IN_PROGRESS)
			return stat;
		
		if (pathSize > 1)
		{
			float portalApex[3], portalLeft[3], portalRight[3];
			dtVcopy(portalApex, closestStartPos);
			dtVcopy(portalLeft, portalApex);
			dtVcopy(portalRight, portalApex);
			int apexIndex = 0;
			int leftIndex = 0;
			int rightIndex = 0;
			
			unsigned char leftPolyType = 0;
			unsigned char rightPolyType = 0;
			
			dtPolyRef leftPolyRef = path[0];
			dtPolyRef rightPolyRef = path[0];
			
			for (int i = 0; i < pathSize; ++i)
			{
				float left[3], right[3];
				unsigned char fromType, toType;
				
				if (i+1 < pathSize)
				{
					// Next portal.
					if (dtStatusFailed(getPortalPoints(path[i], path[i+1], left, right, fromType, toType)))
					{
						// Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
						// Clamp the end point to path[i], and return the path so far.
						
						if (dtStatusFailed(closestPointOnPolyBoundary(path[i], endPos, closestEndPos)))
						{
							// This should only happen when the first polygon is invalid.
							return DT_FAILURE | DT_INVALID_PARAM;
						}
	
						// Apeend portals along the current straight path segment.
						if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
						{
							stat = appendPortals(apexIndex, i, closestEndPos, path,
												 straightPath, straightPathFlags, straightPathRefs,
												 straightPathCount, maxStraightPath, options);
						}
	
						stat = appendVertex(closestEndPos, 0, path[i],
											straightPath, straightPathFlags, straightPathRefs,
											straightPathCount, maxStraightPath);
						
						return DT_SUCCESS | DT_PARTIAL_RESULT | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
					}
					
					// If starting really close the portal, advance.
					if (i == 0)
					{
						float t;
						if (dtDistancePtSegSqr2D(portalApex, left, right, t) < dtSqr(0.001f))
							continue;
					}
				}
				else
				{
					// End of the path.
					dtVcopy(left, closestEndPos);
					dtVcopy(right, closestEndPos);
					
					fromType = toType = DT_POLYTYPE_GROUND;
				}
				
				// Right vertex.
				if (dtTriArea2D(portalApex, portalRight, right) <= 0.0f)
				{
					if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) > 0.0f)
					{
						dtVcopy(portalRight, right);
						rightPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
						rightPolyType = toType;
						rightIndex = i;
					}
					else
					{
						// Append portals along the current straight path segment.
						if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
						{
							stat = appendPortals(apexIndex, leftIndex, portalLeft, path,
												 straightPath, straightPathFlags, straightPathRefs,
												 straightPathCount, maxStraightPath, options);
							if (stat != DT_IN_PROGRESS)
								return stat;					
						}
					
						dtVcopy(portalApex, portalLeft);
						apexIndex = leftIndex;
						
						unsigned char flags = 0;
						if (!leftPolyRef)
							flags = DT_STRAIGHTPATH_END;
						else if (leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
							flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
						dtPolyRef ref = leftPolyRef;
						
						// Append or update vertex
						stat = appendVertex(portalApex, flags, ref,
											straightPath, straightPathFlags, straightPathRefs,
											straightPathCount, maxStraightPath);
						if (stat != DT_IN_PROGRESS)
							return stat;
						
						dtVcopy(portalLeft, portalApex);
						dtVcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;
						
						// Restart
						i = apexIndex;
						
						continue;
					}
				}
				
				// Left vertex.
				if (dtTriArea2D(portalApex, portalLeft, left) >= 0.0f)
				{
					if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) < 0.0f)
					{
						dtVcopy(portalLeft, left);
						leftPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
						leftPolyType = toType;
						leftIndex = i;
					}
					else
					{
						// Append portals along the current straight path segment.
						if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
						{
							stat = appendPortals(apexIndex, rightIndex, portalRight, path,
												 straightPath, straightPathFlags, straightPathRefs,
												 straightPathCount, maxStraightPath, options);
							if (stat != DT_IN_PROGRESS)
								return stat;
						}
	
						dtVcopy(portalApex, portalRight);
						apexIndex = rightIndex;
						
						unsigned char flags = 0;
						if (!rightPolyRef)
							flags = DT_STRAIGHTPATH_END;
						else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
							flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
						dtPolyRef ref = rightPolyRef;
	
						// Append or update vertex
						stat = appendVertex(portalApex, flags, ref,
											straightPath, straightPathFlags, straightPathRefs,
											straightPathCount, maxStraightPath);
						if (stat != DT_IN_PROGRESS)
							return stat;
						
						dtVcopy(portalLeft, portalApex);
						dtVcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;
						
						// Restart
						i = apexIndex;
						
						continue;
					}
				}
			}
	
			// Append portals along the current straight path segment.
			if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
			{
				stat = appendPortals(apexIndex, pathSize-1, closestEndPos, path,
									 straightPath, straightPathFlags, straightPathRefs,
									 straightPathCount, maxStraightPath, options);
				if (stat != DT_IN_PROGRESS)
					return stat;
			}
		}
	
		stat = appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0,
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath);
		
		return DT_SUCCESS | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
	}
	
	/// @par
	///
	/// This method is optimized for small delta movement and a small number of 
	/// polygons. If used for too great a distance, the result set will form an 
	/// incomplete path.
	///
	/// @p resultPos will equal the @p endPos if the end is reached. 
	/// Otherwise the closest reachable position will be returned.
	/// 
	/// @p resultPos is not projected onto the surface of the navigation 
	/// mesh. Use #getPolyHeight if this is needed.
	///
	/// This method treats the end position in the same manner as 
	/// the #raycast method. (As a 2D point.) See that method's documentation 
	/// for details.
	/// 
	/// If the @p visited array is too small to hold the entire result set, it will 
	/// be filled as far as possible from the start position toward the end 
	/// position.
	///
	dtStatus dtNavMeshQuery::moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
											  const dtQueryFilter* filter,
											  float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const
	{
		dtAssert(m_nav);
		dtAssert(m_tinyNodePool);
	
		*visitedCount = 0;
		
		// Validate input
		if (!startRef)
			return DT_FAILURE | DT_INVALID_PARAM;
		if (!m_nav->isValidPolyRef(startRef))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		dtStatus status = DT_SUCCESS;
		
		static const int MAX_STACK = 48;
		dtNode* stack[MAX_STACK];
		int nstack = 0;
		
		m_tinyNodePool->clear();
		
		dtNode* startNode = m_tinyNodePool->getNode(startRef);
		startNode->pidx = 0;
		startNode->cost = 0;
		startNode->total = 0;
		startNode->id = startRef;
		startNode->flags = DT_NODE_CLOSED;
		stack[nstack++] = startNode;
		
		float bestPos[3];
		float bestDist = FLT_MAX;
		dtNode* bestNode = 0;
		dtVcopy(bestPos, startPos);
		
		// Search constraints
		float searchPos[3], searchRadSqr;
		dtVlerp(searchPos, startPos, endPos, 0.5f);
		searchRadSqr = dtSqr(dtVdist(startPos, endPos)/2.0f + 0.001f);
		
		float verts[DT_VERTS_PER_POLYGON*3];
		
		while (nstack)
		{
			// Pop front.
			dtNode* curNode = stack[0];
			for (int i = 0; i < nstack-1; ++i)
				stack[i] = stack[i+1];
			nstack--;
			
			// Get poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtPolyRef curRef = curNode->id;
			const dtMeshTile* curTile = 0;
			const dtPoly* curPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);			
			
			// Collect vertices.
			const int nverts = curPoly->vertCount;
			for (int i = 0; i < nverts; ++i)
				dtVcopy(&verts[i*3], &curTile->verts[curPoly->verts[i]*3]);
			
			// If target is inside the poly, stop search.
			if (dtPointInPolygon(endPos, verts, nverts))
			{
				bestNode = curNode;
				dtVcopy(bestPos, endPos);
				break;
			}
			
			// Find wall edges and find nearest point inside the walls.
			for (int i = 0, j = (int)curPoly->vertCount-1; i < (int)curPoly->vertCount; j = i++)
			{
				// Find links to neighbours.
				static const int MAX_NEIS = 8;
				int nneis = 0;
				dtPolyRef neis[MAX_NEIS];
				
				if (curPoly->neis[j] & DT_EXT_LINK)
				{
					// Tile border.
					for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
					{
						const dtLink* link = &curTile->links[k];
						if (link->edge == j)
						{
							if (link->ref != 0)
							{
								const dtMeshTile* neiTile = 0;
								const dtPoly* neiPoly = 0;
								m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
								if (filter->passFilter(link->ref, neiTile, neiPoly))
								{
									if (nneis < MAX_NEIS)
										neis[nneis++] = link->ref;
								}
							}
						}
					}
				}
				else if (curPoly->neis[j])
				{
					const unsigned int idx = (unsigned int)(curPoly->neis[j]-1);
					const dtPolyRef ref = m_nav->getPolyRefBase(curTile) | idx;
					if (filter->passFilter(ref, curTile, &curTile->polys[idx]))
					{
						// Internal edge, encode id.
						neis[nneis++] = ref;
					}
				}
				
				if (!nneis)
				{
					// Wall edge, calc distance.
					const float* vj = &verts[j*3];
					const float* vi = &verts[i*3];
					float tseg;
					const float distSqr = dtDistancePtSegSqr2D(endPos, vj, vi, tseg);
					if (distSqr < bestDist)
					{
	                // Update nearest distance.
						dtVlerp(bestPos, vj,vi, tseg);
						bestDist = distSqr;
						bestNode = curNode;
					}
				}
				else
				{
					for (int k = 0; k < nneis; ++k)
					{
						// Skip if no node can be allocated.
						dtNode* neighbourNode = m_tinyNodePool->getNode(neis[k]);
						if (!neighbourNode)
							continue;
						// Skip if already visited.
						if (neighbourNode->flags & DT_NODE_CLOSED)
							continue;
						
						// Skip the link if it is too far from search constraint.
						// TODO: Maybe should use getPortalPoints(), but this one is way faster.
						const float* vj = &verts[j*3];
						const float* vi = &verts[i*3];
						float tseg;
						float distSqr = dtDistancePtSegSqr2D(searchPos, vj, vi, tseg);
						if (distSqr > searchRadSqr)
							continue;
						
						// Mark as the node as visited and push to queue.
						if (nstack < MAX_STACK)
						{
							neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
							neighbourNode->flags |= DT_NODE_CLOSED;
							stack[nstack++] = neighbourNode;
						}
					}
				}
			}
		}
		
		int n = 0;
		if (bestNode)
		{
			// Reverse the path.
			dtNode* prev = 0;
			dtNode* node = bestNode;
			do
			{
				dtNode* next = m_tinyNodePool->getNodeAtIdx(node->pidx);
				node->pidx = m_tinyNodePool->getNodeIdx(prev);
				prev = node;
				node = next;
			}
			while (node);
			
			// Store result
			node = prev;
			do
			{
				visited[n++] = node->id;
				if (n >= maxVisitedSize)
				{
					status |= DT_BUFFER_TOO_SMALL;
					break;
				}
				node = m_tinyNodePool->getNodeAtIdx(node->pidx);
			}
			while (node);
		}
		
		dtVcopy(resultPos, bestPos);
		
		*visitedCount = n;
		
		return status;
	}
	
	*/
	static class PortalResult {
		final Status status;
		final float[] left;
		final float[] right;
		final int fromType;
		final int toType;

		public PortalResult(Status status) {
			this.status = status;
			fromType = toType = 0;
			left = right = null;
		}

		public PortalResult(Status status, float[] left, float[] right, int fromType, int toType) {
			this.status = status;
			this.left = left;
			this.right = right;
			this.fromType = fromType;
			this.toType = toType;
		}

	}

	PortalResult getPortalPoints(long from, long to) {
		Tupple2<MeshTile,Poly> tileAndPoly = m_nav.getTileAndPolyByRef(from);
		MeshTile fromTile = tileAndPoly.first;
		Poly fromPoly = tileAndPoly.second;
		int fromType = fromPoly.getType();

		tileAndPoly = m_nav.getTileAndPolyByRef(to);
		MeshTile toTile = tileAndPoly.first;
		Poly toPoly = tileAndPoly.second;
		int toType = toPoly.getType();

		return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, fromType, toType);
	}

	// Returns portal points between two polygons.
	PortalResult getPortalPoints(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly, MeshTile toTile,
			int fromType, int toType) {
		float[] left = new float[3];
		float[] right = new float[3];
		// Find the link that points to the 'to' polygon.
		Link link = null;
		for (int i = fromPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = fromTile.links.get(i).next) {
			if (fromTile.links.get(i).ref == to) {
				link = fromTile.links.get(i);
				break;
			}
		}
		if (link == null)
			throw new IllegalArgumentException("Null link");

		// Handle off-mesh connections.
		if (fromPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
			// Find link that points to first vertex.
			for (int i = fromPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = fromTile.links.get(i).next) {
				if (fromTile.links.get(i).ref == to) {
					int v = fromTile.links.get(i).edge;
					System.arraycopy(fromTile.verts, fromPoly.verts[v] * 3, left, 0, 3);
					System.arraycopy(fromTile.verts, fromPoly.verts[v] * 3, right, 0, 3);
					return new PortalResult(Status.SUCCSESS, left, right, fromType, toType);
				}
			}
			throw new IllegalArgumentException("Invalid offmesh from connection");
		}

		if (toPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
			for (int i = toPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = toTile.links.get(i).next) {
				if (toTile.links.get(i).ref == from) {
					int v = toTile.links.get(i).edge;
					System.arraycopy(toTile.verts, toPoly.verts[v] * 3, left, 0, 3);
					System.arraycopy(toTile.verts, toPoly.verts[v] * 3, right, 0, 3);
					return new PortalResult(Status.SUCCSESS, left, right, fromType, toType);
				}
			}
			throw new IllegalArgumentException("Invalid offmesh to connection");
		}

		// Find portal vertices.
		int v0 = fromPoly.verts[link.edge];
		int v1 = fromPoly.verts[(link.edge + 1) % (int) fromPoly.vertCount];
		System.arraycopy(fromTile.verts, v0 * 3, left, 0, 3);
		System.arraycopy(fromTile.verts, v1 * 3, right, 0, 3);

		// If the link is at tile boundary, dtClamp the vertices to
		// the link width.
		if (link.side != 0xff) {
			// Unpack portal limits.
			if (link.bmin != 0 || link.bmax != 255) {
				float s = 1.0f / 255.0f;
				float tmin = link.bmin * s;
				float tmax = link.bmax * s;
				left = vLerp(fromTile.verts, v0 * 3, v1 * 3, tmin);
				right = vLerp(fromTile.verts, v0 * 3, v1 * 3, tmax);
			}
		}

		return new PortalResult(Status.SUCCSESS, left, right, fromType, toType);
	}

	// Returns edge mid point between two polygons.
	Tupple2<Status, float[]> getEdgeMidPoint(long from, long to) {
		PortalResult ppoints = getPortalPoints(from, to);
		float[] left = ppoints.left;
		float[] right = ppoints.right;
		float[] mid = new float[3];
		mid[0] = (left[0] + right[0]) * 0.5f;
		mid[1] = (left[1] + right[1]) * 0.5f;
		mid[2] = (left[2] + right[2]) * 0.5f;
		return new Tupple2<>(Status.SUCCSESS, mid);
	}

	Tupple2<Status, float[]> getEdgeMidPoint(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly,
			MeshTile toTile) {
		PortalResult ppoints = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
		float[] left = ppoints.left;
		float[] right = ppoints.right;
		float[] mid = new float[3];
		mid[0] = (left[0] + right[0]) * 0.5f;
		mid[1] = (left[1] + right[1]) * 0.5f;
		mid[2] = (left[2] + right[2]) * 0.5f;
		return new Tupple2<>(Status.SUCCSESS, mid);
	}

	/// @par
	///
	/// This method is meant to be used for quick, short distance checks.
	///
	/// If the path array is too small to hold the result, it will be filled as 
	/// far as possible from the start postion toward the end position.
	///
	/// <b>Using the Hit Parameter t of RaycastHit</b>
	/// 
	/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
	/// the end position. In this case the path represents a valid corridor to the 
	/// end position and the value of @p hitNormal is undefined.
	///
	/// If the hit parameter is zero, then the start position is on the wall that 
	/// was hit and the value of @p hitNormal is undefined.
	///
	/// If 0 < t < 1.0 then the following applies:
	///
	/// @code
	/// distanceToHitBorder = distanceToEndPosition * t
	/// hitPoint = startPos + (endPos - startPos) * t
	/// @endcode
	///
	/// <b>Use Case Restriction</b>
	///
	/// The raycast ignores the y-value of the end position. (2D check.) This 
	/// places significant limits on how it can be used. For example:
	///
	/// Consider a scene where there is a main floor with a second floor balcony 
	/// that hangs over the main floor. So the first floor mesh extends below the 
	/// balcony mesh. The start position is somewhere on the first floor. The end 
	/// position is on the balcony.
	///
	/// The raycast will search toward the end position along the first floor mesh. 
	/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
	/// (no wall hit), meaning it reached the end position. This is one example of why
	/// this method is meant for short distance checks.
	///
	private static float s = 1.0f/255.0f;

	RaycastHit raycast(long startRef, float[] startPos, float[] endPos, QueryFilter filter, int options, long prevRef) {
		// Validate input
		if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			throw new IllegalArgumentException("Invalid start ref");
		if (prevRef != 0 && !m_nav.isValidPolyRef(prevRef))
			throw new IllegalArgumentException("Invalid pref ref");

		RaycastHit hit = new RaycastHit();
		hit.status = Status.SUCCSESS;

		float[] verts = new float[NavMesh.DT_VERTS_PER_POLYGON * 3 + 3];

		float[] curPos = new float[3], lastPos = new float[3];
		VectorPtr curPosV = new VectorPtr(curPos);

		vCopy(curPos, startPos);
		float[] dir = vSub(endPos, startPos);

		MeshTile prevTile, tile, nextTile;
		Poly prevPoly, poly, nextPoly;

		// The API input has been checked already, skip checking internal data.
		long curRef = startRef;
		Tupple2<MeshTile, Poly> tileAndPolyUns = m_nav.getTileAndPolyByRefUnsafe(curRef);
		tile = tileAndPolyUns.first;
		poly = tileAndPolyUns.second;
		nextTile = prevTile = tile;
		nextPoly = prevPoly = poly;
		if (prevRef != 0) {
			tileAndPolyUns = m_nav.getTileAndPolyByRefUnsafe(prevRef);
			prevTile = tileAndPolyUns.first;
			prevPoly = tileAndPolyUns.second;
		}
		while (curRef != 0) {
			// Cast ray against current polygon.

			// Collect vertices.
			int nv = 0;
			for (int i = 0; i < (int) poly.vertCount; ++i) {
				System.arraycopy(tile.verts, poly.verts[i] * 3, verts, nv * 3, 3);
				nv++;
			}

			IntersectResult iresult = intersectSegmentPoly2D(startPos, endPos, verts, nv);
			if (!iresult.intersects) {
				// Could not hit the polygon, keep the old t and report hit.
				return hit;
			}
			// Keep track of furthest t so far.
			if (iresult.tmax > hit.t)
				hit.t = iresult.tmax;

			// Store visited polygons.
			hit.path.add(curRef);

			// Ray end is completely inside the polygon.
			if (iresult.segMax == -1) {
				hit.t = Float.MAX_VALUE;

				// add the cost
				if ((options & DT_RAYCAST_USE_COSTS) != 0)
					hit.pathCost += filter.getCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly,
							curRef, tile, poly);
				return hit;
			}

			// Follow neighbours.
			long nextRef = 0;

			for (int i = poly.firstLink; i != NavMesh.DT_NULL_LINK; i = tile.links.get(i).next) {
				Link link = tile.links.get(i);

				// Find link which contains this edge.
				if (link.edge != iresult.segMax)
					continue;

				// Get pointer to the next polygon.
				tileAndPolyUns = m_nav.getTileAndPolyByRefUnsafe(link.ref);
				nextTile = tileAndPolyUns.first;
				nextPoly = tileAndPolyUns.second;
				// Skip off-mesh connections.
				if (nextPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;

				// Skip links based on filter.
				if (!filter.passFilter(link.ref, nextTile, nextPoly))
					continue;

				// If the link is internal, just return the ref.
				if (link.side == 0xff) {
					nextRef = link.ref;
					break;
				}

				// If the link is at tile boundary,

				// Check if the link spans the whole edge, and accept.
				if (link.bmin == 0 && link.bmax == 255) {
					nextRef = link.ref;
					break;
				}

				// Check for partial edge links.
				int v0 = poly.verts[link.edge];
				int v1 = poly.verts[(link.edge + 1) % poly.vertCount];
				int left = v0 * 3;
				int right = v1 * 3;

				// Check that the intersection lies inside the link portal.
				if (link.side == 0 || link.side == 4) {
					// Calculate link size.
					float lmin = tile.verts[left + 2]
							+ (tile.verts[right + 2] - tile.verts[left + 2]) * (link.bmin * s);
					float lmax = tile.verts[left + 2]
							+ (tile.verts[right + 2] - tile.verts[left + 2]) * (link.bmax * s);
					if (lmin > lmax) {
						float temp = lmin;
						lmin = lmax;
						lmax = temp;
					}

					// Find Z intersection.
					float z = startPos[2] + (endPos[2] - startPos[2]) * iresult.tmax;
					if (z >= lmin && z <= lmax) {
						nextRef = link.ref;
						break;
					}
				} else if (link.side == 2 || link.side == 6) {
					// Calculate link size.
					float lmin = tile.verts[left] + (tile.verts[right] - tile.verts[left]) * (link.bmin * s);
					float lmax = tile.verts[left] + (tile.verts[right] - tile.verts[left]) * (link.bmax * s);
					if (lmin > lmax) {
						float temp = lmin;
						lmin = lmax;
						lmax = temp;
					}

					// Find X intersection.
					float x = startPos[0] + (endPos[0] - startPos[0]) * iresult.tmax;
					if (x >= lmin && x <= lmax) {
						nextRef = link.ref;
						break;
					}
				}
			}

			// add the cost
			if ((options & DT_RAYCAST_USE_COSTS) != 0) {
				// compute the intersection point at the furthest end of the polygon
				// and correct the height (since the raycast moves in 2d)
				vCopy(lastPos, curPos);
				curPos = vMad(startPos, dir, hit.t);
				VectorPtr e1 = new VectorPtr(verts, iresult.segMax * 3);
				VectorPtr e2 = new VectorPtr(verts, ((iresult.segMax + 1) % nv) * 3);
				float[] eDir = vSub(e2, e1);
				float[] diff = vSub(curPosV, e1);
				float s = sqr(eDir[0]) > sqr(eDir[2]) ? diff[0] / eDir[0] : diff[2] / eDir[2];
				curPos[1] = e1.get(1) + eDir[1] * s;

				hit.pathCost += filter.getCost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, tile, poly,
						nextRef, nextTile, nextPoly);
			}

			if (nextRef == 0) {
				// No neighbour, we hit a wall.

				// Calculate hit normal.
				int a = iresult.segMax;
				int b = iresult.segMax + 1 < nv ? iresult.segMax + 1 : 0;
				int va = a * 3;
				int vb = b * 3;
				float dx = verts[vb] - verts[va];
				float dz = verts[vb + 2] - verts[va + 2];
				hit.hitNormal[0] = dz;
				hit.hitNormal[1] = 0;
				hit.hitNormal[2] = -dx;
				vNnormalize(hit.hitNormal);
				return hit;
			}

			// No hit, advance to neighbour polygon.
			prevRef = curRef;
			curRef = nextRef;
			prevTile = tile;
			tile = nextTile;
			prevPoly = poly;
			poly = nextPoly;
		}

		return hit;
	}

	/*
	
	
	/// @par
	///
	/// At least one result array must be provided.
	///
	/// The order of the result set is from least to highest cost to reach the polygon.
	///
	/// A common use case for this method is to perform Dijkstra searches. 
	/// Candidate polygons are found by searching the graph beginning at the start polygon.
	///
	/// If a polygon is not found via the graph search, even if it intersects the 
	/// search circle, it will not be included in the result set. For example:
	///
	/// polyA is the start polygon.
	/// polyB shares an edge with polyA. (Is adjacent.)
	/// polyC shares an edge with polyB, but not with polyA
	/// Even if the search circle overlaps polyC, it will not be included in the 
	/// result set unless polyB is also in the set.
	/// 
	/// The value of the center point is used as the start position for cost 
	/// calculations. It is not projected onto the surface of the mesh, so its 
	/// y-value will effect the costs.
	///
	/// Intersection tests occur in 2D. All polygons and the search circle are 
	/// projected onto the xz-plane. So the y-value of the center point does not 
	/// effect intersection tests.
	///
	/// If the result arrays are to small to hold the entire result set, they will be 
	/// filled to capacity.
	/// 
	dtStatus dtNavMeshQuery::findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
												   const dtQueryFilter* filter,
												   dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
												   int* resultCount, const int maxResult) const
	{
		dtAssert(m_nav);
		dtAssert(m_nodePool);
		dtAssert(m_openList);
	
		*resultCount = 0;
		
		// Validate input
		if (!startRef || !m_nav->isValidPolyRef(startRef))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		m_nodePool->clear();
		m_openList->clear();
		
		dtNode* startNode = m_nodePool->getNode(startRef);
		dtVcopy(startNode->pos, centerPos);
		startNode->pidx = 0;
		startNode->cost = 0;
		startNode->total = 0;
		startNode->id = startRef;
		startNode->flags = DT_NODE_OPEN;
		m_openList->push(startNode);
		
		dtStatus status = DT_SUCCESS;
		
		int n = 0;
		if (n < maxResult)
		{
			if (resultRef)
				resultRef[n] = startNode->id;
			if (resultParent)
				resultParent[n] = 0;
			if (resultCost)
				resultCost[n] = 0;
			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}
		
		const float radiusSqr = dtSqr(radius);
		
		while (!m_openList->empty())
		{
			dtNode* bestNode = m_openList->pop();
			bestNode->flags &= ~DT_NODE_OPEN;
			bestNode->flags |= DT_NODE_CLOSED;
			
			// Get poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtPolyRef bestRef = bestNode->id;
			const dtMeshTile* bestTile = 0;
			const dtPoly* bestPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
			
			// Get parent poly and tile.
			dtPolyRef parentRef = 0;
			const dtMeshTile* parentTile = 0;
			const dtPoly* parentPoly = 0;
			if (bestNode->pidx)
				parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
			if (parentRef)
				m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
			
			for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
			{
				const dtLink* link = &bestTile->links[i];
				dtPolyRef neighbourRef = link->ref;
				// Skip invalid neighbours and do not follow back to parent.
				if (!neighbourRef || neighbourRef == parentRef)
					continue;
				
				// Expand to neighbour
				const dtMeshTile* neighbourTile = 0;
				const dtPoly* neighbourPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
				// Do not advance if the polygon is excluded by the filter.
				if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;
				
				// Find edge and calc distance to the edge.
				float va[3], vb[3];
				if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
					continue;
				
				// If the circle is not touching the next polygon, skip it.
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
				if (distSqr > radiusSqr)
					continue;
				
				dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
				if (!neighbourNode)
				{
					status |= DT_OUT_OF_NODES;
					continue;
				}
					
				if (neighbourNode->flags & DT_NODE_CLOSED)
					continue;
				
				// Cost
				if (neighbourNode->flags == 0)
					dtVlerp(neighbourNode->pos, va, vb, 0.5f);
				
				const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
				
				// The node is already in open list and the new result is worse, skip.
				if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
					continue;
				
				neighbourNode->id = neighbourRef;
				neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
				neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
				neighbourNode->total = total;
				
				if (neighbourNode->flags & DT_NODE_OPEN)
				{
					m_openList->modify(neighbourNode);
				}
				else
				{
					if (n < maxResult)
					{
						if (resultRef)
							resultRef[n] = neighbourNode->id;
						if (resultParent)
							resultParent[n] = m_nodePool->getNodeAtIdx(neighbourNode->pidx)->id;
						if (resultCost)
							resultCost[n] = neighbourNode->total;
						++n;
					}
					else
					{
						status |= DT_BUFFER_TOO_SMALL;
					}
					neighbourNode->flags = DT_NODE_OPEN;
					m_openList->push(neighbourNode);
				}
			}
		}
		
		*resultCount = n;
		
		return status;
	}
	
	/// @par
	///
	/// The order of the result set is from least to highest cost.
	/// 
	/// At least one result array must be provided.
	///
	/// A common use case for this method is to perform Dijkstra searches. 
	/// Candidate polygons are found by searching the graph beginning at the start 
	/// polygon.
	/// 
	/// The same intersection test restrictions that apply to findPolysAroundCircle()
	/// method apply to this method.
	/// 
	/// The 3D centroid of the search polygon is used as the start position for cost 
	/// calculations.
	/// 
	/// Intersection tests occur in 2D. All polygons are projected onto the 
	/// xz-plane. So the y-values of the vertices do not effect intersection tests.
	/// 
	/// If the result arrays are is too small to hold the entire result set, they will 
	/// be filled to capacity.
	///
	dtStatus dtNavMeshQuery::findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
												  const dtQueryFilter* filter,
												  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
												  int* resultCount, const int maxResult) const
	{
		dtAssert(m_nav);
		dtAssert(m_nodePool);
		dtAssert(m_openList);
		
		*resultCount = 0;
		
		// Validate input
		if (!startRef || !m_nav->isValidPolyRef(startRef))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		m_nodePool->clear();
		m_openList->clear();
		
		float centerPos[3] = {0,0,0};
		for (int i = 0; i < nverts; ++i)
			dtVadd(centerPos,centerPos,&verts[i*3]);
		dtVscale(centerPos,centerPos,1.0f/nverts);
	
		dtNode* startNode = m_nodePool->getNode(startRef);
		dtVcopy(startNode->pos, centerPos);
		startNode->pidx = 0;
		startNode->cost = 0;
		startNode->total = 0;
		startNode->id = startRef;
		startNode->flags = DT_NODE_OPEN;
		m_openList->push(startNode);
		
		dtStatus status = DT_SUCCESS;
	
		int n = 0;
		if (n < maxResult)
		{
			if (resultRef)
				resultRef[n] = startNode->id;
			if (resultParent)
				resultParent[n] = 0;
			if (resultCost)
				resultCost[n] = 0;
			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}
		
		while (!m_openList->empty())
		{
			dtNode* bestNode = m_openList->pop();
			bestNode->flags &= ~DT_NODE_OPEN;
			bestNode->flags |= DT_NODE_CLOSED;
			
			// Get poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtPolyRef bestRef = bestNode->id;
			const dtMeshTile* bestTile = 0;
			const dtPoly* bestPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
			
			// Get parent poly and tile.
			dtPolyRef parentRef = 0;
			const dtMeshTile* parentTile = 0;
			const dtPoly* parentPoly = 0;
			if (bestNode->pidx)
				parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
			if (parentRef)
				m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
			
			for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
			{
				const dtLink* link = &bestTile->links[i];
				dtPolyRef neighbourRef = link->ref;
				// Skip invalid neighbours and do not follow back to parent.
				if (!neighbourRef || neighbourRef == parentRef)
					continue;
				
				// Expand to neighbour
				const dtMeshTile* neighbourTile = 0;
				const dtPoly* neighbourPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
				
				// Do not advance if the polygon is excluded by the filter.
				if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;
				
				// Find edge and calc distance to the edge.
				float va[3], vb[3];
				if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
					continue;
				
				// If the poly is not touching the edge to the next polygon, skip the connection it.
				float tmin, tmax;
				int segMin, segMax;
				if (!dtIntersectSegmentPoly2D(va, vb, verts, nverts, tmin, tmax, segMin, segMax))
					continue;
				if (tmin > 1.0f || tmax < 0.0f)
					continue;
				
				dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
				if (!neighbourNode)
				{
					status |= DT_OUT_OF_NODES;
					continue;
				}
				
				if (neighbourNode->flags & DT_NODE_CLOSED)
					continue;
				
				// Cost
				if (neighbourNode->flags == 0)
					dtVlerp(neighbourNode->pos, va, vb, 0.5f);
				
				const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
				
				// The node is already in open list and the new result is worse, skip.
				if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
					continue;
				
				neighbourNode->id = neighbourRef;
				neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
				neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
				neighbourNode->total = total;
				
				if (neighbourNode->flags & DT_NODE_OPEN)
				{
					m_openList->modify(neighbourNode);
				}
				else
				{
					if (n < maxResult)
					{
						if (resultRef)
							resultRef[n] = neighbourNode->id;
						if (resultParent)
							resultParent[n] = m_nodePool->getNodeAtIdx(neighbourNode->pidx)->id;
						if (resultCost)
							resultCost[n] = neighbourNode->total;
						++n;
					}
					else
					{
						status |= DT_BUFFER_TOO_SMALL;
					}
					neighbourNode->flags = DT_NODE_OPEN;
					m_openList->push(neighbourNode);
				}
			}
		}
		
		*resultCount = n;
		
		return status;
	}
	
	/// @par
	///
	/// This method is optimized for a small search radius and small number of result 
	/// polygons.
	///
	/// Candidate polygons are found by searching the navigation graph beginning at 
	/// the start polygon.
	///
	/// The same intersection test restrictions that apply to the findPolysAroundCircle 
	/// mehtod applies to this method.
	///
	/// The value of the center point is used as the start point for cost calculations. 
	/// It is not projected onto the surface of the mesh, so its y-value will effect 
	/// the costs.
	/// 
	/// Intersection tests occur in 2D. All polygons and the search circle are 
	/// projected onto the xz-plane. So the y-value of the center point does not 
	/// effect intersection tests.
	/// 
	/// If the result arrays are is too small to hold the entire result set, they will 
	/// be filled to capacity.
	/// 
	dtStatus dtNavMeshQuery::findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
													const dtQueryFilter* filter,
													dtPolyRef* resultRef, dtPolyRef* resultParent,
													int* resultCount, const int maxResult) const
	{
		dtAssert(m_nav);
		dtAssert(m_tinyNodePool);
		
		*resultCount = 0;
	
		// Validate input
		if (!startRef || !m_nav->isValidPolyRef(startRef))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		static const int MAX_STACK = 48;
		dtNode* stack[MAX_STACK];
		int nstack = 0;
		
		m_tinyNodePool->clear();
		
		dtNode* startNode = m_tinyNodePool->getNode(startRef);
		startNode->pidx = 0;
		startNode->id = startRef;
		startNode->flags = DT_NODE_CLOSED;
		stack[nstack++] = startNode;
		
		const float radiusSqr = dtSqr(radius);
		
		float pa[DT_VERTS_PER_POLYGON*3];
		float pb[DT_VERTS_PER_POLYGON*3];
		
		dtStatus status = DT_SUCCESS;
		
		int n = 0;
		if (n < maxResult)
		{
			resultRef[n] = startNode->id;
			if (resultParent)
				resultParent[n] = 0;
			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}
		
		while (nstack)
		{
			// Pop front.
			dtNode* curNode = stack[0];
			for (int i = 0; i < nstack-1; ++i)
				stack[i] = stack[i+1];
			nstack--;
			
			// Get poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtPolyRef curRef = curNode->id;
			const dtMeshTile* curTile = 0;
			const dtPoly* curPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);
			
			for (unsigned int i = curPoly->firstLink; i != DT_NULL_LINK; i = curTile->links[i].next)
			{
				const dtLink* link = &curTile->links[i];
				dtPolyRef neighbourRef = link->ref;
				// Skip invalid neighbours.
				if (!neighbourRef)
					continue;
				
				// Skip if cannot alloca more nodes.
				dtNode* neighbourNode = m_tinyNodePool->getNode(neighbourRef);
				if (!neighbourNode)
					continue;
				// Skip visited.
				if (neighbourNode->flags & DT_NODE_CLOSED)
					continue;
				
				// Expand to neighbour
				const dtMeshTile* neighbourTile = 0;
				const dtPoly* neighbourPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
				
				// Skip off-mesh connections.
				if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;
				
				// Do not advance if the polygon is excluded by the filter.
				if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;
				
				// Find edge and calc distance to the edge.
				float va[3], vb[3];
				if (!getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
					continue;
				
				// If the circle is not touching the next polygon, skip it.
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
				if (distSqr > radiusSqr)
					continue;
				
				// Mark node visited, this is done before the overlap test so that
				// we will not visit the poly again if the test fails.
				neighbourNode->flags |= DT_NODE_CLOSED;
				neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
				
				// Check that the polygon does not collide with existing polygons.
				
				// Collect vertices of the neighbour poly.
				const int npa = neighbourPoly->vertCount;
				for (int k = 0; k < npa; ++k)
					dtVcopy(&pa[k*3], &neighbourTile->verts[neighbourPoly->verts[k]*3]);
				
				bool overlap = false;
				for (int j = 0; j < n; ++j)
				{
					dtPolyRef pastRef = resultRef[j];
					
					// Connected polys do not overlap.
					bool connected = false;
					for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
					{
						if (curTile->links[k].ref == pastRef)
						{
							connected = true;
							break;
						}
					}
					if (connected)
						continue;
					
					// Potentially overlapping.
					const dtMeshTile* pastTile = 0;
					const dtPoly* pastPoly = 0;
					m_nav->getTileAndPolyByRefUnsafe(pastRef, &pastTile, &pastPoly);
					
					// Get vertices and test overlap
					const int npb = pastPoly->vertCount;
					for (int k = 0; k < npb; ++k)
						dtVcopy(&pb[k*3], &pastTile->verts[pastPoly->verts[k]*3]);
					
					if (dtOverlapPolyPoly2D(pa,npa, pb,npb))
					{
						overlap = true;
						break;
					}
				}
				if (overlap)
					continue;
				
				// This poly is fine, store and advance to the poly.
				if (n < maxResult)
				{
					resultRef[n] = neighbourRef;
					if (resultParent)
						resultParent[n] = curRef;
					++n;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
				
				if (nstack < MAX_STACK)
				{
					stack[nstack++] = neighbourNode;
				}
			}
		}
		
		*resultCount = n;
		
		return status;
	}
	
	
	struct dtSegInterval
	{
		dtPolyRef ref;
		short tmin, tmax;
	};
	
	static void insertInterval(dtSegInterval* ints, int& nints, const int maxInts,
							   const short tmin, const short tmax, const dtPolyRef ref)
	{
		if (nints+1 > maxInts) return;
		// Find insertion point.
		int idx = 0;
		while (idx < nints)
		{
			if (tmax <= ints[idx].tmin)
				break;
			idx++;
		}
		// Move current results.
		if (nints-idx)
			memmove(ints+idx+1, ints+idx, sizeof(dtSegInterval)*(nints-idx));
		// Store
		ints[idx].ref = ref;
		ints[idx].tmin = tmin;
		ints[idx].tmax = tmax;
		nints++;
	}
	
	/// @par
	///
	/// If the @p segmentRefs parameter is provided, then all polygon segments will be returned. 
	/// Otherwise only the wall segments are returned.
	/// 
	/// A segment that is normally a portal will be included in the result set as a 
	/// wall if the @p filter results in the neighbor polygon becoomming impassable.
	/// 
	/// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the 
	/// maximum segments per polygon of the source navigation mesh.
	/// 
	dtStatus dtNavMeshQuery::getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
												 float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
												 const int maxSegments) const
	{
		dtAssert(m_nav);
		
		*segmentCount = 0;
		
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		int n = 0;
		static const int MAX_INTERVAL = 16;
		dtSegInterval ints[MAX_INTERVAL];
		int nints;
		
		const bool storePortals = segmentRefs != 0;
		
		dtStatus status = DT_SUCCESS;
		
		for (int i = 0, j = (int)poly->vertCount-1; i < (int)poly->vertCount; j = i++)
		{
			// Skip non-solid edges.
			nints = 0;
			if (poly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
				{
					const dtLink* link = &tile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const dtMeshTile* neiTile = 0;
							const dtPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
							{
								insertInterval(ints, nints, MAX_INTERVAL, link->bmin, link->bmax, link->ref);
							}
						}
					}
				}
			}
			else
			{
				// Internal edge
				dtPolyRef neiRef = 0;
				if (poly->neis[j])
				{
					const unsigned int idx = (unsigned int)(poly->neis[j]-1);
					neiRef = m_nav->getPolyRefBase(tile) | idx;
					if (!filter->passFilter(neiRef, tile, &tile->polys[idx]))
						neiRef = 0;
				}
	
				// If the edge leads to another polygon and portals are not stored, skip.
				if (neiRef != 0 && !storePortals)
					continue;
				
				if (n < maxSegments)
				{
					const float* vj = &tile->verts[poly->verts[j]*3];
					const float* vi = &tile->verts[poly->verts[i]*3];
					float* seg = &segmentVerts[n*6];
					dtVcopy(seg+0, vj);
					dtVcopy(seg+3, vi);
					if (segmentRefs)
						segmentRefs[n] = neiRef;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
				
				continue;
			}
			
			// Add sentinels
			insertInterval(ints, nints, MAX_INTERVAL, -1, 0, 0);
			insertInterval(ints, nints, MAX_INTERVAL, 255, 256, 0);
			
			// Store segments.
			const float* vj = &tile->verts[poly->verts[j]*3];
			const float* vi = &tile->verts[poly->verts[i]*3];
			for (int k = 1; k < nints; ++k)
			{
				// Portal segment.
				if (storePortals && ints[k].ref)
				{
					const float tmin = ints[k].tmin/255.0f; 
					const float tmax = ints[k].tmax/255.0f; 
					if (n < maxSegments)
					{
						float* seg = &segmentVerts[n*6];
						dtVlerp(seg+0, vj,vi, tmin);
						dtVlerp(seg+3, vj,vi, tmax);
						if (segmentRefs)
							segmentRefs[n] = ints[k].ref;
						n++;
					}
					else
					{
						status |= DT_BUFFER_TOO_SMALL;
					}
				}
	
				// Wall segment.
				const int imin = ints[k-1].tmax;
				const int imax = ints[k].tmin;
				if (imin != imax)
				{
					const float tmin = imin/255.0f; 
					const float tmax = imax/255.0f; 
					if (n < maxSegments)
					{
						float* seg = &segmentVerts[n*6];
						dtVlerp(seg+0, vj,vi, tmin);
						dtVlerp(seg+3, vj,vi, tmax);
						if (segmentRefs)
							segmentRefs[n] = 0;
						n++;
					}
					else
					{
						status |= DT_BUFFER_TOO_SMALL;
					}
				}
			}
		}
		
		*segmentCount = n;
		
		return status;
	}
	
	/// @par
	///
	/// @p hitPos is not adjusted using the height detail data.
	///
	/// @p hitDist will equal the search radius if there is no wall within the 
	/// radius. In this case the values of @p hitPos and @p hitNormal are
	/// undefined.
	///
	/// The normal will become unpredicable if @p hitDist is a very small number.
	///
	dtStatus dtNavMeshQuery::findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
												const dtQueryFilter* filter,
												float* hitDist, float* hitPos, float* hitNormal) const
	{
		dtAssert(m_nav);
		dtAssert(m_nodePool);
		dtAssert(m_openList);
		
		// Validate input
		if (!startRef || !m_nav->isValidPolyRef(startRef))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		m_nodePool->clear();
		m_openList->clear();
		
		dtNode* startNode = m_nodePool->getNode(startRef);
		dtVcopy(startNode->pos, centerPos);
		startNode->pidx = 0;
		startNode->cost = 0;
		startNode->total = 0;
		startNode->id = startRef;
		startNode->flags = DT_NODE_OPEN;
		m_openList->push(startNode);
		
		float radiusSqr = dtSqr(maxRadius);
		
		dtStatus status = DT_SUCCESS;
		
		while (!m_openList->empty())
		{
			dtNode* bestNode = m_openList->pop();
			bestNode->flags &= ~DT_NODE_OPEN;
			bestNode->flags |= DT_NODE_CLOSED;
			
			// Get poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtPolyRef bestRef = bestNode->id;
			const dtMeshTile* bestTile = 0;
			const dtPoly* bestPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
			
			// Get parent poly and tile.
			dtPolyRef parentRef = 0;
			const dtMeshTile* parentTile = 0;
			const dtPoly* parentPoly = 0;
			if (bestNode->pidx)
				parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
			if (parentRef)
				m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
			
			// Hit test walls.
			for (int i = 0, j = (int)bestPoly->vertCount-1; i < (int)bestPoly->vertCount; j = i++)
			{
				// Skip non-solid edges.
				if (bestPoly->neis[j] & DT_EXT_LINK)
				{
					// Tile border.
					bool solid = true;
					for (unsigned int k = bestPoly->firstLink; k != DT_NULL_LINK; k = bestTile->links[k].next)
					{
						const dtLink* link = &bestTile->links[k];
						if (link->edge == j)
						{
							if (link->ref != 0)
							{
								const dtMeshTile* neiTile = 0;
								const dtPoly* neiPoly = 0;
								m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
								if (filter->passFilter(link->ref, neiTile, neiPoly))
									solid = false;
							}
							break;
						}
					}
					if (!solid) continue;
				}
				else if (bestPoly->neis[j])
				{
					// Internal edge
					const unsigned int idx = (unsigned int)(bestPoly->neis[j]-1);
					const dtPolyRef ref = m_nav->getPolyRefBase(bestTile) | idx;
					if (filter->passFilter(ref, bestTile, &bestTile->polys[idx]))
						continue;
				}
				
				// Calc distance to the edge.
				const float* vj = &bestTile->verts[bestPoly->verts[j]*3];
				const float* vi = &bestTile->verts[bestPoly->verts[i]*3];
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(centerPos, vj, vi, tseg);
				
				// Edge is too far, skip.
				if (distSqr > radiusSqr)
					continue;
				
				// Hit wall, update radius.
				radiusSqr = distSqr;
				// Calculate hit pos.
				hitPos[0] = vj[0] + (vi[0] - vj[0])*tseg;
				hitPos[1] = vj[1] + (vi[1] - vj[1])*tseg;
				hitPos[2] = vj[2] + (vi[2] - vj[2])*tseg;
			}
			
			for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
			{
				const dtLink* link = &bestTile->links[i];
				dtPolyRef neighbourRef = link->ref;
				// Skip invalid neighbours and do not follow back to parent.
				if (!neighbourRef || neighbourRef == parentRef)
					continue;
				
				// Expand to neighbour.
				const dtMeshTile* neighbourTile = 0;
				const dtPoly* neighbourPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
				
				// Skip off-mesh connections.
				if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;
				
				// Calc distance to the edge.
				const float* va = &bestTile->verts[bestPoly->verts[link->edge]*3];
				const float* vb = &bestTile->verts[bestPoly->verts[(link->edge+1) % bestPoly->vertCount]*3];
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
				
				// If the circle is not touching the next polygon, skip it.
				if (distSqr > radiusSqr)
					continue;
				
				if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
					continue;
	
				dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
				if (!neighbourNode)
				{
					status |= DT_OUT_OF_NODES;
					continue;
				}
				
				if (neighbourNode->flags & DT_NODE_CLOSED)
					continue;
				
				// Cost
				if (neighbourNode->flags == 0)
				{
					getEdgeMidPoint(bestRef, bestPoly, bestTile,
									neighbourRef, neighbourPoly, neighbourTile, neighbourNode->pos);
				}
				
				const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
				
				// The node is already in open list and the new result is worse, skip.
				if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
					continue;
				
				neighbourNode->id = neighbourRef;
				neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
				neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
				neighbourNode->total = total;
					
				if (neighbourNode->flags & DT_NODE_OPEN)
				{
					m_openList->modify(neighbourNode);
				}
				else
				{
					neighbourNode->flags |= DT_NODE_OPEN;
					m_openList->push(neighbourNode);
				}
			}
		}
		
		// Calc hit normal.
		dtVsub(hitNormal, centerPos, hitPos);
		dtVnormalize(hitNormal);
		
		*hitDist = dtMathSqrtf(radiusSqr);
		
		return status;
	}
	*/
	boolean isValidPolyRef(long ref, QueryFilter filter) {
		try {
			Tupple2<MeshTile,Poly> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
			// If cannot pass filter, assume flags has changed and boundary is invalid.
			if (filter.passFilter(ref, tileAndPoly.first, tileAndPoly.second))
				return true;
		} catch (IllegalArgumentException e) {
			// If cannot get polygon, assume it does not exists and boundary is invalid.
		}
		return false;
	}

	/*
	/// @par
	///
	/// The closed list is the list of polygons that were fully evaluated during 
	/// the last navigation graph search. (A* or Dijkstra)
	/// 
	bool dtNavMeshQuery::isInClosedList(dtPolyRef ref) const
	{
		if (!m_nodePool) return false;
		
		dtNode* nodes[DT_MAX_STATES_PER_NODE];
		int n= m_nodePool->findNodes(ref, nodes, DT_MAX_STATES_PER_NODE);
	
		for (int i=0; i<n; i++)
		{
			if (nodes[i]->flags & DT_NODE_CLOSED)
				return true;
		}		
	
		return false;
	}
	
	
	*/

}