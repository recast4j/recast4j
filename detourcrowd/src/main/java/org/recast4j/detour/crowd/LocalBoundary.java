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
package org.recast4j.detour.crowd;

import static org.recast4j.detour.DetourCommon.distancePtSegSqr2D;
import static org.recast4j.detour.DetourCommon.sqr;
import static org.recast4j.detour.DetourCommon.vCopy;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

import org.recast4j.detour.FindLocalNeighbourhoodResult;
import org.recast4j.detour.GetPolyWallSegmentsResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Tupple2;

public class LocalBoundary {

	private static final int MAX_LOCAL_SEGS = 8;

	private static class Segment {
		/** Segment start/end */
		float[] s = new float[6];
		/** Distance for pruning. */
		float d;
	}

	float[] m_center = new float[3];
	PriorityQueue<Segment> m_segs = new PriorityQueue<>((o1, o2) -> Float.compare(o1.d, o2.d));
	List<Long> m_polys = new ArrayList<>();

	protected LocalBoundary() {
		m_center[0] = m_center[1] = m_center[2] = Float.MAX_VALUE;
	}

	protected void reset() {
		m_center[0] = m_center[1] = m_center[2] = Float.MAX_VALUE;
		m_polys.clear();
		m_segs.clear();
	}

	protected void addSegment(float dist, float[] s) {
		// Insert neighbour based on the distance.
		if (m_segs.size() >= MAX_LOCAL_SEGS) {
			return;
		}
		Segment seg = new Segment();
		System.arraycopy(s, 0, seg.s, 0, 6);
		seg.d = dist;
		m_segs.add(seg);
	}

	public void update(long ref, float[] pos, float collisionQueryRange, NavMeshQuery navquery, QueryFilter filter) {
		if (ref == 0) {
			reset();
			return;
		}
		vCopy(m_center, pos);
		// First query non-overlapping polygons.
		FindLocalNeighbourhoodResult res = navquery.findLocalNeighbourhood(ref, pos, collisionQueryRange, filter);
		this.m_polys = res.getRefs();
		m_segs.clear();
		// Secondly, store all polygon edges.
		for (int j = 0; j < m_polys.size(); ++j) {
			GetPolyWallSegmentsResult gpws = navquery.getPolyWallSegments(m_polys.get(j), filter);
			for (int k = 0; k < gpws.getSegmentRefs().size(); ++k) {
				float[] s = gpws.getSegmentVerts().get(k);
				// Skip too distant segments.
				Tupple2<Float, Float> distseg = distancePtSegSqr2D(pos, s, 0, 3);
				if (distseg.first > sqr(collisionQueryRange))
					continue;
				addSegment(distseg.first, s);
			}
		}
	}

	public boolean isValid(NavMeshQuery navquery, QueryFilter filter) {
		if (m_polys.isEmpty())
			return false;

		// Check that all polygons still pass query filter.
		for (long ref : m_polys) {
			if (!navquery.isValidPolyRef(ref, filter))
				return false;
		}

		return true;
	}

}
