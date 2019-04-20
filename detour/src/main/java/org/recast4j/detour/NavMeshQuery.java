/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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

import static org.recast4j.detour.DetourCommon.clamp;
import static org.recast4j.detour.DetourCommon.distancePtPolyEdgesSqr;
import static org.recast4j.detour.DetourCommon.distancePtSegSqr2D;
import static org.recast4j.detour.DetourCommon.intersectSegSeg2D;
import static org.recast4j.detour.DetourCommon.intersectSegmentPoly2D;
import static org.recast4j.detour.DetourCommon.overlapBounds;
import static org.recast4j.detour.DetourCommon.overlapPolyPoly2D;
import static org.recast4j.detour.DetourCommon.overlapQuantBounds;
import static org.recast4j.detour.DetourCommon.pointInPolygon;
import static org.recast4j.detour.DetourCommon.randomPointInConvexPoly;
import static org.recast4j.detour.DetourCommon.sqr;
import static org.recast4j.detour.DetourCommon.triArea2D;
import static org.recast4j.detour.DetourCommon.vAdd;
import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vDist;
import static org.recast4j.detour.DetourCommon.vDistSqr;
import static org.recast4j.detour.DetourCommon.vEqual;
import static org.recast4j.detour.DetourCommon.vIsFinite;
import static org.recast4j.detour.DetourCommon.vIsFinite2D;
import static org.recast4j.detour.DetourCommon.vLenSqr;
import static org.recast4j.detour.DetourCommon.vLerp;
import static org.recast4j.detour.DetourCommon.vMad;
import static org.recast4j.detour.DetourCommon.vMax;
import static org.recast4j.detour.DetourCommon.vMin;
import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vSub;
import static org.recast4j.detour.Node.DT_NODE_CLOSED;
import static org.recast4j.detour.Node.DT_NODE_OPEN;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Random;

import org.recast4j.detour.DetourCommon.IntersectResult;

public class NavMeshQuery {

    /**
     * Use raycasts during pathfind to "shortcut" (raycast still consider costs) Options for
     * NavMeshQuery::initSlicedFindPath and updateSlicedFindPath
     */
    public static final int DT_FINDPATH_ANY_ANGLE = 0x02;

    /** Raycast should calculate movement cost along the ray and fill RaycastHit::cost */
    public static final int DT_RAYCAST_USE_COSTS = 0x01;

    /// Vertex flags returned by findStraightPath.
    /** The vertex is the start position in the path. */
    public static final int DT_STRAIGHTPATH_START = 0x01;
    /** The vertex is the end position in the path. */
    public static final int DT_STRAIGHTPATH_END = 0x02;
    /** The vertex is the start of an off-mesh connection. */
    public static final int DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04;

    /// Options for findStraightPath.
    public static final int DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01; /// < Add a vertex at every polygon edge crossing
                                                                   /// where area changes.
    public static final int DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02; /// < Add a vertex at every polygon edge crossing.

    static float H_SCALE = 0.999f; // Search heuristic scale.

    private final NavMesh m_nav;
    private final NodePool m_nodePool;
    private final NodePool m_tinyNodePool;
    private final NodeQueue m_openList;
    private QueryData m_query; /// < Sliced query state.

    public NavMeshQuery(NavMesh nav) {
        m_nav = nav;
        m_nodePool = new NodePool();
        m_tinyNodePool = new NodePool();
        m_openList = new NodeQueue();
    }

    public static class FRand {
        Random r = new Random();

        public float frand() {
            return r.nextFloat();
        }
    }

    /**
     * Returns random location on navmesh. Polygons are chosen weighted by area. The search runs in linear related to
     * number of polygon.
     *
     * @param filter
     *            The polygon filter to apply to the query.
     * @param frand
     *            Function returning a random number [0..1).
     * @return Random location
     */
    public Result<FindRandomPointResult> findRandomPoint(QueryFilter filter, FRand frand) {
        // Randomly pick one tile. Assume that all tiles cover roughly the same area.
        if (Objects.isNull(filter) || Objects.isNull(frand)) {
            return Result.invalidParam();
        }
        MeshTile tile = null;
        float tsum = 0.0f;
        for (int i = 0; i < m_nav.getMaxTiles(); i++) {
            MeshTile t = m_nav.getTile(i);
            if (t == null || t.data == null || t.data.header == null) {
                continue;
            }

            // Choose random tile using reservoi sampling.
            float area = 1.0f; // Could be tile area too.
            tsum += area;
            float u = frand.frand();
            if (u * tsum <= area) {
                tile = t;
            }
        }
        if (tile == null) {
            return Result.invalidParam("Tile not found");
        }

        // Randomly pick one polygon weighted by polygon area.
        Poly poly = null;
        long polyRef = 0;
        long base = m_nav.getPolyRefBase(tile);

        float areaSum = 0.0f;
        for (int i = 0; i < tile.data.header.polyCount; ++i) {
            Poly p = tile.data.polys[i];
            // Do not return off-mesh connection polygons.
            if (p.getType() != Poly.DT_POLYTYPE_GROUND) {
                continue;
            }
            // Must pass filter
            long ref = base | i;
            if (!filter.passFilter(ref, tile, p)) {
                continue;
            }

            // Calc area of the polygon.
            float polyArea = 0.0f;
            for (int j = 2; j < p.vertCount; ++j) {
                int va = p.verts[0] * 3;
                int vb = p.verts[j - 1] * 3;
                int vc = p.verts[j] * 3;
                polyArea += triArea2D(tile.data.verts, va, vb, vc);
            }

            // Choose random polygon weighted by area, using reservoi sampling.
            areaSum += polyArea;
            float u = frand.frand();
            if (u * areaSum <= polyArea) {
                poly = p;
                polyRef = ref;
            }
        }

        if (poly == null) {
            return Result.invalidParam("Poly not found");
        }

        // Randomly pick point on polygon.
        float[] verts = new float[3 * m_nav.getMaxVertsPerPoly()];
        float[] areas = new float[m_nav.getMaxVertsPerPoly()];
        System.arraycopy(tile.data.verts, poly.verts[0] * 3, verts, 0, 3);
        for (int j = 1; j < poly.vertCount; ++j) {
            System.arraycopy(tile.data.verts, poly.verts[j] * 3, verts, j * 3, 3);
        }

        float s = frand.frand();
        float t = frand.frand();

        float[] pt = randomPointInConvexPoly(verts, poly.vertCount, areas, s, t);
        FindRandomPointResult result = new FindRandomPointResult(polyRef, pt);
        Result<Float> pheight = getPolyHeight(polyRef, pt);
        if (pheight.failed()) {
            return Result.of(pheight.status, result);
        }
        pt[1] = pheight.result;
        return Result.success(result);
    }

    /**
     * Returns random location on navmesh within the reach of specified location. Polygons are chosen weighted by area.
     * The search runs in linear related to number of polygon. The location is not exactly constrained by the circle,
     * but it limits the visited polygons.
     *
     * @param startRef
     *            The reference id of the polygon where the search starts.
     * @param centerPos
     *            The center of the search circle. [(x, y, z)]
     * @param maxRadius
     * @param filter
     *            The polygon filter to apply to the query.
     * @param frand
     *            Function returning a random number [0..1).
     * @return Random location
     */
    public Result<FindRandomPointResult> findRandomPointAroundCircle(long startRef, float[] centerPos, float maxRadius,
            QueryFilter filter, FRand frand) {

        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !vIsFinite(centerPos) || maxRadius < 0
                || !Float.isFinite(maxRadius) || Objects.isNull(filter) || Objects.isNull(frand)) {
            return Result.invalidParam();
        }

        Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(startRef);
        MeshTile startTile = tileAndPoly.first;
        Poly startPoly = tileAndPoly.second;
        if (!filter.passFilter(startRef, startTile, startPoly)) {
            return Result.invalidParam("Invalid start ref");
        }

        m_nodePool.clear();
        m_openList.clear();

        Node startNode = m_nodePool.getNode(startRef);
        vCopy(startNode.pos, centerPos);
        startNode.pidx = 0;
        startNode.cost = 0;
        startNode.total = 0;
        startNode.id = startRef;
        startNode.flags = DT_NODE_OPEN;
        m_openList.push(startNode);

        float radiusSqr = maxRadius * maxRadius;
        float areaSum = 0.0f;

        MeshTile randomTile = null;
        Poly randomPoly = null;
        long randomPolyRef = 0;

        while (!m_openList.isEmpty()) {
            Node bestNode = m_openList.pop();
            bestNode.flags &= ~DT_NODE_OPEN;
            bestNode.flags |= DT_NODE_CLOSED;
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
                    polyArea += triArea2D(bestTile.data.verts, va, vb, vc);
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
            if (bestNode.pidx != 0) {
                parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
            }
            if (parentRef != 0) {
                Tupple2<MeshTile, Poly> parentTilePoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
                MeshTile parentTile = parentTilePoly.first;
                Poly parentPoly = parentTilePoly.second;
            }

            for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.ref;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour
                Tupple2<MeshTile, Poly> neighbourTilePoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = neighbourTilePoly.first;
                Poly neighbourPoly = neighbourTilePoly.second;

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> portalpoints = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef,
                        neighbourPoly, neighbourTile, 0, 0);
                if (portalpoints.failed()) {
                    continue;
                }
                float[] va = portalpoints.result.left;
                float[] vb = portalpoints.result.right;

                // If the circle is not touching the next polygon, skip it.
                Tupple2<Float, Float> distseg = distancePtSegSqr2D(centerPos, va, vb);
                float distSqr = distseg.first;
                if (distSqr > radiusSqr) {
                    continue;
                }

                Node neighbourNode = m_nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    neighbourNode.pos = vLerp(va, vb, 0.5f);
                }

                float total = bestNode.total + vDist(bestNode.pos, neighbourNode.pos);

                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total) {
                    continue;
                }

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

        if (randomPoly == null) {
            return Result.failure();
        }

        // Randomly pick point on polygon.
        float[] verts = new float[3 * m_nav.getMaxVertsPerPoly()];
        float[] areas = new float[m_nav.getMaxVertsPerPoly()];
        System.arraycopy(randomTile.data.verts, randomPoly.verts[0] * 3, verts, 0, 3);
        for (int j = 1; j < randomPoly.vertCount; ++j) {
            System.arraycopy(randomTile.data.verts, randomPoly.verts[j] * 3, verts, j * 3, 3);
        }

        float s = frand.frand();
        float t = frand.frand();

        float[] pt = randomPointInConvexPoly(verts, randomPoly.vertCount, areas, s, t);
        FindRandomPointResult result = new FindRandomPointResult(randomPolyRef, pt);
        Result<Float> pheight = getPolyHeight(randomPolyRef, pt);
        if (pheight.failed()) {
            return Result.of(pheight.status, result);
        }
        pt[1] = pheight.result;
        return Result.success(result);
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
    /// Finds the closest point on the specified polygon.
    /// @param[in] ref The reference id of the polygon.
    /// @param[in] pos The position to check. [(x, y, z)]
    /// @param[out] closest
    /// @param[out] posOverPoly
    /// @returns The status flags for the query.
    public Result<ClosestPointOnPolyResult> closestPointOnPoly(long ref, float[] pos) {
        if (!m_nav.isValidPolyRef(ref) || Objects.isNull(pos) || !vIsFinite(pos)) {
            return Result.invalidParam();
        }
        return Result.success(m_nav.closestPointOnPoly(ref, pos));
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
    /// Returns a point on the boundary closest to the source point if the source point is outside the
    /// polygon's xz-bounds.
    /// @param[in] ref The reference id to the polygon.
    /// @param[in] pos The position to check. [(x, y, z)]
    /// @param[out] closest The closest point. [(x, y, z)]
    /// @returns The status flags for the query.
    public Result<float[]> closestPointOnPolyBoundary(long ref, float[] pos) {

        Result<Tupple2<MeshTile, Poly>> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
        if (tileAndPoly.failed()) {
            return Result.of(tileAndPoly.status, tileAndPoly.message);
        }
        MeshTile tile = tileAndPoly.result.first;
        Poly poly = tileAndPoly.result.second;
        if (tile == null) {
            return Result.invalidParam("Invalid tile");
        }

        if (Objects.isNull(pos) || !vIsFinite(pos)) {
            return Result.invalidParam();
        }
        // Collect vertices.
        float[] verts = new float[m_nav.getMaxVertsPerPoly() * 3];
        float[] edged = new float[m_nav.getMaxVertsPerPoly()];
        float[] edget = new float[m_nav.getMaxVertsPerPoly()];
        int nv = poly.vertCount;
        for (int i = 0; i < nv; ++i) {
            System.arraycopy(tile.data.verts, poly.verts[i] * 3, verts, i * 3, 3);
        }

        float[] closest;
        if (distancePtPolyEdgesSqr(pos, verts, nv, edged, edget)) {
            closest = vCopy(pos);
        } else {
            // Point is outside the polygon, dtClamp to nearest edge.
            float dmin = edged[0];
            int imin = 0;
            for (int i = 1; i < nv; ++i) {
                if (edged[i] < dmin) {
                    dmin = edged[i];
                    imin = i;
                }
            }
            int va = imin * 3;
            int vb = ((imin + 1) % nv) * 3;
            closest = vLerp(verts, va, vb, edget[imin]);
        }
        return Result.success(closest);
    }

    /// @par
    ///
    /// Will return #DT_FAILURE if the provided position is outside the xz-bounds
    /// of the polygon.
    ///
    /// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
    /// @param[in] ref The reference id of the polygon.
    /// @param[in] pos A position within the xz-bounds of the polygon. [(x, y, z)]
    /// @param[out] height The height at the surface of the polygon.
    /// @returns The status flags for the query.
    public Result<Float> getPolyHeight(long ref, float[] pos) {

        Result<Tupple2<MeshTile, Poly>> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
        if (tileAndPoly.failed()) {
            return Result.of(tileAndPoly.status, tileAndPoly.message);
        }
        MeshTile tile = tileAndPoly.result.first;
        Poly poly = tileAndPoly.result.second;

        if (Objects.isNull(pos) || !vIsFinite2D(pos)) {
            return Result.invalidParam();
        }

        // We used to return success for offmesh connections, but the
        // getPolyHeight in DetourNavMesh does not do this, so special
        // case it here.
        if (poly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
            int i = poly.verts[0] * 3;
            float[] v0 = new float[] { tile.data.verts[i], tile.data.verts[i + 1], tile.data.verts[i + 2] };
            i = poly.verts[1] * 3;
            float[] v1 = new float[] { tile.data.verts[i], tile.data.verts[i + 1], tile.data.verts[i + 2] };
            Tupple2<Float, Float> dt = distancePtSegSqr2D(pos, v0, v1);
            return Result.success(v0[1] + (v1[1] - v0[1]) * dt.second);
        }
        Optional<Float> height = m_nav.getPolyHeight(tile, poly, pos);
        return height.isPresent() ? Result.success(height.get()) : Result.invalidParam();
    }

    /// @par
    ///
    /// @note If the search box does not intersect any polygons the search will
    /// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check
    /// @p nearestRef before using @p nearestPt.
    ///
    /// @}
    /// @name Local Query Functions
    /// @{

    /// Finds the polygon nearest to the specified center point.
    /// @param[in] center The center of the search box. [(x, y, z)]
    /// @param[in] extents The search distance along each axis. [(x, y, z)]
    /// @param[in] filter The polygon filter to apply to the query.
    /// @returns The status flags for the query.
    public Result<FindNearestPolyResult> findNearestPoly(float[] center, float[] halfExtents, QueryFilter filter) {

        float[] nearestPt = new float[] {center[0], center[1], center[2]};

        // Get nearby polygons from proximity grid.
        Result<List<Long>> polysResult = queryPolygons(center, halfExtents, filter);
        if (polysResult.failed()) {
            return Result.of(polysResult.status, polysResult.message);
        }
        List<Long> polys = polysResult.result;
        // Find nearest polygon amongst the nearby polygons.
        long nearest = 0;
        float nearestDistanceSqr = Float.MAX_VALUE;
        for (int i = 0; i < polys.size(); ++i) {
            long ref = polys.get(i);
            Result<ClosestPointOnPolyResult> closest = closestPointOnPoly(ref, center);
            boolean posOverPoly = closest.result.isPosOverPoly();
            float[] closestPtPoly = closest.result.getClosest();

            // If a point is directly over a polygon and closer than
            // climb height, favor that instead of straight line nearest point.
            float d = 0;
            float[] diff = vSub(center, closestPtPoly);
            if (posOverPoly) {
                Tupple2<MeshTile, Poly> tilaAndPoly = m_nav.getTileAndPolyByRefUnsafe(polys.get(i));
                MeshTile tile = tilaAndPoly.first;
                d = Math.abs(diff[1]) - tile.data.header.walkableClimb;
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

        return Result.success(new FindNearestPolyResult(nearest, nearestPt));
    }

    // FIXME: (PP) duplicate?
    protected List<Long> queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax, QueryFilter filter) {
        List<Long> polys = new ArrayList<>();
        if (tile.data.bvTree != null) {
            int nodeIndex = 0;
            float[] tbmin = tile.data.header.bmin;
            float[] tbmax = tile.data.header.bmax;
            float qfac = tile.data.header.bvQuantFactor;
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
            int end = tile.data.header.bvNodeCount;
            while (nodeIndex < end) {
                BVNode node = tile.data.bvTree[nodeIndex];
                boolean overlap = overlapQuantBounds(bmin, bmax, node.bmin, node.bmax);
                boolean isLeafNode = node.i >= 0;

                if (isLeafNode && overlap) {
                    long ref = base | node.i;
                    if (filter.passFilter(ref, tile, tile.data.polys[node.i])) {
                        polys.add(ref);
                    }
                }

                if (overlap || isLeafNode) {
                    nodeIndex++;
                } else {
                    int escapeIndex = -node.i;
                    nodeIndex += escapeIndex;
                }
            }
            return polys;
        } else {
            float[] bmin = new float[3];
            float[] bmax = new float[3];
            long base = m_nav.getPolyRefBase(tile);
            for (int i = 0; i < tile.data.header.polyCount; ++i) {
                Poly p = tile.data.polys[i];
                // Do not return off-mesh connection polygons.
                if (p.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }
                long ref = base | i;
                if (!filter.passFilter(ref, tile, p)) {
                    continue;
                }
                // Calc polygon bounds.
                int v = p.verts[0] * 3;
                vCopy(bmin, tile.data.verts, v);
                vCopy(bmax, tile.data.verts, v);
                for (int j = 1; j < p.vertCount; ++j) {
                    v = p.verts[j] * 3;
                    vMin(bmin, tile.data.verts, v);
                    vMax(bmax, tile.data.verts, v);
                }
                if (overlapBounds(qmin, qmax, bmin, bmax)) {
                    polys.add(ref);
                }
            }
            return polys;
        }
    }

    /**
     * Finds polygons that overlap the search box.
     *
     * If no polygons are found, the function will return with a polyCount of zero.
     *
     * @param center
     *            The center of the search box. [(x, y, z)]
     * @param halfExtents
     *            The search distance along each axis. [(x, y, z)]
     * @param filter
     *            The polygon filter to apply to the query.
     * @return The reference ids of the polygons that overlap the query box.
     */
    public Result<List<Long>> queryPolygons(float[] center, float[] halfExtents, QueryFilter filter) {
        if (Objects.isNull(center) || !vIsFinite(center) || Objects.isNull(halfExtents) || !vIsFinite(halfExtents)
                || Objects.isNull(filter)) {
            // return DT_FAILURE | DT_INVALID_PARAM;
        }
        float[] bmin = vSub(center, halfExtents);
        float[] bmax = vAdd(center, halfExtents);
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
                    List<Long> polysInTile = queryPolygonsInTile(neis.get(j), bmin, bmax, filter);
                    polys.addAll(polysInTile);
                }
            }
        }
        return Result.success(polys);
    }

    /**
     * Finds a path from the start polygon to the end polygon.
     *
     * If the end polygon cannot be reached through the navigation graph, the last polygon in the path will be the
     * nearest the end polygon.
     *
     * The start and end positions are used to calculate traversal costs. (The y-values impact the result.)
     *
     * @param startRef
     *            The refrence id of the start polygon.
     * @param endRef
     *            The reference id of the end polygon.
     * @param startPos
     *            A position within the start polygon. [(x, y, z)]
     * @param endPos
     *            A position within the end polygon. [(x, y, z)]
     * @param filter
     *            The polygon filter to apply to the query.
     * @return Found path
     */
    public Result<List<Long>> findPath(long startRef, long endRef, float[] startPos, float[] endPos,
            QueryFilter filter) {
        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef) || Objects.isNull(startPos)
                || !vIsFinite(startPos) || Objects.isNull(endPos) || !vIsFinite(endPos) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        if (startRef == endRef) {
            List<Long> path = new ArrayList<>(1);
            path.add(startRef);
            return Result.success(path);
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
            if (bestNode.pidx != 0) {
                parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
            }
            if (parentRef != 0) {
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.first;
                parentPoly = tileAndPoly.second;
            }

            for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
                long neighbourRef = bestTile.links.get(i).ref;

                // Skip invalid ids and do not expand back to where we came from.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Get neighbour poly and tile.
                // The API input has been cheked already, skip checking internal data.
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.first;
                Poly neighbourPoly = tileAndPoly.second;

                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // deal explicitly with crossing tile boundaries
                int crossSide = 0;
                if (bestTile.links.get(i).side != 0xff) {
                    crossSide = bestTile.links.get(i).side >> 1;
                }

                // get the node
                Node neighbourNode = m_nodePool.getNode(neighbourRef, crossSide);

                // If the node is visited the first time, calculate node position.
                if (neighbourNode.flags == 0) {
                    Result<float[]> midpod = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                            neighbourTile);
                    if (!midpod.failed()) {
                        neighbourNode.pos = midpod.result;
                    }
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
                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total) {
                    continue;
                }
                // The node is already visited and process, and the new result is worse, skip.
                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0 && total >= neighbourNode.total) {
                    continue;
                }

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

        List<Long> path = getPathToNode(lastBestNode);

        if (lastBestNode.id != endRef) {
            status = Status.PARTIAL_RESULT;
        }

        return Result.of(status, path);
    }

    /**
     * Intializes a sliced path query.
     *
     * Common use case: -# Call initSlicedFindPath() to initialize the sliced path query. -# Call updateSlicedFindPath()
     * until it returns complete. -# Call finalizeSlicedFindPath() to get the path.
     *
     * @param startRef
     *            The reference id of the start polygon.
     * @param endRef
     *            The reference id of the end polygon.
     * @param startPos
     *            A position within the start polygon. [(x, y, z)]
     * @param endPos
     *            A position within the end polygon. [(x, y, z)]
     * @param filter
     *            The polygon filter to apply to the query.
     * @param options
     *            query options (see: #FindPathOptions)
     * @return
     */
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

        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef) || Objects.isNull(startPos)
                || !vIsFinite(startPos) || Objects.isNull(endPos) || !vIsFinite(endPos) || Objects.isNull(filter)) {
            return Status.FAILURE_INVALID_PARAM;
        }

        // trade quality with performance?
        if ((options & DT_FINDPATH_ANY_ANGLE) != 0) {
            // limiting to several times the character radius yields nice results. It is not sensitive
            // so it is enough to compute it from the first tile.
            MeshTile tile = m_nav.getTileByRef(startRef);
            float agentRadius = tile.data.header.walkableRadius;
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

    /**
     * Updates an in-progress sliced path query.
     *
     * @param maxIter
     *            The maximum number of iterations to perform.
     * @return The status flags for the query.
     */
    public Result<Integer> updateSlicedFindPath(int maxIter) {
        if (!m_query.status.isInProgress()) {
            return Result.of(m_query.status, 0);
        }

        // Make sure the request is still valid.
        if (!m_nav.isValidPolyRef(m_query.startRef) || !m_nav.isValidPolyRef(m_query.endRef)) {
            m_query.status = Status.FAILURE;
            return Result.of(m_query.status, 0);
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
                return Result.of(m_query.status, iter);
            }

            // Get current poly and tile.
            // The API input has been cheked already, skip checking internal
            // data.
            long bestRef = bestNode.id;
            Result<Tupple2<MeshTile, Poly>> tileAndPoly = m_nav.getTileAndPolyByRef(bestRef);
            if (tileAndPoly.failed()) {
                m_query.status = Status.FAILURE;
                // The polygon has disappeared during the sliced query, fail.
                return Result.of(m_query.status, iter);
            }
            MeshTile bestTile = tileAndPoly.result.first;
            Poly bestPoly = tileAndPoly.result.second;
            // Get parent and grand parent poly and tile.
            long parentRef = 0, grandpaRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            Node parentNode = null;
            if (bestNode.pidx != 0) {
                parentNode = m_nodePool.getNodeAtIdx(bestNode.pidx);
                parentRef = parentNode.id;
                if (parentNode.pidx != 0) {
                    grandpaRef = m_nodePool.getNodeAtIdx(parentNode.pidx).id;
                }
            }
            if (parentRef != 0) {
                boolean invalidParent = false;
                tileAndPoly = m_nav.getTileAndPolyByRef(parentRef);
                invalidParent = tileAndPoly.failed();
                if (invalidParent || (grandpaRef != 0 && !m_nav.isValidPolyRef(grandpaRef))) {
                    // The polygon has disappeared during the sliced query,
                    // fail.
                    m_query.status = Status.FAILURE;
                    return Result.of(m_query.status, iter);
                }
                parentTile = tileAndPoly.result.first;
                parentPoly = tileAndPoly.result.second;
            }

            // decide whether to test raycast to previous nodes
            boolean tryLOS = false;
            if ((m_query.options & DT_FINDPATH_ANY_ANGLE) != 0) {
                if ((parentRef != 0) && (vDistSqr(parentNode.pos, bestNode.pos) < m_query.raycastLimitSqr)) {
                    tryLOS = true;
                }
            }

            for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
                long neighbourRef = bestTile.links.get(i).ref;

                // Skip invalid ids and do not expand back to where we came
                // from.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Get neighbour poly and tile.
                // The API input has been cheked already, skip checking internal
                // data.
                Tupple2<MeshTile, Poly> tileAndPolyUns = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPolyUns.first;
                Poly neighbourPoly = tileAndPolyUns.second;

                if (!m_query.filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // get the neighbor node
                Node neighbourNode = m_nodePool.getNode(neighbourRef, 0);

                // do not expand to nodes that were already visited from the
                // same parent
                if (neighbourNode.pidx != 0 && neighbourNode.pidx == bestNode.pidx) {
                    continue;
                }

                // If the node is visited the first time, calculate node
                // position.
                if (neighbourNode.flags == 0) {
                    Result<float[]> midpod = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                            neighbourTile);
                    if (!midpod.failed()) {
                        neighbourNode.pos = midpod.result;
                    }
                }

                // Calculate cost and heuristic.
                float cost = 0;
                float heuristic = 0;

                // raycast parent
                boolean foundShortCut = false;
                if (tryLOS) {
                    Result<RaycastHit> rayHit = raycast(parentRef, parentNode.pos, neighbourNode.pos, m_query.filter,
                            DT_RAYCAST_USE_COSTS, grandpaRef);
                    if (rayHit.succeeded()) {
                        foundShortCut = rayHit.result.t >= 1.0f;
                        if (foundShortCut) {
                            // shortcut found using raycast. Using shorter cost
                            // instead
                            cost = parentNode.cost + rayHit.result.pathCost;
                        }
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
                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total) {
                    continue;
                }
                // The node is already visited and process, and the new result
                // is worse, skip.
                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0 && total >= neighbourNode.total) {
                    continue;
                }

                // Add or update the node.
                neighbourNode.pidx = foundShortCut ? bestNode.pidx : m_nodePool.getNodeIdx(bestNode);
                neighbourNode.id = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~(Node.DT_NODE_CLOSED | Node.DT_NODE_PARENT_DETACHED));
                neighbourNode.cost = cost;
                neighbourNode.total = total;
                if (foundShortCut) {
                    neighbourNode.flags = (neighbourNode.flags | Node.DT_NODE_PARENT_DETACHED);
                }

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

        return Result.of(m_query.status, iter);
    }

    /// Finalizes and returns the results of a sliced path query.
    /// @param[out] path An ordered list of polygon references representing the path. (Start to end.)
    /// [(polyRef) * @p pathCount]
    /// @returns The status flags for the query.
    public Result<List<Long>> finalizeSlicedFindPath() {

        List<Long> path = new ArrayList<>(64);
        if (m_query.status.isFailed()) {
            // Reset query.
            m_query = new QueryData();
            return Result.failure(path);
        }

        if (m_query.startRef == m_query.endRef) {
            // Special case: the search starts and ends at same poly.
            path.add(m_query.startRef);
        } else {
            // Reverse the path.
            if (m_query.lastBestNode.id != m_query.endRef) {
                m_query.status = Status.PARTIAL_RESULT;
            }

            Node prev = null;
            Node node = m_query.lastBestNode;
            int prevRay = 0;
            do {
                Node next = m_nodePool.getNodeAtIdx(node.pidx);
                node.pidx = m_nodePool.getNodeIdx(prev);
                prev = node;
                int nextRay = node.flags & Node.DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent
                                                                         // (i.e. due to raycast shortcut)
                node.flags = (node.flags & ~Node.DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed
                                                                                     // path's node
                prevRay = nextRay;
                node = next;
            } while (node != null);

            // Store path
            node = prev;
            do {
                Node next = m_nodePool.getNodeAtIdx(node.pidx);
                if ((node.flags & Node.DT_NODE_PARENT_DETACHED) != 0) {
                    Result<RaycastHit> iresult = raycast(node.id, node.pos, next.pos, m_query.filter, 0, 0);
                    if (iresult.succeeded()) {
                        path.addAll(iresult.result.path);
                    }
                    // raycast ends on poly boundary and the path might include the next poly boundary.
                    if (path.get(path.size() - 1) == next.id) {
                        path.remove(path.size() - 1); // remove to avoid duplicates
                    }
                } else {
                    path.add(node.id);
                }

                node = next;
            } while (node != null);
        }

        Status status = m_query.status;
        // Reset query.
        m_query = new QueryData();

        return Result.of(status, path);
    }

    /// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
    /// polygon on the existing path that was visited during the search.
    /// @param[in] existing An array of polygon references for the existing path.
    /// @param[in] existingSize The number of polygon in the @p existing array.
    /// @param[out] path An ordered list of polygon references representing the path. (Start to end.)
    /// [(polyRef) * @p pathCount]
    /// @returns The status flags for the query.
    public Result<List<Long>> finalizeSlicedFindPathPartial(List<Long> existing) {

        List<Long> path = new ArrayList<>(64);
        if (Objects.isNull(existing) || existing.size() <= 0) {
            return Result.failure(path);
        }
        if (m_query.status.isFailed()) {
            // Reset query.
            m_query = new QueryData();
            return Result.failure(path);
        }
        if (m_query.startRef == m_query.endRef) {
            // Special case: the search starts and ends at same poly.
            path.add(m_query.startRef);
        } else {
            // Find furthest existing node that was visited.
            Node prev = null;
            Node node = null;
            for (int i = existing.size() - 1; i >= 0; --i) {
                node = m_nodePool.findNode(existing.get(i));
                if (node != null) {
                    break;
                }
            }

            if (node == null) {
                m_query.status = Status.PARTIAL_RESULT;
                node = m_query.lastBestNode;
            }

            // Reverse the path.
            int prevRay = 0;
            do {
                Node next = m_nodePool.getNodeAtIdx(node.pidx);
                node.pidx = m_nodePool.getNodeIdx(prev);
                prev = node;
                int nextRay = node.flags & Node.DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent
                                                                         // (i.e. due to raycast shortcut)
                node.flags = (node.flags & ~Node.DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed
                                                                                     // path's node
                prevRay = nextRay;
                node = next;
            } while (node != null);

            // Store path
            node = prev;
            do {
                Node next = m_nodePool.getNodeAtIdx(node.pidx);
                if ((node.flags & Node.DT_NODE_PARENT_DETACHED) != 0) {
                    Result<RaycastHit> iresult = raycast(node.id, node.pos, next.pos, m_query.filter, 0, 0);
                    if (iresult.succeeded()) {
                        path.addAll(iresult.result.path);
                    }
                    // raycast ends on poly boundary and the path might include the next poly boundary.
                    if (path.get(path.size() - 1) == next.id) {
                        path.remove(path.size() - 1); // remove to avoid duplicates
                    }
                } else {
                    path.add(node.id);
                }

                node = next;
            } while (node != null);
        }
        Status status = m_query.status;
        // Reset query.
        m_query = new QueryData();

        return Result.of(status, path);
    }

    protected Status appendVertex(float[] pos, int flags, long ref, List<StraightPathItem> straightPath,
            int maxStraightPath) {
        if (straightPath.size() > 0 && vEqual(straightPath.get(straightPath.size() - 1).pos, pos)) {
            // The vertices are equal, update flags and poly.
            straightPath.get(straightPath.size() - 1).flags = flags;
            straightPath.get(straightPath.size() - 1).ref = ref;
        } else {
            if (straightPath.size() < maxStraightPath) {
                // Append new vertex.
                straightPath.add(new StraightPathItem(pos, flags, ref));
            }
            // If reached end of path or there is no space to append more vertices, return.
            if (flags == DT_STRAIGHTPATH_END || straightPath.size() >= maxStraightPath) {
                return Status.SUCCSESS;
            }
        }
        return Status.IN_PROGRESS;
    }

    protected Status appendPortals(int startIdx, int endIdx, float[] endPos, List<Long> path,
            List<StraightPathItem> straightPath, int maxStraightPath, int options) {
        float[] startPos = straightPath.get(straightPath.size() - 1).pos;
        // Append or update last vertex
        Status stat = null;
        for (int i = startIdx; i < endIdx; i++) {
            // Calculate portal
            long from = path.get(i);
            Result<Tupple2<MeshTile, Poly>> tileAndPoly = m_nav.getTileAndPolyByRef(from);
            if (tileAndPoly.failed()) {
                return Status.FAILURE;
            }
            MeshTile fromTile = tileAndPoly.result.first;
            Poly fromPoly = tileAndPoly.result.second;

            long to = path.get(i + 1);
            tileAndPoly = m_nav.getTileAndPolyByRef(to);
            if (tileAndPoly.failed()) {
                return Status.FAILURE;
            }
            MeshTile toTile = tileAndPoly.result.first;
            Poly toPoly = tileAndPoly.result.second;

            Result<PortalResult> portals = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
            if (portals.failed()) {
                break;
            }
            float[] left = portals.result.left;
            float[] right = portals.result.right;

            if ((options & DT_STRAIGHTPATH_AREA_CROSSINGS) != 0) {
                // Skip intersection if only area crossings are requested.
                if (fromPoly.getArea() == toPoly.getArea()) {
                    continue;
                }
            }

            // Append intersection
            Optional<Tupple2<Float, Float>> interect = intersectSegSeg2D(startPos, endPos, left, right);
            if (interect.isPresent()) {
                float t = interect.get().second;
                float[] pt = vLerp(left, right, t);
                stat = appendVertex(pt, 0, path.get(i + 1), straightPath, maxStraightPath);
                if (!stat.isInProgress()) {
                    return stat;
                }
            }
        }
        return Status.IN_PROGRESS;
    }

    /// @par
    /// Finds the straight path from the start to the end position within the polygon corridor.
    ///
    /// This method peforms what is often called 'string pulling'.
    ///
    /// The start position is clamped to the first polygon in the path, and the
    /// end position is clamped to the last. So the start and end positions should
    /// normally be within or very near the first and last polygons respectively.
    ///
    /// The returned polygon references represent the reference id of the polygon
    /// that is entered at the associated path position. The reference id associated
    /// with the end point will always be zero. This allows, for example, matching
    /// off-mesh link points to their representative polygons.
    ///
    /// If the provided result buffers are too small for the entire result set,
    /// they will be filled as far as possible from the start toward the end
    /// position.
    ///
    /// @param[in] startPos Path start position. [(x, y, z)]
    /// @param[in] endPos Path end position. [(x, y, z)]
    /// @param[in] path An array of polygon references that represent the path corridor.
    /// @param[out] straightPath Points describing the straight path. [(x, y, z) * @p straightPathCount].
    /// @param[in] maxStraightPath The maximum number of points the straight path arrays can hold. [Limit: > 0]
    /// @param[in] options Query options. (see: #dtStraightPathOptions)
    /// @returns The status flags for the query.
    public Result<List<StraightPathItem>> findStraightPath(float[] startPos, float[] endPos, List<Long> path,
            int maxStraightPath, int options) {

        List<StraightPathItem> straightPath = new ArrayList<>();
        if (Objects.isNull(startPos) || !vIsFinite(startPos) || Objects.isNull(endPos) || !vIsFinite(endPos)
                || Objects.isNull(path) || path.isEmpty() || path.get(0) == 0 || maxStraightPath <= 0) {
            return Result.invalidParam();
        }
        // TODO: Should this be callers responsibility?
        Result<float[]> closestStartPosRes = closestPointOnPolyBoundary(path.get(0), startPos);
        if (closestStartPosRes.failed()) {
            return Result.invalidParam("Cannot find start position");
        }
        float[] closestStartPos = closestStartPosRes.result;
        Result<float[]> closestEndPosRes = closestPointOnPolyBoundary(path.get(path.size() - 1), endPos);
        if (closestEndPosRes.failed()) {
            return Result.invalidParam("Cannot find end position");
        }
        float[] closestEndPos = closestEndPosRes.result;
        // Add start point.
        Status stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path.get(0), straightPath, maxStraightPath);
        if (!stat.isInProgress()) {
            return Result.success(straightPath);
        }

        if (path.size() > 1) {
            float[] portalApex = vCopy(closestStartPos);
            float[] portalLeft = vCopy(portalApex);
            float[] portalRight = vCopy(portalApex);
            int apexIndex = 0;
            int leftIndex = 0;
            int rightIndex = 0;

            int leftPolyType = 0;
            int rightPolyType = 0;

            long leftPolyRef = path.get(0);
            long rightPolyRef = path.get(0);

            for (int i = 0; i < path.size(); ++i) {
                float[] left;
                float[] right;
                int toType;

                if (i + 1 < path.size()) {
                    // Next portal.
                    Result<PortalResult> portalPoints = getPortalPoints(path.get(i), path.get(i + 1));
                    left = portalPoints.result.left;
                    right = portalPoints.result.right;
                    toType = portalPoints.result.toType;
                    if (portalPoints.failed()) {
                        closestEndPosRes = closestPointOnPolyBoundary(path.get(i), endPos);
                        if (closestEndPosRes.failed()) {
                            return Result.invalidParam();
                        }
                        closestEndPos = closestEndPosRes.result;
                        // Append portals along the current straight path segment.
                        if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                            // Ignore status return value as we're just about to return anyway.
                            appendPortals(apexIndex, i, closestEndPos, path, straightPath, maxStraightPath, options);
                        }
                        // Ignore status return value as we're just about to return anyway.
                        appendVertex(closestEndPos, 0, path.get(i), straightPath, maxStraightPath);
                        return Result.success(straightPath);
                    }

                    // If starting really close the portal, advance.
                    if (i == 0) {
                        Tupple2<Float, Float> dt = distancePtSegSqr2D(portalApex, left, right);
                        if (dt.first < sqr(0.001f)) {
                            continue;
                        }
                    }
                } else {
                    // End of the path.
                    left = vCopy(closestEndPos);
                    right = vCopy(closestEndPos);
                    toType = Poly.DT_POLYTYPE_GROUND;
                }

                // Right vertex.
                if (triArea2D(portalApex, portalRight, right) <= 0.0f) {
                    if (vEqual(portalApex, portalRight) || triArea2D(portalApex, portalLeft, right) > 0.0f) {
                        portalRight = vCopy(right);
                        rightPolyRef = (i + 1 < path.size()) ? path.get(i + 1) : 0;
                        rightPolyType = toType;
                        rightIndex = i;
                    } else {
                        // Append portals along the current straight path segment.
                        if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                            stat = appendPortals(apexIndex, leftIndex, portalLeft, path, straightPath, maxStraightPath,
                                    options);
                            if (!stat.isInProgress()) {
                                return Result.success(straightPath);
                            }
                        }

                        portalApex = vCopy(portalLeft);
                        apexIndex = leftIndex;

                        int flags = 0;
                        if (leftPolyRef == 0) {
                            flags = DT_STRAIGHTPATH_END;
                        } else if (leftPolyType == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                            flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
                        }
                        long ref = leftPolyRef;

                        // Append or update vertex
                        stat = appendVertex(portalApex, flags, ref, straightPath, maxStraightPath);
                        if (!stat.isInProgress()) {
                            return Result.success(straightPath);
                        }

                        portalLeft = vCopy(portalApex);
                        portalRight = vCopy(portalApex);
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        // Restart
                        i = apexIndex;

                        continue;
                    }
                }

                // Left vertex.
                if (triArea2D(portalApex, portalLeft, left) >= 0.0f) {
                    if (vEqual(portalApex, portalLeft) || triArea2D(portalApex, portalRight, left) < 0.0f) {
                        portalLeft = vCopy(left);
                        leftPolyRef = (i + 1 < path.size()) ? path.get(i + 1) : 0;
                        leftPolyType = toType;
                        leftIndex = i;
                    } else {
                        // Append portals along the current straight path segment.
                        if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                            stat = appendPortals(apexIndex, rightIndex, portalRight, path, straightPath,
                                    maxStraightPath, options);
                            if (!stat.isInProgress()) {
                                return Result.success(straightPath);
                            }
                        }

                        portalApex = vCopy(portalRight);
                        apexIndex = rightIndex;

                        int flags = 0;
                        if (rightPolyRef == 0) {
                            flags = DT_STRAIGHTPATH_END;
                        } else if (rightPolyType == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                            flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
                        }
                        long ref = rightPolyRef;

                        // Append or update vertex
                        stat = appendVertex(portalApex, flags, ref, straightPath, maxStraightPath);
                        if (!stat.isInProgress()) {
                            return Result.success(straightPath);
                        }

                        portalLeft = vCopy(portalApex);
                        portalRight = vCopy(portalApex);
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;

                        // Restart
                        i = apexIndex;

                        continue;
                    }
                }
            }

            // Append portals along the current straight path segment.
            if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0) {
                stat = appendPortals(apexIndex, path.size() - 1, closestEndPos, path, straightPath, maxStraightPath,
                        options);
                if (!stat.isInProgress()) {
                    return Result.success(straightPath);
                }
            }
        }

        // Ignore status return value as we're just about to return anyway.
        appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0, straightPath, maxStraightPath);
        return Result.success(straightPath);
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
    /// Moves from the start to the end position constrained to the navigation mesh.
    /// @param[in] startRef The reference id of the start polygon.
    /// @param[in] startPos A position of the mover within the start polygon. [(x, y, x)]
    /// @param[in] endPos The desired end position of the mover. [(x, y, z)]
    /// @param[in] filter The polygon filter to apply to the query.
    /// @returns Path
    public Result<MoveAlongSurfaceResult> moveAlongSurface(long startRef, float[] startPos, float[] endPos,
            QueryFilter filter) {

        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(startPos) || !vIsFinite(startPos)
                || Objects.isNull(endPos) || !vIsFinite(endPos) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        m_tinyNodePool.clear();

        Node startNode = m_tinyNodePool.getNode(startRef);
        startNode.pidx = 0;
        startNode.cost = 0;
        startNode.total = 0;
        startNode.id = startRef;
        startNode.flags = Node.DT_NODE_CLOSED;
        LinkedList<Node> stack = new LinkedList<>();
        stack.add(startNode);

        float[] bestPos = new float[3];
        float bestDist = Float.MAX_VALUE;
        Node bestNode = null;
        vCopy(bestPos, startPos);

        // Search constraints
        float[] searchPos = vLerp(startPos, endPos, 0.5f);
        float searchRadSqr = sqr(vDist(startPos, endPos) / 2.0f + 0.001f);

        float[] verts = new float[m_nav.getMaxVertsPerPoly() * 3];

        while (!stack.isEmpty()) {
            // Pop front.
            Node curNode = stack.pop();

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long curRef = curNode.id;
            Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(curRef);
            MeshTile curTile = tileAndPoly.first;
            Poly curPoly = tileAndPoly.second;

            // Collect vertices.
            int nverts = curPoly.vertCount;
            for (int i = 0; i < nverts; ++i) {
                System.arraycopy(curTile.data.verts, curPoly.verts[i] * 3, verts, i * 3, 3);
            }

            // If target is inside the poly, stop search.
            if (pointInPolygon(endPos, verts, nverts)) {
                bestNode = curNode;
                vCopy(bestPos, endPos);
                break;
            }

            // Find wall edges and find nearest point inside the walls.
            for (int i = 0, j = curPoly.vertCount - 1; i < curPoly.vertCount; j = i++) {
                // Find links to neighbours.
                int MAX_NEIS = 8;
                int nneis = 0;
                long[] neis = new long[MAX_NEIS];

                if ((curPoly.neis[j] & NavMesh.DT_EXT_LINK) != 0) {
                    // Tile border.
                    for (int k = curPoly.firstLink; k != NavMesh.DT_NULL_LINK; k = curTile.links.get(k).next) {
                        Link link = curTile.links.get(k);
                        if (link.edge == j) {
                            if (link.ref != 0) {
                                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(link.ref);
                                MeshTile neiTile = tileAndPoly.first;
                                Poly neiPoly = tileAndPoly.second;
                                if (filter.passFilter(link.ref, neiTile, neiPoly)) {
                                    if (nneis < MAX_NEIS) {
                                        neis[nneis++] = link.ref;
                                    }
                                }
                            }
                        }
                    }
                } else if (curPoly.neis[j] != 0) {
                    int idx = curPoly.neis[j] - 1;
                    long ref = m_nav.getPolyRefBase(curTile) | idx;
                    if (filter.passFilter(ref, curTile, curTile.data.polys[idx])) {
                        // Internal edge, encode id.
                        neis[nneis++] = ref;
                    }
                }

                if (nneis == 0) {
                    // Wall edge, calc distance.
                    int vj = j * 3;
                    int vi = i * 3;
                    Tupple2<Float, Float> distSeg = distancePtSegSqr2D(endPos, verts, vj, vi);
                    float distSqr = distSeg.first;
                    float tseg = distSeg.second;
                    if (distSqr < bestDist) {
                        // Update nearest distance.
                        bestPos = vLerp(verts, vj, vi, tseg);
                        bestDist = distSqr;
                        bestNode = curNode;
                    }
                } else {
                    for (int k = 0; k < nneis; ++k) {
                        Node neighbourNode = m_tinyNodePool.getNode(neis[k]);
                        // Skip if already visited.
                        if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0) {
                            continue;
                        }

                        // Skip the link if it is too far from search constraint.
                        // TODO: Maybe should use getPortalPoints(), but this one is way faster.
                        int vj = j * 3;
                        int vi = i * 3;
                        Tupple2<Float, Float> distseg = distancePtSegSqr2D(searchPos, verts, vj, vi);
                        float distSqr = distseg.first;
                        if (distSqr > searchRadSqr) {
                            continue;
                        }

                        // Mark as the node as visited and push to queue.
                        neighbourNode.pidx = m_tinyNodePool.getNodeIdx(curNode);
                        neighbourNode.flags |= Node.DT_NODE_CLOSED;
                        stack.add(neighbourNode);
                    }
                }
            }
        }

        List<Long> visited = new ArrayList<>();
        if (bestNode != null) {
            // Reverse the path.
            Node prev = null;
            Node node = bestNode;
            do {
                Node next = m_tinyNodePool.getNodeAtIdx(node.pidx);
                node.pidx = m_tinyNodePool.getNodeIdx(prev);
                prev = node;
                node = next;
            } while (node != null);

            // Store result
            node = prev;
            do {
                visited.add(node.id);
                node = m_tinyNodePool.getNodeAtIdx(node.pidx);
            } while (node != null);
        }
        return Result.success(new MoveAlongSurfaceResult(bestPos, visited));
    }

    static class PortalResult {
        final float[] left;
        final float[] right;
        final int fromType;
        final int toType;

        public PortalResult(float[] left, float[] right, int fromType, int toType) {
            this.left = left;
            this.right = right;
            this.fromType = fromType;
            this.toType = toType;
        }

    }

    protected Result<PortalResult> getPortalPoints(long from, long to) {
        Result<Tupple2<MeshTile, Poly>> tileAndPolyResult = m_nav.getTileAndPolyByRef(from);
        if (tileAndPolyResult.failed()) {
            return Result.of(tileAndPolyResult.status, tileAndPolyResult.message);
        }
        Tupple2<MeshTile, Poly> tileAndPoly = tileAndPolyResult.result;
        MeshTile fromTile = tileAndPoly.first;
        Poly fromPoly = tileAndPoly.second;
        int fromType = fromPoly.getType();

        tileAndPolyResult = m_nav.getTileAndPolyByRef(to);
        if (tileAndPolyResult.failed()) {
            return Result.of(tileAndPolyResult.status, tileAndPolyResult.message);
        }
        tileAndPoly = tileAndPolyResult.result;
        MeshTile toTile = tileAndPoly.first;
        Poly toPoly = tileAndPoly.second;
        int toType = toPoly.getType();

        return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, fromType, toType);
    }

    // Returns portal points between two polygons.
    protected Result<PortalResult> getPortalPoints(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly,
            MeshTile toTile, int fromType, int toType) {
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
        if (link == null) {
            return Result.invalidParam("No link found");
        }

        // Handle off-mesh connections.
        if (fromPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
            // Find link that points to first vertex.
            for (int i = fromPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = fromTile.links.get(i).next) {
                if (fromTile.links.get(i).ref == to) {
                    int v = fromTile.links.get(i).edge;
                    System.arraycopy(fromTile.data.verts, fromPoly.verts[v] * 3, left, 0, 3);
                    System.arraycopy(fromTile.data.verts, fromPoly.verts[v] * 3, right, 0, 3);
                    return Result.success(new PortalResult(left, right, fromType, toType));
                }
            }
            return Result.invalidParam("Invalid offmesh from connection");
        }

        if (toPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
            for (int i = toPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = toTile.links.get(i).next) {
                if (toTile.links.get(i).ref == from) {
                    int v = toTile.links.get(i).edge;
                    System.arraycopy(toTile.data.verts, toPoly.verts[v] * 3, left, 0, 3);
                    System.arraycopy(toTile.data.verts, toPoly.verts[v] * 3, right, 0, 3);
                    return Result.success(new PortalResult(left, right, fromType, toType));
                }
            }
            return Result.invalidParam("Invalid offmesh to connection");
        }

        // Find portal vertices.
        int v0 = fromPoly.verts[link.edge];
        int v1 = fromPoly.verts[(link.edge + 1) % fromPoly.vertCount];
        System.arraycopy(fromTile.data.verts, v0 * 3, left, 0, 3);
        System.arraycopy(fromTile.data.verts, v1 * 3, right, 0, 3);

        // If the link is at tile boundary, dtClamp the vertices to
        // the link width.
        if (link.side != 0xff) {
            // Unpack portal limits.
            if (link.bmin != 0 || link.bmax != 255) {
                float s = 1.0f / 255.0f;
                float tmin = link.bmin * s;
                float tmax = link.bmax * s;
                left = vLerp(fromTile.data.verts, v0 * 3, v1 * 3, tmin);
                right = vLerp(fromTile.data.verts, v0 * 3, v1 * 3, tmax);
            }
        }

        return Result.success(new PortalResult(left, right, fromType, toType));
    }

    // Returns edge mid point between two polygons.
    protected Result<float[]> getEdgeMidPoint(long from, long to) {
        Result<PortalResult> ppoints = getPortalPoints(from, to);
        if (ppoints.failed()) {
            return Result.of(ppoints.status, ppoints.message);
        }
        float[] left = ppoints.result.left;
        float[] right = ppoints.result.right;
        float[] mid = new float[3];
        mid[0] = (left[0] + right[0]) * 0.5f;
        mid[1] = (left[1] + right[1]) * 0.5f;
        mid[2] = (left[2] + right[2]) * 0.5f;
        return Result.success(mid);
    }

    protected Result<float[]> getEdgeMidPoint(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly,
            MeshTile toTile) {
        Result<PortalResult> ppoints = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
        if (ppoints.failed()) {
            return Result.of(ppoints.status, ppoints.message);
        }
        float[] left = ppoints.result.left;
        float[] right = ppoints.result.right;
        float[] mid = new float[3];
        mid[0] = (left[0] + right[0]) * 0.5f;
        mid[1] = (left[1] + right[1]) * 0.5f;
        mid[2] = (left[2] + right[2]) * 0.5f;
        return Result.success(mid);
    }

    private static float s = 1.0f / 255.0f;

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
    /// Casts a 'walkability' ray along the surface of the navigation mesh from
    /// the start position toward the end position.
    /// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
    /// @param[in] startRef The reference id of the start polygon.
    /// @param[in] startPos A position within the start polygon representing
    /// the start of the ray. [(x, y, z)]
    /// @param[in] endPos The position to cast the ray toward. [(x, y, z)]
    /// @param[out] t The hit parameter. (FLT_MAX if no wall hit.)
    /// @param[out] hitNormal The normal of the nearest wall hit. [(x, y, z)]
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] path The reference ids of the visited polygons. [opt]
    /// @param[out] pathCount The number of visited polygons. [opt]
    /// @param[in] maxPath The maximum number of polygons the @p path array can hold.
    /// @returns The status flags for the query.
    public Result<RaycastHit> raycast(long startRef, float[] startPos, float[] endPos, QueryFilter filter, int options,
            long prevRef) {
        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(startPos) || !vIsFinite(startPos)
                || Objects.isNull(endPos) || !vIsFinite(endPos) || Objects.isNull(filter)
                || (prevRef != 0 && !m_nav.isValidPolyRef(prevRef))) {
            return Result.invalidParam();
        }

        RaycastHit hit = new RaycastHit();

        float[] verts = new float[m_nav.getMaxVertsPerPoly() * 3 + 3];

        float[] curPos = new float[3], lastPos = new float[3];

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
            for (int i = 0; i < poly.vertCount; ++i) {
                System.arraycopy(tile.data.verts, poly.verts[i] * 3, verts, nv * 3, 3);
                nv++;
            }

            IntersectResult iresult = intersectSegmentPoly2D(startPos, endPos, verts, nv);
            if (!iresult.intersects) {
                // Could not hit the polygon, keep the old t and report hit.
                return Result.success(hit);
            }

            hit.hitEdgeIndex = iresult.segMax;

            // Keep track of furthest t so far.
            if (iresult.tmax > hit.t) {
                hit.t = iresult.tmax;
            }

            // Store visited polygons.
            hit.path.add(curRef);

            // Ray end is completely inside the polygon.
            if (iresult.segMax == -1) {
                hit.t = Float.MAX_VALUE;

                // add the cost
                if ((options & DT_RAYCAST_USE_COSTS) != 0) {
                    hit.pathCost += filter.getCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly,
                            curRef, tile, poly);
                }
                return Result.success(hit);
            }

            // Follow neighbours.
            long nextRef = 0;

            for (int i = poly.firstLink; i != NavMesh.DT_NULL_LINK; i = tile.links.get(i).next) {
                Link link = tile.links.get(i);

                // Find link which contains this edge.
                if (link.edge != iresult.segMax) {
                    continue;
                }

                // Get pointer to the next polygon.
                tileAndPolyUns = m_nav.getTileAndPolyByRefUnsafe(link.ref);
                nextTile = tileAndPolyUns.first;
                nextPoly = tileAndPolyUns.second;
                // Skip off-mesh connections.
                if (nextPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Skip links based on filter.
                if (!filter.passFilter(link.ref, nextTile, nextPoly)) {
                    continue;
                }

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
                    float lmin = tile.data.verts[left + 2]
                            + (tile.data.verts[right + 2] - tile.data.verts[left + 2]) * (link.bmin * s);
                    float lmax = tile.data.verts[left + 2]
                            + (tile.data.verts[right + 2] - tile.data.verts[left + 2]) * (link.bmax * s);
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
                    float lmin = tile.data.verts[left]
                            + (tile.data.verts[right] - tile.data.verts[left]) * (link.bmin * s);
                    float lmax = tile.data.verts[left]
                            + (tile.data.verts[right] - tile.data.verts[left]) * (link.bmax * s);
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
                float[] diff = vSub(new VectorPtr(curPos), e1);
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
                vNormalize(hit.hitNormal);
                return Result.success(hit);
            }

            // No hit, advance to neighbour polygon.
            prevRef = curRef;
            curRef = nextRef;
            prevTile = tile;
            tile = nextTile;
            prevPoly = poly;
            poly = nextPoly;
        }

        return Result.success(hit);
    }

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
    /// @}
    /// @name Dijkstra Search Functions
    /// @{

    /// Finds the polygons along the navigation graph that touch the specified circle.
    /// @param[in] startRef The reference id of the polygon where the search starts.
    /// @param[in] centerPos The center of the search circle. [(x, y, z)]
    /// @param[in] radius The radius of the search circle.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] resultRef The reference ids of the polygons touched by the circle. [opt]
    /// @param[out] resultParent The reference ids of the parent polygons for each result.
    /// Zero if a result polygon has no parent. [opt]
    /// @param[out] resultCost The search cost from @p centerPos to the polygon. [opt]
    /// @param[out] resultCount The number of polygons found. [opt]
    /// @param[in] maxResult The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    public Result<FindPolysAroundResult> findPolysAroundCircle(long startRef, float[] centerPos, float radius,
            QueryFilter filter) {

        // Validate input

        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !vIsFinite(centerPos) || radius < 0
                || !Float.isFinite(radius) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        List<Long> resultRef = new ArrayList<>();
        List<Long> resultParent = new ArrayList<>();
        List<Float> resultCost = new ArrayList<>();

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

        float radiusSqr = sqr(radius);

        while (!m_openList.isEmpty()) {
            Node bestNode = m_openList.pop();
            bestNode.flags &= ~Node.DT_NODE_OPEN;
            bestNode.flags |= Node.DT_NODE_CLOSED;

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.id;
            Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.first;
            Poly bestPoly = tileAndPoly.second;

            // Get parent poly and tile.
            long parentRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            if (bestNode.pidx != 0) {
                parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
            }
            if (parentRef != 0) {
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.first;
                parentPoly = tileAndPoly.second;
            }

            resultRef.add(bestRef);
            resultParent.add(parentRef);
            resultCost.add(bestNode.total);

            for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.ref;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.first;
                Poly neighbourPoly = tileAndPoly.second;

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> pp = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                        neighbourTile, 0, 0);
                if (pp.failed()) {
                    continue;
                }
                float[] va = pp.result.left;
                float[] vb = pp.result.right;

                // If the circle is not touching the next polygon, skip it.
                Tupple2<Float, Float> distseg = distancePtSegSqr2D(centerPos, va, vb);
                float distSqr = distseg.first;
                if (distSqr > radiusSqr) {
                    continue;
                }

                Node neighbourNode = m_nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    neighbourNode.pos = vLerp(va, vb, 0.5f);
                }

                float cost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly, bestRef,
                        bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);

                float total = bestNode.total + cost;
                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total) {
                    continue;
                }

                neighbourNode.id = neighbourRef;
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

        return Result.success(new FindPolysAroundResult(resultRef, resultParent, resultCost));
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
    /// Finds the polygons along the naviation graph that touch the specified convex polygon.
    /// @param[in] startRef The reference id of the polygon where the search starts.
    /// @param[in] verts The vertices describing the convex polygon. (CCW)
    /// [(x, y, z) * @p nverts]
    /// @param[in] nverts The number of vertices in the polygon.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] resultRef The reference ids of the polygons touched by the search polygon. [opt]
    /// @param[out] resultParent The reference ids of the parent polygons for each result. Zero if a
    /// result polygon has no parent. [opt]
    /// @param[out] resultCost The search cost from the centroid point to the polygon. [opt]
    /// @param[out] resultCount The number of polygons found.
    /// @param[in] maxResult The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    public Result<FindPolysAroundResult> findPolysAroundShape(long startRef, float[] verts, int nverts,
            QueryFilter filter) {
        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(verts) || nverts < 3 || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        List<Long> resultRef = new ArrayList<>();
        List<Long> resultParent = new ArrayList<>();
        List<Float> resultCost = new ArrayList<>();

        m_nodePool.clear();
        m_openList.clear();

        float[] centerPos = new float[] { 0, 0, 0 };
        for (int i = 0; i < nverts; ++i) {
            centerPos[0] += verts[i * 3];
            centerPos[1] += verts[i * 3 + 1];
            centerPos[2] += verts[i * 3 + 2];
        }
        float scale = 1.0f / nverts;
        centerPos[0] *= scale;
        centerPos[1] *= scale;
        centerPos[2] *= scale;

        Node startNode = m_nodePool.getNode(startRef);
        vCopy(startNode.pos, centerPos);
        startNode.pidx = 0;
        startNode.cost = 0;
        startNode.total = 0;
        startNode.id = startRef;
        startNode.flags = Node.DT_NODE_OPEN;
        m_openList.push(startNode);

        while (!m_openList.isEmpty()) {
            Node bestNode = m_openList.pop();
            bestNode.flags &= ~Node.DT_NODE_OPEN;
            bestNode.flags |= Node.DT_NODE_CLOSED;

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.id;
            Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.first;
            Poly bestPoly = tileAndPoly.second;

            // Get parent poly and tile.
            long parentRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            if (bestNode.pidx != 0) {
                parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
            }
            if (parentRef != 0) {
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.first;
                parentPoly = tileAndPoly.second;
            }

            resultRef.add(bestRef);
            resultParent.add(parentRef);
            resultCost.add(bestNode.total);

            for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.ref;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.first;
                Poly neighbourPoly = tileAndPoly.second;

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> pp = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                        neighbourTile, 0, 0);
                if (pp.failed()) {
                    continue;
                }
                float[] va = pp.result.left;
                float[] vb = pp.result.right;

                // If the poly is not touching the edge to the next polygon, skip the connection it.
                IntersectResult ir = intersectSegmentPoly2D(va, vb, verts, nverts);
                if (!ir.intersects) {
                    continue;
                }
                if (ir.tmin > 1.0f || ir.tmax < 0.0f) {
                    continue;
                }

                Node neighbourNode = m_nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    neighbourNode.pos = vLerp(va, vb, 0.5f);
                }

                float cost = filter.getCost(bestNode.pos, neighbourNode.pos, parentRef, parentTile, parentPoly, bestRef,
                        bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);

                float total = bestNode.total + cost;

                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total) {
                    continue;
                }

                neighbourNode.id = neighbourRef;
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

        return Result.success(new FindPolysAroundResult(resultRef, resultParent, resultCost));
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
    /// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
    /// @param[in] startRef The reference id of the polygon where the search starts.
    /// @param[in] centerPos The center of the query circle. [(x, y, z)]
    /// @param[in] radius The radius of the query circle.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] resultRef The reference ids of the polygons touched by the circle.
    /// @param[out] resultParent The reference ids of the parent polygons for each result.
    /// Zero if a result polygon has no parent. [opt]
    /// @param[out] resultCount The number of polygons found.
    /// @param[in] maxResult The maximum number of polygons the result arrays can hold.
    /// @returns The status flags for the query.
    public Result<FindLocalNeighbourhoodResult> findLocalNeighbourhood(long startRef, float[] centerPos, float radius,
            QueryFilter filter) {

        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !vIsFinite(centerPos) || radius < 0
                || !Float.isFinite(radius) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

        List<Long> resultRef = new ArrayList<>();
        List<Long> resultParent = new ArrayList<>();

        m_tinyNodePool.clear();

        Node startNode = m_tinyNodePool.getNode(startRef);
        startNode.pidx = 0;
        startNode.id = startRef;
        startNode.flags = Node.DT_NODE_CLOSED;
        LinkedList<Node> stack = new LinkedList<>();
        stack.add(startNode);

        resultRef.add(startNode.id);
        resultParent.add(0L);

        float radiusSqr = sqr(radius);

        float[] pa = new float[m_nav.getMaxVertsPerPoly() * 3];
        float[] pb = new float[m_nav.getMaxVertsPerPoly() * 3];

        while (!stack.isEmpty()) {
            // Pop front.
            Node curNode = stack.pop();

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long curRef = curNode.id;
            Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(curRef);
            MeshTile curTile = tileAndPoly.first;
            Poly curPoly = tileAndPoly.second;

            for (int i = curPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = curTile.links.get(i).next) {
                Link link = curTile.links.get(i);
                long neighbourRef = link.ref;
                // Skip invalid neighbours.
                if (neighbourRef == 0) {
                    continue;
                }

                Node neighbourNode = m_tinyNodePool.getNode(neighbourRef);
                // Skip visited.
                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0) {
                    continue;
                }

                // Expand to neighbour
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.first;
                Poly neighbourPoly = tileAndPoly.second;

                // Skip off-mesh connections.
                if (neighbourPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Do not advance if the polygon is excluded by the filter.
                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                // Find edge and calc distance to the edge.
                Result<PortalResult> pp = getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly,
                        neighbourTile, 0, 0);
                if (pp.failed()) {
                    continue;
                }
                float[] va = pp.result.left;
                float[] vb = pp.result.right;

                // If the circle is not touching the next polygon, skip it.
                Tupple2<Float, Float> distseg = distancePtSegSqr2D(centerPos, va, vb);
                float distSqr = distseg.first;
                if (distSqr > radiusSqr) {
                    continue;
                }

                // Mark node visited, this is done before the overlap test so that
                // we will not visit the poly again if the test fails.
                neighbourNode.flags |= Node.DT_NODE_CLOSED;
                neighbourNode.pidx = m_tinyNodePool.getNodeIdx(curNode);

                // Check that the polygon does not collide with existing polygons.

                // Collect vertices of the neighbour poly.
                int npa = neighbourPoly.vertCount;
                for (int k = 0; k < npa; ++k) {
                    System.arraycopy(neighbourTile.data.verts, neighbourPoly.verts[k] * 3, pa, k * 3, 3);
                }

                boolean overlap = false;
                for (int j = 0; j < resultRef.size(); ++j) {
                    long pastRef = resultRef.get(j);

                    // Connected polys do not overlap.
                    boolean connected = false;
                    for (int k = curPoly.firstLink; k != NavMesh.DT_NULL_LINK; k = curTile.links.get(k).next) {
                        if (curTile.links.get(k).ref == pastRef) {
                            connected = true;
                            break;
                        }
                    }
                    if (connected) {
                        continue;
                    }

                    // Potentially overlapping.
                    tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(pastRef);
                    MeshTile pastTile = tileAndPoly.first;
                    Poly pastPoly = tileAndPoly.second;

                    // Get vertices and test overlap
                    int npb = pastPoly.vertCount;
                    for (int k = 0; k < npb; ++k) {
                        System.arraycopy(pastTile.data.verts, pastPoly.verts[k] * 3, pb, k * 3, 3);
                    }

                    if (overlapPolyPoly2D(pa, npa, pb, npb)) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) {
                    continue;
                }

                resultRef.add(neighbourRef);
                resultParent.add(curRef);
                stack.add(neighbourNode);
            }
        }

        return Result.success(new FindLocalNeighbourhoodResult(resultRef, resultParent));
    }

    private static class SegInterval {
        long ref;
        int tmin, tmax;

        public SegInterval(long ref, int tmin, int tmax) {
            this.ref = ref;
            this.tmin = tmin;
            this.tmax = tmax;
        }

    };

    protected void insertInterval(List<SegInterval> ints, int tmin, int tmax, long ref) {
        // Find insertion point.
        int idx = 0;
        while (idx < ints.size()) {
            if (tmax <= ints.get(idx).tmin) {
                break;
            }
            idx++;
        }
        // Store
        ints.add(idx, new SegInterval(ref, tmin, tmax));
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
    /// Returns the segments for the specified polygon, optionally including portals.
    /// @param[in] ref The reference id of the polygon.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] segmentVerts The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
    /// @param[out] segmentRefs The reference ids of each segment's neighbor polygon.
    /// Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount]
    /// @param[out] segmentCount The number of segments returned.
    /// @param[in] maxSegments The maximum number of segments the result arrays can hold.
    /// @returns The status flags for the query.
    public Result<GetPolyWallSegmentsResult> getPolyWallSegments(long ref, boolean storePortals, QueryFilter filter) {
        Result<Tupple2<MeshTile, Poly>> tileAndPoly = m_nav.getTileAndPolyByRef(ref);
        if (tileAndPoly.failed()) {
            return Result.of(tileAndPoly.status, tileAndPoly.message);
        }
        if (Objects.isNull(filter)) {
            return Result.invalidParam();
        }
        MeshTile tile = tileAndPoly.result.first;
        Poly poly = tileAndPoly.result.second;

        List<Long> segmentRefs = new ArrayList<>();
        List<float[]> segmentVerts = new ArrayList<>();
        List<SegInterval> ints = new ArrayList<>(16);

        for (int i = 0, j = poly.vertCount - 1; i < poly.vertCount; j = i++) {
            // Skip non-solid edges.
            ints.clear();
            if ((poly.neis[j] & NavMesh.DT_EXT_LINK) != 0) {
                // Tile border.
                for (int k = poly.firstLink; k != NavMesh.DT_NULL_LINK; k = tile.links.get(k).next) {
                    Link link = tile.links.get(k);
                    if (link.edge == j) {
                        if (link.ref != 0) {
                            Tupple2<MeshTile, Poly> tileAndPolyUnsafe = m_nav.getTileAndPolyByRefUnsafe(link.ref);
                            MeshTile neiTile = tileAndPolyUnsafe.first;
                            Poly neiPoly = tileAndPolyUnsafe.second;
                            if (filter.passFilter(link.ref, neiTile, neiPoly)) {
                                insertInterval(ints, link.bmin, link.bmax, link.ref);
                            }
                        }
                    }
                }
            } else {
                // Internal edge
                long neiRef = 0;
                if (poly.neis[j] != 0) {
                    int idx = (poly.neis[j] - 1);
                    neiRef = m_nav.getPolyRefBase(tile) | idx;
                    if (!filter.passFilter(neiRef, tile, tile.data.polys[idx])) {
                        neiRef = 0;
                    }
                }
                // If the edge leads to another polygon and portals are not stored, skip.
                if (neiRef != 0 && !storePortals) {
                    continue;
                }

                int vj = poly.verts[j] * 3;
                int vi = poly.verts[i] * 3;
                float[] seg = new float[6];
                System.arraycopy(tile.data.verts, vj, seg, 0, 3);
                System.arraycopy(tile.data.verts, vi, seg, 3, 3);
                segmentVerts.add(seg);
                segmentRefs.add(neiRef);
                continue;
            }

            // Add sentinels
            insertInterval(ints, -1, 0, 0);
            insertInterval(ints, 255, 256, 0);

            // Store segments.
            int vj = poly.verts[j] * 3;
            int vi = poly.verts[i] * 3;
            for (int k = 1; k < ints.size(); ++k) {
                // Portal segment.
                if (storePortals && ints.get(k).ref != 0) {
                    float tmin = ints.get(k).tmin / 255.0f;
                    float tmax = ints.get(k).tmax / 255.0f;
                    float[] seg = new float[6];
                    System.arraycopy(vLerp(tile.data.verts, vj, vi, tmin), 0, seg, 0, 3);
                    System.arraycopy(vLerp(tile.data.verts, vj, vi, tmax), 0, seg, 3, 3);
                    segmentVerts.add(seg);
                    segmentRefs.add(ints.get(k).ref);
                }

                // Wall segment.
                int imin = ints.get(k - 1).tmax;
                int imax = ints.get(k).tmin;
                if (imin != imax) {
                    float tmin = imin / 255.0f;
                    float tmax = imax / 255.0f;
                    float[] seg = new float[6];
                    System.arraycopy(vLerp(tile.data.verts, vj, vi, tmin), 0, seg, 0, 3);
                    System.arraycopy(vLerp(tile.data.verts, vj, vi, tmax), 0, seg, 3, 3);
                    segmentVerts.add(seg);
                    segmentRefs.add(0L);
                }
            }
        }

        return Result.success(new GetPolyWallSegmentsResult(segmentVerts, segmentRefs));
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
    /// Finds the distance from the specified position to the nearest polygon wall.
    /// @param[in] startRef The reference id of the polygon containing @p centerPos.
    /// @param[in] centerPos The center of the search circle. [(x, y, z)]
    /// @param[in] maxRadius The radius of the search circle.
    /// @param[in] filter The polygon filter to apply to the query.
    /// @param[out] hitDist The distance to the nearest wall from @p centerPos.
    /// @param[out] hitPos The nearest position on the wall that was hit. [(x, y, z)]
    /// @param[out] hitNormal The normalized ray formed from the wall point to the
    /// source point. [(x, y, z)]
    /// @returns The status flags for the query.
    public Result<FindDistanceToWallResult> findDistanceToWall(long startRef, float[] centerPos, float maxRadius,
            QueryFilter filter) {

        // Validate input
        if (!m_nav.isValidPolyRef(startRef) || Objects.isNull(centerPos) || !vIsFinite(centerPos) || maxRadius < 0
                || !Float.isFinite(maxRadius) || Objects.isNull(filter)) {
            return Result.invalidParam();
        }

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

        float radiusSqr = sqr(maxRadius);
        float[] hitPos = new float[3];
        VectorPtr bestvj = null;
        VectorPtr bestvi = null;
        while (!m_openList.isEmpty()) {
            Node bestNode = m_openList.pop();
            bestNode.flags &= ~Node.DT_NODE_OPEN;
            bestNode.flags |= Node.DT_NODE_CLOSED;

            // Get poly and tile.
            // The API input has been cheked already, skip checking internal data.
            long bestRef = bestNode.id;
            Tupple2<MeshTile, Poly> tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(bestRef);
            MeshTile bestTile = tileAndPoly.first;
            Poly bestPoly = tileAndPoly.second;

            // Get parent poly and tile.
            long parentRef = 0;
            MeshTile parentTile = null;
            Poly parentPoly = null;
            if (bestNode.pidx != 0) {
                parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
            }
            if (parentRef != 0) {
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
                parentTile = tileAndPoly.first;
                parentPoly = tileAndPoly.second;
            }

            // Hit test walls.
            for (int i = 0, j = bestPoly.vertCount - 1; i < bestPoly.vertCount; j = i++) {
                // Skip non-solid edges.
                if ((bestPoly.neis[j] & NavMesh.DT_EXT_LINK) != 0) {
                    // Tile border.
                    boolean solid = true;
                    for (int k = bestPoly.firstLink; k != NavMesh.DT_NULL_LINK; k = bestTile.links.get(k).next) {
                        Link link = bestTile.links.get(k);
                        if (link.edge == j) {
                            if (link.ref != 0) {
                                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(link.ref);
                                MeshTile neiTile = tileAndPoly.first;
                                Poly neiPoly = tileAndPoly.second;
                                if (filter.passFilter(link.ref, neiTile, neiPoly)) {
                                    solid = false;
                                }
                            }
                            break;
                        }
                    }
                    if (!solid) {
                        continue;
                    }
                } else if (bestPoly.neis[j] != 0) {
                    // Internal edge
                    int idx = (bestPoly.neis[j] - 1);
                    long ref = m_nav.getPolyRefBase(bestTile) | idx;
                    if (filter.passFilter(ref, bestTile, bestTile.data.polys[idx])) {
                        continue;
                    }
                }

                // Calc distance to the edge.
                int vj = bestPoly.verts[j] * 3;
                int vi = bestPoly.verts[i] * 3;
                Tupple2<Float, Float> distseg = distancePtSegSqr2D(centerPos, bestTile.data.verts, vj, vi);
                float distSqr = distseg.first;
                float tseg = distseg.second;

                // Edge is too far, skip.
                if (distSqr > radiusSqr) {
                    continue;
                }

                // Hit wall, update radius.
                radiusSqr = distSqr;
                // Calculate hit pos.
                hitPos[0] = bestTile.data.verts[vj] + (bestTile.data.verts[vi] - bestTile.data.verts[vj]) * tseg;
                hitPos[1] = bestTile.data.verts[vj + 1]
                        + (bestTile.data.verts[vi + 1] - bestTile.data.verts[vj + 1]) * tseg;
                hitPos[2] = bestTile.data.verts[vj + 2]
                        + (bestTile.data.verts[vi + 2] - bestTile.data.verts[vj + 2]) * tseg;
                bestvj = new VectorPtr(bestTile.data.verts, vj);
                bestvi = new VectorPtr(bestTile.data.verts, vi);
            }

            for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links.get(i).next) {
                Link link = bestTile.links.get(i);
                long neighbourRef = link.ref;
                // Skip invalid neighbours and do not follow back to parent.
                if (neighbourRef == 0 || neighbourRef == parentRef) {
                    continue;
                }

                // Expand to neighbour.
                tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(neighbourRef);
                MeshTile neighbourTile = tileAndPoly.first;
                Poly neighbourPoly = tileAndPoly.second;

                // Skip off-mesh connections.
                if (neighbourPoly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
                    continue;
                }

                // Calc distance to the edge.
                int va = bestPoly.verts[link.edge] * 3;
                int vb = bestPoly.verts[(link.edge + 1) % bestPoly.vertCount] * 3;
                Tupple2<Float, Float> distseg = distancePtSegSqr2D(centerPos, bestTile.data.verts, va, vb);
                float distSqr = distseg.first;
                // If the circle is not touching the next polygon, skip it.
                if (distSqr > radiusSqr) {
                    continue;
                }

                if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly)) {
                    continue;
                }

                Node neighbourNode = m_nodePool.getNode(neighbourRef);

                if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0) {
                    continue;
                }

                // Cost
                if (neighbourNode.flags == 0) {
                    Result<float[]> midPoint = getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly,
                            neighbourTile);
                    if (midPoint.succeeded()) {
                        neighbourNode.pos = midPoint.result;
                    }
                }

                float total = bestNode.total + vDist(bestNode.pos, neighbourNode.pos);

                // The node is already in open list and the new result is worse, skip.
                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total) {
                    continue;
                }

                neighbourNode.id = neighbourRef;
                neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
                neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
                neighbourNode.total = total;

                if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0) {
                    m_openList.modify(neighbourNode);
                } else {
                    neighbourNode.flags |= Node.DT_NODE_OPEN;
                    m_openList.push(neighbourNode);
                }
            }
        }

        // Calc hit normal.
        float[] hitNormal = new float[3];
        if (bestvi != null && bestvj != null) {
            float[] tangent = vSub(bestvi, bestvj);
            hitNormal[0] = tangent[2];
            hitNormal[1] = 0;
            hitNormal[2] = -tangent[0];
            vNormalize(hitNormal);
        }
        return Result.success(new FindDistanceToWallResult((float) Math.sqrt(radiusSqr), hitPos, hitNormal));
    }

    /// Returns true if the polygon reference is valid and passes the filter restrictions.
    /// @param[in] ref The polygon reference to check.
    /// @param[in] filter The filter to apply.
    public boolean isValidPolyRef(long ref, QueryFilter filter) {
        Result<Tupple2<MeshTile, Poly>> tileAndPolyResult = m_nav.getTileAndPolyByRef(ref);
        if (tileAndPolyResult.failed()) {
            return false;
        }
        Tupple2<MeshTile, Poly> tileAndPoly = tileAndPolyResult.result;
        // If cannot pass filter, assume flags has changed and boundary is invalid.
        if (!filter.passFilter(ref, tileAndPoly.first, tileAndPoly.second)) {
            return false;
        }
        return true;
    }

    /// Gets the navigation mesh the query object is using.
    /// @return The navigation mesh the query object is using.
    public NavMesh getAttachedNavMesh() {
        return m_nav;
    }

    /**
     * Gets a path from the explored nodes in the previous search.
     *
     * @param endRef
     *            The reference id of the end polygon.
     * @returns An ordered list of polygon references representing the path. (Start to end.)
     * @remarks The result of this function depends on the state of the query object. For that reason it should only be
     *          used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
     */
    public Result<List<Long>> getPathFromDijkstraSearch(long endRef) {
        if (!m_nav.isValidPolyRef(endRef)) {
            return Result.invalidParam("Invalid end ref");
        }
        List<Node> nodes = m_nodePool.findNodes(endRef);
        if (nodes.size() != 1) {
            return Result.invalidParam("Invalid end ref");
        }
        Node endNode = nodes.get(0);
        if ((endNode.flags & DT_NODE_CLOSED) == 0) {
            return Result.invalidParam("Invalid end ref");
        }
        return Result.success(getPathToNode(endNode));
    }

    /**
     * Gets the path leading to the specified end node.
     */
    private List<Long> getPathToNode(Node endNode) {
        List<Long> path = new ArrayList<>();
        // Reverse the path.
        Node curNode = endNode;
        do {
            path.add(0, curNode.id);
            curNode = m_nodePool.getNodeAtIdx(curNode.pidx);
        } while (curNode != null);

        return path;
    }

    /**
     * The closed list is the list of polygons that were fully evaluated during the last navigation graph search. (A* or
     * Dijkstra)
     */
    public boolean isInClosedList(long ref) {
        if (m_nodePool == null) {
            return false;
        }
        for (Node n : m_nodePool.findNodes(ref)) {
            if ((n.flags & DT_NODE_CLOSED) != 0) {
                return true;
            }
        }
        return false;
    }

    public NodePool getNodePool() {
        return m_nodePool;
    }

}