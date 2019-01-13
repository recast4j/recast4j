/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.demo.tool;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.recast4j.detour.Link;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Poly;
import org.recast4j.detour.Result;
import org.recast4j.detour.StraightPathItem;
import org.recast4j.detour.Tupple2;

public class PathUtils {

    private final static int MAX_STEER_POINTS = 3;

    public static class SteerTarget {
        public final float[] steerPos;
        public final int steerPosFlag;
        public final long steerPosRef;
        public final float[] steerPoints;

        public SteerTarget(float[] steerPos, int steerPosFlag, long steerPosRef, float[] steerPoints) {
            this.steerPos = steerPos;
            this.steerPosFlag = steerPosFlag;
            this.steerPosRef = steerPosRef;
            this.steerPoints = steerPoints;
        }

    }

    public static Optional<SteerTarget> getSteerTarget(NavMeshQuery navQuery, float[] startPos, float[] endPos,
            float minTargetDist, List<Long> path) {
        // Find steer target.
        Result<List<StraightPathItem>> result = navQuery.findStraightPath(startPos, endPos, path, MAX_STEER_POINTS, 0);
        if (result.failed()) {
            return Optional.empty();
        }
        List<StraightPathItem> straightPath = result.result;
        float[] steerPoints = new float[straightPath.size() * 3];
        for (int i = 0; i < straightPath.size(); i++) {
            steerPoints[i * 3] = straightPath.get(i).getPos()[0];
            steerPoints[i * 3 + 1] = straightPath.get(i).getPos()[1];
            steerPoints[i * 3 + 2] = straightPath.get(i).getPos()[2];
        }

        // Find vertex far enough to steer to.
        int ns = 0;
        while (ns < straightPath.size()) {
            // Stop at Off-Mesh link or when point is further than slop away.
            if (((straightPath.get(ns).getFlags() & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0)
                    || !inRange(straightPath.get(ns).getPos(), startPos, minTargetDist, 1000.0f))
                break;
            ns++;
        }
        // Failed to find good point to steer to.
        if (ns >= straightPath.size())
            return Optional.empty();

        float[] steerPos = new float[] { straightPath.get(ns).getPos()[0], startPos[1],
                straightPath.get(ns).getPos()[2] };
        int steerPosFlag = straightPath.get(ns).getFlags();
        long steerPosRef = straightPath.get(ns).getRef();

        SteerTarget target = new SteerTarget(steerPos, steerPosFlag, steerPosRef, steerPoints);
        return Optional.of(target);

    }

    public static boolean inRange(float[] v1, float[] v2, float r, float h) {
        float dx = v2[0] - v1[0];
        float dy = v2[1] - v1[1];
        float dz = v2[2] - v1[2];
        return (dx * dx + dz * dz) < r * r && Math.abs(dy) < h;
    }

    static List<Long> fixupCorridor(List<Long> path, List<Long> visited) {
        int furthestPath = -1;
        int furthestVisited = -1;

        // Find furthest common polygon.
        for (int i = path.size() - 1; i >= 0; --i) {
            boolean found = false;
            for (int j = visited.size() - 1; j >= 0; --j) {
                if (path.get(i).longValue() == visited.get(j).longValue()) {
                    furthestPath = i;
                    furthestVisited = j;
                    found = true;
                }
            }
            if (found)
                break;
        }

        // If no intersection found just return current path.
        if (furthestPath == -1 || furthestVisited == -1)
            return path;

        // Concatenate paths.

        // Adjust beginning of the buffer to include the visited.
        int req = visited.size() - furthestVisited;
        int orig = Math.min(furthestPath + 1, path.size());
        int size = Math.max(0, path.size() - orig);
        List<Long> fixupPath = new ArrayList<>();
        // Store visited
        for (int i = 0; i < req; ++i) {
            fixupPath.add(visited.get((visited.size() - 1) - i));
        }
        for (int i = 0; i < size; i++) {
            fixupPath.add(path.get(orig + i));
        }

        return fixupPath;
    }

    // This function checks if the path has a small U-turn, that is,
    // a polygon further in the path is adjacent to the first polygon
    // in the path. If that happens, a shortcut is taken.
    // This can happen if the target (T) location is at tile boundary,
    // and we're (S) approaching it parallel to the tile edge.
    // The choice at the vertex can be arbitrary,
    // +---+---+
    // |:::|:::|
    // +-S-+-T-+
    // |:::| | <-- the step can end up in here, resulting U-turn path.
    // +---+---+
    static List<Long> fixupShortcuts(List<Long> path, NavMeshQuery navQuery) {
        if (path.size() < 3) {
            return path;
        }

        // Get connected polygons
        List<Long> neis = new ArrayList<>();

        Result<Tupple2<MeshTile, Poly>> tileAndPoly = navQuery.getAttachedNavMesh().getTileAndPolyByRef(path.get(0));
        if (tileAndPoly.failed()) {
            return path;
        }
        MeshTile tile = tileAndPoly.result.first;
        Poly poly = tileAndPoly.result.second;

        for (int k = poly.firstLink; k != NavMesh.DT_NULL_LINK; k = tile.links.get(k).next) {
            Link link = tile.links.get(k);
            if (link.ref != 0) {
                neis.add(link.ref);
            }
        }

        // If any of the neighbour polygons is within the next few polygons
        // in the path, short cut to that polygon directly.
        int maxLookAhead = 6;
        int cut = 0;
        for (int i = Math.min(maxLookAhead, path.size()) - 1; i > 1 && cut == 0; i--) {
            for (int j = 0; j < neis.size(); j++) {
                if (path.get(i).longValue() == neis.get(j).longValue()) {
                    cut = i;
                    break;
                }
            }
        }
        if (cut > 1) {
            List<Long> shortcut = new ArrayList<>();
            shortcut.add(path.get(0));
            shortcut.addAll(path.subList(cut, path.size()));
            return shortcut;
        }
        return path;
    }

}
