package org.recast4j.demo.tool;

import java.util.List;
import java.util.Optional;

import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.StraightPathItem;

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
        List<StraightPathItem> straightPath = navQuery.findStraightPath(startPos, endPos, path, MAX_STEER_POINTS, 0);
        if (straightPath.isEmpty()) {
            return Optional.empty();
        }

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

}
