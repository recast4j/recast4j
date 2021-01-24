package org.recast4j.detour;

import static org.recast4j.detour.DetourCommon.vLenSqr;
import static org.recast4j.detour.DetourCommon.vSub;

public class FindNearestPolyQuery implements PolyQuery {

    private final NavMeshQuery query;
    private final float[] center;
    private long nearestRef;
    private float[] nearestPt;
    private boolean overPoly;
    private float nearestDistanceSqr;

    public FindNearestPolyQuery(NavMeshQuery query, float[] center) {
        this.query = query;
        this.center = center;
        nearestDistanceSqr = Float.MAX_VALUE;
        nearestPt = new float[] { center[0], center[1], center[2] };
    }

    @Override
    public void process(MeshTile tile, Poly poly, long ref) {
        // Find nearest polygon amongst the nearby polygons.
        Result<ClosestPointOnPolyResult> closest = query.closestPointOnPoly(ref, center);
        boolean posOverPoly = closest.result.isPosOverPoly();
        float[] closestPtPoly = closest.result.getClosest();

        // If a point is directly over a polygon and closer than
        // climb height, favor that instead of straight line nearest point.
        float d = 0;
        float[] diff = vSub(center, closestPtPoly);
        if (posOverPoly) {
            d = Math.abs(diff[1]) - tile.data.header.walkableClimb;
            d = d > 0 ? d * d : 0;
        } else {
            d = vLenSqr(diff);
        }

        if (d < nearestDistanceSqr) {
            nearestPt = closestPtPoly;
            nearestDistanceSqr = d;
            nearestRef = ref;
            overPoly = posOverPoly;
        }

    }

    public FindNearestPolyResult result() {
        return new FindNearestPolyResult(nearestRef, nearestPt, overPoly);
    }

}
