package org.recast4j.detour.extras.jumplink;

import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vSet;
import static org.recast4j.detour.DetourCommon.vSub;

import java.util.ArrayList;
import java.util.List;

public class EdgeSampler {

    public final GroundSegment start = new GroundSegment();
    public final List<GroundSegment> end = new ArrayList<>();
    public final Trajectory trajectory;

    final float ax[] = new float[3];
    final float ay[] = new float[3];
    final float az[] = new float[3];

    public EdgeSampler(Edge edge, Trajectory trajectory) {
        this.trajectory = trajectory;
        vCopy(ax, vSub(edge.sq, edge.sp));
        vNormalize(ax);
        vSet(az, ax[2], 0, -ax[0]);
        vNormalize(az);
        vSet(ay, 0, 1, 0);
    }

}
