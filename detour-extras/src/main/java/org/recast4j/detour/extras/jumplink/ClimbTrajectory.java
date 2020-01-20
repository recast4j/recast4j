package org.recast4j.detour.extras.jumplink;

public class ClimbTrajectory implements Trajectory {

    @Override
    public float[] apply(float[] start, float[] end, float u) {
        return new float[] { lerp(start[0], end[0], Math.min(2f * u, 1f)),
                lerp(start[1], end[1], Math.max(0f, 2f * u - 1f)),
                lerp(start[2], end[2], Math.min(2f * u, 1f)) };
    }

}
