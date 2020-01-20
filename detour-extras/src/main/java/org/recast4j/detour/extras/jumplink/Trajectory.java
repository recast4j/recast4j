package org.recast4j.detour.extras.jumplink;

public interface Trajectory {

    default float lerp(float f, float g, float u) {
        return u * g + (1f - u) * f;
    }

    float[] apply(float[] start, float[] end, float u);

}
