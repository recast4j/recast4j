package org.recast4j.demo.tool.jumplink;

import org.recast4j.demo.tool.jumplink.JumpLinkBuilder.EdgeSamplerType;

public class EdgeSampler {

    public final EdgeSamplerType type;
    public final GroundSegment start = new GroundSegment();
    public final GroundSegment end = new GroundSegment();
    public float heightRange;
    public final Trajectory2D trajectory = new Trajectory2D();

    final float ax[] = new float[3];
    final float ay[] = new float[3];
    final float az[] = new float[3];

    public EdgeSampler(EdgeSamplerType type) {
        this.type = type;
    }

}
