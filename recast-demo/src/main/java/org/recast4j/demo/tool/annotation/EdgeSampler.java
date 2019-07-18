package org.recast4j.demo.tool.annotation;

public class EdgeSampler {
    public final GroundSegment start = new GroundSegment();
    public final GroundSegment end = new GroundSegment();

    public float groundRange;

    public final Trajectory2D trajectory = new Trajectory2D();

    final float rigp[] = new float[3];
    final float rigq[] = new float[3];
    final float ax[] = new float[3];
    final float ay[] = new float[3];
    final float az[] = new float[3];

}
