package org.recast4j.detour.extras.jumplink;

public class JumpLink {

    public static final int MAX_SPINE = 8;
    public final int nspine = MAX_SPINE;
    public final float[] spine0 = new float[MAX_SPINE * 3];
    public final float[] spine1 = new float[MAX_SPINE * 3];
    public GroundSample[] startSamples;
    public GroundSample[] endSamples;
    public GroundSegment start;
    public GroundSegment end;
    public Trajectory trajectory;

}
