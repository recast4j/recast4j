package org.recast4j.demo.tool.jumplink;

public class JumpLink {

    public static final int MAX_SPINE = 8;
    public final float[] spine0 = new float[MAX_SPINE * 3];
    public final float[] spine1 = new float[MAX_SPINE * 3];
    public long startPoly;
    public long endPoly;
    public int nspine;
    public EdgeSampler esgeSampler;

}
