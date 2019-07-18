package org.recast4j.demo.tool.annotation;

import java.util.Arrays;

public class Edge {

    public final float sp[] = new float[3];
    public final float sq[] = new float[3];

    @Override
    public String toString() {
        return "Edge [sp=" + Arrays.toString(sp) + ", sq=" + Arrays.toString(sq) + "]";
    }

}
