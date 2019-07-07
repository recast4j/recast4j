package org.recast4j.demo.draw;

import java.nio.ByteBuffer;

public class OpenGLVertex {

    private final float x;
    private final float y;
    private final float z;
    private final int color;
    private final float u;
    private final float v;

    public OpenGLVertex(float[] pos, float[] uv, int color) {
        this(pos[0], pos[1], pos[2], uv[0], uv[1], color);
    }

    public OpenGLVertex(float[] pos, int color) {
        this(pos[0], pos[1], pos[2], 0f, 0f, color);
    }

    public OpenGLVertex(float x, float y, float z, int color) {
        this(x, y, z, 0f, 0f, color);
    }

    public OpenGLVertex(float x, float y, float z, float u, float v, int color) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.u = u;
        this.v = v;
        this.color = color;
    }

    public void store(ByteBuffer buffer) {
        buffer.putFloat(x);
        buffer.putFloat(y);
        buffer.putFloat(z);
        buffer.putFloat(u);
        buffer.putFloat(v);
        buffer.putInt(color);
    }
}
