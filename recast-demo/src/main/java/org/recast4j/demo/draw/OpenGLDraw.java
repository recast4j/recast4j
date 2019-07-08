package org.recast4j.demo.draw;

public interface OpenGLDraw {

    void init();

    void clear();

    void begin(DebugDrawPrimitives prim, float size);

    void end();

    void vertex(float x, float y, float z, int color);

    void vertex(float[] pos, int color);

    void vertex(float[] pos, int color, float[] uv);

    void vertex(float x, float y, float z, int color, float u, float v);

    void fog(boolean state);

    void depthMask(boolean state);

    void texture(GLCheckerTexture g_tex, boolean state);

    void projectionMatrix(float[] projectionMatrix);

    void viewMatrix(float[] viewMatrix);

    void fog(float start, float end);

}
