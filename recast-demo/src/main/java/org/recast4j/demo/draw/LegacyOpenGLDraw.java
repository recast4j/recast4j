package org.recast4j.demo.draw;

import static org.lwjgl.opengl.GL11.GL_BLEND;
import static org.lwjgl.opengl.GL11.GL_COLOR_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_CULL_FACE;
import static org.lwjgl.opengl.GL11.GL_DEPTH_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_DEPTH_TEST;
import static org.lwjgl.opengl.GL11.GL_FOG;
import static org.lwjgl.opengl.GL11.GL_FOG_COLOR;
import static org.lwjgl.opengl.GL11.GL_FOG_END;
import static org.lwjgl.opengl.GL11.GL_FOG_MODE;
import static org.lwjgl.opengl.GL11.GL_FOG_START;
import static org.lwjgl.opengl.GL11.GL_LEQUAL;
import static org.lwjgl.opengl.GL11.GL_LINEAR;
import static org.lwjgl.opengl.GL11.GL_LINES;
import static org.lwjgl.opengl.GL11.GL_MODELVIEW;
import static org.lwjgl.opengl.GL11.GL_ONE_MINUS_SRC_ALPHA;
import static org.lwjgl.opengl.GL11.GL_POINTS;
import static org.lwjgl.opengl.GL11.GL_PROJECTION;
import static org.lwjgl.opengl.GL11.GL_QUADS;
import static org.lwjgl.opengl.GL11.GL_SRC_ALPHA;
import static org.lwjgl.opengl.GL11.GL_TEXTURE_2D;
import static org.lwjgl.opengl.GL11.GL_TRIANGLES;
import static org.lwjgl.opengl.GL11.glBegin;
import static org.lwjgl.opengl.GL11.glBlendFunc;
import static org.lwjgl.opengl.GL11.glClear;
import static org.lwjgl.opengl.GL11.glClearColor;
import static org.lwjgl.opengl.GL11.glColor4ub;
import static org.lwjgl.opengl.GL11.glDepthFunc;
import static org.lwjgl.opengl.GL11.glDepthMask;
import static org.lwjgl.opengl.GL11.glDisable;
import static org.lwjgl.opengl.GL11.glEnable;
import static org.lwjgl.opengl.GL11.glEnd;
import static org.lwjgl.opengl.GL11.glFogf;
import static org.lwjgl.opengl.GL11.glFogfv;
import static org.lwjgl.opengl.GL11.glFogi;
import static org.lwjgl.opengl.GL11.glLineWidth;
import static org.lwjgl.opengl.GL11.glLoadMatrixf;
import static org.lwjgl.opengl.GL11.glMatrixMode;
import static org.lwjgl.opengl.GL11.glPointSize;
import static org.lwjgl.opengl.GL11.glTexCoord2f;
import static org.lwjgl.opengl.GL11.glTexCoord2fv;
import static org.lwjgl.opengl.GL11.glVertex3f;
import static org.lwjgl.opengl.GL11.glVertex3fv;

public class LegacyOpenGLDraw implements OpenGLDraw {

    @Override
    public void fog(boolean state) {
        if (state) {
            glEnable(GL_FOG);
        } else {
            glDisable(GL_FOG);
        }
    }

    @Override
    public void init() {
        // Fog.
        float fogDistance = 1000f;
        float fogColor[] = { 0.32f, 0.31f, 0.30f, 1.0f };
        glEnable(GL_FOG);
        glFogi(GL_FOG_MODE, GL_LINEAR);
        glFogf(GL_FOG_START, fogDistance * 0.1f);
        glFogf(GL_FOG_END, fogDistance * 1.25f);
        glFogfv(GL_FOG_COLOR, fogColor);
        glDepthFunc(GL_LEQUAL);
    }

    @Override
    public void clear() {
        glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
    }

    @Override
    public void projectionMatrix(float[] matrix) {
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(matrix);
    }

    @Override
    public void viewMatrix(float[] matrix) {
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(matrix);
    }

    @Override
    public void begin(DebugDrawPrimitives prim, float size) {
        switch (prim) {
        case POINTS:
            glPointSize(size);
            glBegin(GL_POINTS);
            break;
        case LINES:
            glLineWidth(size);
            glBegin(GL_LINES);
            break;
        case TRIS:
            glBegin(GL_TRIANGLES);
            break;
        case QUADS:
            glBegin(GL_QUADS);
            break;
        }
    }

    @Override
    public void vertex(float[] pos, int color) {
        glColor4ubv(color);
        glVertex3fv(pos);
    }

    @Override
    public void vertex(float x, float y, float z, int color) {
        glColor4ubv(color);
        glVertex3f(x, y, z);
    }

    @Override
    public void vertex(float[] pos, int color, float[] uv) {
        glColor4ubv(color);
        glTexCoord2fv(uv);
        glVertex3fv(pos);
    }

    @Override
    public void vertex(float x, float y, float z, int color, float u, float v) {
        glColor4ubv(color);
        glTexCoord2f(u, v);
        glVertex3f(x, y, z);
    }

    private void glColor4ubv(int color) {
        glColor4ub((byte) (color & 0xFF), (byte) ((color >> 8) & 0xFF), (byte) ((color >> 16) & 0xFF),
                (byte) ((color >> 24) & 0xFF));
    }

    @Override
    public void depthMask(boolean state) {
        glDepthMask(state);
    }

    @Override
    public void texture(GLCheckerTexture g_tex, boolean state) {
        if (state) {
            glEnable(GL_TEXTURE_2D);
            g_tex.bind();
        } else {
            glDisable(GL_TEXTURE_2D);
        }
    }

    @Override
    public void end() {
        glEnd();
        glLineWidth(1.0f);
        glPointSize(1.0f);
    }

    @Override
    public void fog(float start, float end) {
        glFogf(GL_FOG_START, start);
        glFogf(GL_FOG_END, end);
    }

}
