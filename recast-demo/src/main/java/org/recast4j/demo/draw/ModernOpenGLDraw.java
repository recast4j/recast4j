package org.recast4j.demo.draw;

import static org.lwjgl.opengl.GL11.GL_BLEND;
import static org.lwjgl.opengl.GL11.GL_COLOR_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_CULL_FACE;
import static org.lwjgl.opengl.GL11.GL_DEPTH_BUFFER_BIT;
import static org.lwjgl.opengl.GL11.GL_DEPTH_TEST;
import static org.lwjgl.opengl.GL11.GL_FLOAT;
import static org.lwjgl.opengl.GL11.GL_LINES;
import static org.lwjgl.opengl.GL11.GL_ONE_MINUS_SRC_ALPHA;
import static org.lwjgl.opengl.GL11.GL_POINTS;
import static org.lwjgl.opengl.GL11.GL_SRC_ALPHA;
import static org.lwjgl.opengl.GL11.GL_TEXTURE_2D;
import static org.lwjgl.opengl.GL11.GL_TRIANGLES;
import static org.lwjgl.opengl.GL11.GL_TRUE;
import static org.lwjgl.opengl.GL11.GL_UNSIGNED_BYTE;
import static org.lwjgl.opengl.GL11.GL_UNSIGNED_INT;
import static org.lwjgl.opengl.GL11.glBindTexture;
import static org.lwjgl.opengl.GL11.glBlendFunc;
import static org.lwjgl.opengl.GL11.glClear;
import static org.lwjgl.opengl.GL11.glClearColor;
import static org.lwjgl.opengl.GL11.glDepthMask;
import static org.lwjgl.opengl.GL11.glDisable;
import static org.lwjgl.opengl.GL11.glDrawElements;
import static org.lwjgl.opengl.GL11.glEnable;
import static org.lwjgl.opengl.GL11.glLineWidth;
import static org.lwjgl.opengl.GL11.glPointSize;
import static org.lwjgl.opengl.GL15.GL_ARRAY_BUFFER;
import static org.lwjgl.opengl.GL15.GL_ELEMENT_ARRAY_BUFFER;
import static org.lwjgl.opengl.GL15.GL_STREAM_DRAW;
import static org.lwjgl.opengl.GL15.GL_WRITE_ONLY;
import static org.lwjgl.opengl.GL15.glBindBuffer;
import static org.lwjgl.opengl.GL15.glBufferData;
import static org.lwjgl.opengl.GL15.glGenBuffers;
import static org.lwjgl.opengl.GL15.glMapBuffer;
import static org.lwjgl.opengl.GL15.glUnmapBuffer;
import static org.lwjgl.opengl.GL20.GL_COMPILE_STATUS;
import static org.lwjgl.opengl.GL20.GL_FRAGMENT_SHADER;
import static org.lwjgl.opengl.GL20.GL_LINK_STATUS;
import static org.lwjgl.opengl.GL20.GL_VERTEX_SHADER;
import static org.lwjgl.opengl.GL20.glAttachShader;
import static org.lwjgl.opengl.GL20.glCompileShader;
import static org.lwjgl.opengl.GL20.glCreateProgram;
import static org.lwjgl.opengl.GL20.glCreateShader;
import static org.lwjgl.opengl.GL20.glEnableVertexAttribArray;
import static org.lwjgl.opengl.GL20.glGetAttribLocation;
import static org.lwjgl.opengl.GL20.glGetProgrami;
import static org.lwjgl.opengl.GL20.glGetShaderi;
import static org.lwjgl.opengl.GL20.glGetUniformLocation;
import static org.lwjgl.opengl.GL20.glLinkProgram;
import static org.lwjgl.opengl.GL20.glShaderSource;
import static org.lwjgl.opengl.GL20.glUniform1f;
import static org.lwjgl.opengl.GL20.glUniform1i;
import static org.lwjgl.opengl.GL20.glUniformMatrix4fv;
import static org.lwjgl.opengl.GL20.glUseProgram;
import static org.lwjgl.opengl.GL20.glVertexAttribPointer;
import static org.lwjgl.opengl.GL30.glBindVertexArray;
import static org.lwjgl.opengl.GL30.glGenVertexArrays;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.lwjgl.system.Platform;

public class ModernOpenGLDraw implements OpenGLDraw {

    private int program;
    private int uniformTexture;
    private int uniformProjectionMatrix;
    private int vbo;
    private int ebo;
    private int vao;
    private DebugDrawPrimitives currentPrim;
    private float fogStart;
    private float fogEnd;
    private boolean fogEnabled;
    private int uniformViewMatrix;
    private final List<OpenGLVertex> vertices = new ArrayList<>();
    private GLCheckerTexture texture;
    private float[] viewMatrix;
    private float[] projectionMatrix;
    private int uniformUseTexture;
    private int uniformFog;
    private int uniformFogStart;
    private int uniformFogEnd;

    @Override
    public void init() {
        String NK_SHADER_VERSION = Platform.get() == Platform.MACOSX ? "#version 150\n" : "#version 300 es\n";
        String vertex_shader = NK_SHADER_VERSION + "uniform mat4 ProjMtx;\n"//
                + "uniform mat4 ViewMtx;\n"//
                + "in vec3 Position;\n"//
                + "in vec2 TexCoord;\n"//
                + "in vec4 Color;\n"//
                + "out vec2 Frag_UV;\n"//
                + "out vec4 Frag_Color;\n"//
                + "out float Frag_Depth;\n"//
                + "void main() {\n"//
                + "   Frag_UV = TexCoord;\n"//
                + "   Frag_Color = Color;\n"//
                + "   vec4 VSPosition = ViewMtx * vec4(Position, 1);\n"//
                + "   Frag_Depth = -VSPosition.z;\n"//
                + "   gl_Position = ProjMtx * VSPosition;\n"//
                + "}\n";
        String fragment_shader = NK_SHADER_VERSION + "precision mediump float;\n"//
                + "uniform sampler2D Texture;\n"//
                + "uniform float UseTexture;\n"//
                + "uniform float EnableFog;\n"//
                + "uniform float FogStart;\n"//
                + "uniform float FogEnd;\n"//
                + "const vec4 FogColor = vec4(0.32f, 0.31f, 0.30f, 1.0f);\n"//
                + "in vec2 Frag_UV;\n"//
                + "in vec4 Frag_Color;\n"//
                + "in float Frag_Depth;\n"//
                + "out vec4 Out_Color;\n"//
                + "void main(){\n"//
                + "   Out_Color = mix(FogColor, Frag_Color * mix(vec4(1), texture(Texture, Frag_UV.st), UseTexture), 1.0 - EnableFog * clamp( (Frag_Depth - FogStart) / (FogEnd - FogStart), 0.0, 1.0) );\n"//
                + "}\n";

        program = glCreateProgram();
        int vert_shdr = glCreateShader(GL_VERTEX_SHADER);
        int frag_shdr = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(vert_shdr, vertex_shader);
        glShaderSource(frag_shdr, fragment_shader);
        glCompileShader(vert_shdr);
        glCompileShader(frag_shdr);
        if (glGetShaderi(vert_shdr, GL_COMPILE_STATUS) != GL_TRUE) {
            throw new IllegalStateException();
        }
        if (glGetShaderi(frag_shdr, GL_COMPILE_STATUS) != GL_TRUE) {
            throw new IllegalStateException();
        }
        glAttachShader(program, vert_shdr);
        glAttachShader(program, frag_shdr);
        glLinkProgram(program);
        if (glGetProgrami(program, GL_LINK_STATUS) != GL_TRUE) {
            throw new IllegalStateException();
        }
        uniformTexture = glGetUniformLocation(program, "Texture");
        uniformUseTexture = glGetUniformLocation(program, "UseTexture");
        uniformFog = glGetUniformLocation(program, "EnableFog");
        uniformFogStart = glGetUniformLocation(program, "FogStart");
        uniformFogEnd = glGetUniformLocation(program, "FogEnd");
        uniformProjectionMatrix = glGetUniformLocation(program, "ProjMtx");
        uniformViewMatrix = glGetUniformLocation(program, "ViewMtx");
        int attrib_pos = glGetAttribLocation(program, "Position");
        int attrib_uv = glGetAttribLocation(program, "TexCoord");
        int attrib_col = glGetAttribLocation(program, "Color");

        // buffer setup
        vbo = glGenBuffers();
        ebo = glGenBuffers();
        vao = glGenVertexArrays();

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

        glEnableVertexAttribArray(attrib_pos);
        glEnableVertexAttribArray(attrib_uv);
        glEnableVertexAttribArray(attrib_col);

        glVertexAttribPointer(attrib_pos, 3, GL_FLOAT, false, 24, 0);
        glVertexAttribPointer(attrib_uv, 2, GL_FLOAT, false, 24, 12);
        glVertexAttribPointer(attrib_col, 4, GL_UNSIGNED_BYTE, true, 24, 20);

        glBindTexture(GL_TEXTURE_2D, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

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
    public void begin(DebugDrawPrimitives prim, float size) {
        currentPrim = prim;
        vertices.clear();
        glLineWidth(size);
        glPointSize(size);
    }

    @Override
    public void end() {
        if (vertices.isEmpty()) {
            return;
        }
        glUseProgram(program);
        glUniform1i(uniformTexture, 0);
        glUniformMatrix4fv(uniformViewMatrix, false, viewMatrix);
        glUniformMatrix4fv(uniformProjectionMatrix, false, projectionMatrix);
        glUniform1f(uniformFogStart, fogStart);
        glUniform1f(uniformFogEnd, fogEnd);
        glUniform1f(uniformFog, fogEnabled ? 1.0f : 0.0f);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        // glBufferData(GL_ARRAY_BUFFER, MAX_VERTEX_BUFFER, GL_STREAM_DRAW);
        // glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_ELEMENT_BUFFER, GL_STREAM_DRAW);

        int vboSize = vertices.size() * 24;
        int eboSize = currentPrim == DebugDrawPrimitives.QUADS ? vertices.size() * 6 : vertices.size() * 4;

        glBufferData(GL_ARRAY_BUFFER, vboSize, GL_STREAM_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, eboSize, GL_STREAM_DRAW);
        // load draw vertices & elements directly into vertex + element buffer
        ByteBuffer verts = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY, vboSize, null);
        ByteBuffer elems = glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY, eboSize, null);
        vertices.forEach(v -> v.store(verts));
        if (currentPrim == DebugDrawPrimitives.QUADS) {
            for (int i = 0; i < vertices.size(); i += 4) {
                elems.putInt(i);
                elems.putInt(i + 1);
                elems.putInt(i + 2);
                elems.putInt(i);
                elems.putInt(i + 2);
                elems.putInt(i + 3);
            }

        } else {
            for (int i = 0; i < vertices.size(); i++) {
                elems.putInt(i);
            }
        }

        glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        if (texture != null) {
            texture.bind();
            glUniform1f(uniformUseTexture, 1.0f);
        } else {
            glUniform1f(uniformUseTexture, 0.0f);
        }

        switch (currentPrim) {
        case POINTS:
            glDrawElements(GL_POINTS, vertices.size(), GL_UNSIGNED_INT, 0);
            break;
        case LINES:
            glDrawElements(GL_LINES, vertices.size(), GL_UNSIGNED_INT, 0);
            break;
        case TRIS:
            glDrawElements(GL_TRIANGLES, vertices.size(), GL_UNSIGNED_INT, 0);
            break;
        case QUADS:
            glDrawElements(GL_TRIANGLES, vertices.size() * 6 / 4, GL_UNSIGNED_INT, 0);
            break;
        default:
            break;
        }

        glUseProgram(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        vertices.clear();
        glLineWidth(1.0f);
        glPointSize(1.0f);
    }

    @Override
    public void vertex(float x, float y, float z, int color) {
        vertices.add(new OpenGLVertex(x, y, z, color));
    }

    @Override
    public void vertex(float[] pos, int color) {
        vertices.add(new OpenGLVertex(pos, color));
    }

    @Override
    public void vertex(float[] pos, int color, float[] uv) {
        vertices.add(new OpenGLVertex(pos, uv, color));
    }

    @Override
    public void vertex(float x, float y, float z, int color, float u, float v) {
        vertices.add(new OpenGLVertex(x, y, z, u, v, color));
    }

    @Override
    public void depthMask(boolean state) {
        glDepthMask(state);
    }

    @Override
    public void texture(GLCheckerTexture g_tex, boolean state) {
        texture = state ? g_tex : null;
    }

    @Override
    public void projectionMatrix(float[] projectionMatrix) {
        this.projectionMatrix = projectionMatrix;
    }

    @Override
    public void viewMatrix(float[] viewMatrix) {
        this.viewMatrix = viewMatrix;
    }

    @Override
    public void fog(float start, float end) {
        fogStart = start;
        fogEnd = end;
    }

    @Override
    public void fog(boolean state) {
        fogEnabled = state;
    }

}
