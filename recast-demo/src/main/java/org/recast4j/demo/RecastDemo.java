/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.demo;

import static org.lwjgl.glfw.GLFW.GLFW_BLUE_BITS;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MAJOR;
import static org.lwjgl.glfw.GLFW.GLFW_CONTEXT_VERSION_MINOR;
import static org.lwjgl.glfw.GLFW.GLFW_DEPTH_BITS;
import static org.lwjgl.glfw.GLFW.GLFW_DOUBLEBUFFER;
import static org.lwjgl.glfw.GLFW.GLFW_GREEN_BITS;
import static org.lwjgl.glfw.GLFW.GLFW_MOD_CONTROL;
import static org.lwjgl.glfw.GLFW.GLFW_MOD_SHIFT;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_CORE_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_FORWARD_COMPAT;
import static org.lwjgl.glfw.GLFW.GLFW_OPENGL_PROFILE;
import static org.lwjgl.glfw.GLFW.GLFW_RED_BITS;
import static org.lwjgl.glfw.GLFW.GLFW_RESIZABLE;
import static org.lwjgl.glfw.GLFW.GLFW_SAMPLES;
import static org.lwjgl.glfw.GLFW.GLFW_SRGB_CAPABLE;
import static org.lwjgl.glfw.GLFW.GLFW_VISIBLE;
import static org.lwjgl.glfw.GLFW.glfwCreateWindow;
import static org.lwjgl.glfw.GLFW.glfwDefaultWindowHints;
import static org.lwjgl.glfw.GLFW.glfwGetPrimaryMonitor;
import static org.lwjgl.glfw.GLFW.glfwGetVideoMode;
import static org.lwjgl.glfw.GLFW.glfwInit;
import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.glfw.GLFW.glfwPollEvents;
import static org.lwjgl.glfw.GLFW.glfwSetErrorCallback;
import static org.lwjgl.glfw.GLFW.glfwSetWindowPos;
import static org.lwjgl.glfw.GLFW.glfwShowWindow;
import static org.lwjgl.glfw.GLFW.glfwSwapBuffers;
import static org.lwjgl.glfw.GLFW.glfwSwapInterval;
import static org.lwjgl.glfw.GLFW.glfwWindowHint;
import static org.lwjgl.glfw.GLFW.glfwWindowShouldClose;
import static org.lwjgl.opengl.GL.createCapabilities;
import static org.lwjgl.opengl.GL11.GL_FALSE;
import static org.lwjgl.opengl.GL11.GL_RENDERER;
import static org.lwjgl.opengl.GL11.GL_TRUE;
import static org.lwjgl.opengl.GL11.GL_VENDOR;
import static org.lwjgl.opengl.GL11.GL_VERSION;
import static org.lwjgl.opengl.GL11.GL_VIEWPORT;
import static org.lwjgl.opengl.GL11.glGetIntegerv;
import static org.lwjgl.opengl.GL11.glGetString;
import static org.lwjgl.opengl.GL11.glViewport;
import static org.lwjgl.opengl.GL20.GL_SHADING_LANGUAGE_VERSION;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.IntBuffer;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.lwjgl.PointerBuffer;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.glfw.GLFWVidMode;
import org.lwjgl.opengl.ARBDebugOutput;
import org.lwjgl.opengl.GL43;
import org.lwjgl.opengl.GLCapabilities;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import org.recast4j.demo.builder.SoloNavMeshBuilder;
import org.recast4j.demo.builder.TileNavMeshBuilder;
import org.recast4j.demo.draw.GLU;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.io.ObjImporter;
import org.recast4j.demo.math.DemoMath;
import org.recast4j.demo.sample.Sample;
import org.recast4j.demo.settings.SettingsUI;
import org.recast4j.demo.tool.ConvexVolumeTool;
import org.recast4j.demo.tool.CrowdTool;
import org.recast4j.demo.tool.OffMeshConnectionTool;
import org.recast4j.demo.tool.TestNavmeshTool;
import org.recast4j.demo.tool.Tool;
import org.recast4j.demo.tool.ToolsUI;
import org.recast4j.demo.ui.Mouse;
import org.recast4j.demo.ui.MouseListener;
import org.recast4j.demo.ui.NuklearUI;
import org.recast4j.detour.DetourCommon;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.Tupple2;
import org.recast4j.recast.Recast;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RecastDemo {

    private final Logger logger = LoggerFactory.getLogger(getClass());
    private NuklearUI nuklearUI;
    private GLFWErrorCallback errorfun;
    private int width = 1000;
    private int height = 900;
    private final String title = "Recast4j Demo";
    private GLCapabilities capabilities;
    private final RecastDebugDraw dd = new RecastDebugDraw();
    private final NavMeshRenderer renderer = new NavMeshRenderer(dd);
    private boolean building = false;

    private final SoloNavMeshBuilder soloNavMeshBuilder = new SoloNavMeshBuilder();
    private final TileNavMeshBuilder tileNavMeshBuilder = new TileNavMeshBuilder();

    private Sample sample;

    private boolean processHitTest = false;
    private boolean processHitTestShift;
    private int modState;

    private final float[] mousePos = new float[2];

    private boolean mouseOverMenu;
    private boolean pan;
    private boolean movedDuringPan;
    private boolean rotate;
    private boolean movedDuringRotate;
    private float scrollZoom;
    private final float[] origMousePos = new float[2];
    private final float[] origCameraEulers = new float[2];
    private final float[] origCameraPos = new float[3];

    private final float cameraEulers[] = { 45, -45 };
    private final float cameraPos[] = { 0, 0, 0 };

    private final float[] rayStart = new float[3];
    private final float[] rayEnd = new float[3];

    private boolean markerPositionSet;
    private final float[] markerPosition = new float[3];
    private boolean showSample;
    private ToolsUI toolsUI;
    private SettingsUI settingsUI;
    private long prevFrameTime;

    public void start() {
        glfwSetErrorCallback(errorfun = GLFWErrorCallback.createPrint(System.err));
        if (!glfwInit()) {
            throw new IllegalStateException("Unable to initialize GLFW");
        }
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
        glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
        glfwWindowHint(GLFW_SRGB_CAPABLE, GL_TRUE);
        glfwWindowHint(GLFW_RED_BITS, 8);
        glfwWindowHint(GLFW_GREEN_BITS, 8);
        glfwWindowHint(GLFW_BLUE_BITS, 8);
        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
        glfwWindowHint(GLFW_DEPTH_BITS, 24);

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        // glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        // glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        // PointerBuffer monitors = glfwGetMonitors();
        long monitor = glfwGetPrimaryMonitor();
        // if (monitors.limit() > 1) {
        // monitor = monitors.get(1);
        // }
        GLFWVidMode mode = glfwGetVideoMode(monitor);
        // glfwWindowHint(GLFW_RED_BITS, mode.redBits());
        // glfwWindowHint(GLFW_GREEN_BITS, mode.greenBits());
        // glfwWindowHint(GLFW_BLUE_BITS, mode.blueBits());
        // glfwWindowHint(GLFW_REFRESH_RATE, mode.refreshRate());

        float aspect = 16.0f / 9.0f;
        width = Math.min(mode.width(), (int) (mode.height() * aspect)) - 80;
        height = mode.height() - 80;

        long window = glfwCreateWindow(width, height, title, NULL, NULL);
        if (window == NULL) {
            throw new RuntimeException("Failed to create the GLFW window");
        }
        // -- move somewhere else:
        glfwSetWindowPos(window, (mode.width() - width) / 2, (mode.height() - height) / 2);
        // glfwSetWindowMonitor(window.getWindow(), monitor, 0, 0, mode.width(), mode.height(), mode.refreshRate());
        glfwShowWindow(window);

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        capabilities = createCapabilities();
        if (capabilities.OpenGL43) {
            GL43.glDebugMessageControl(GL43.GL_DEBUG_SOURCE_API, GL43.GL_DEBUG_TYPE_OTHER,
                    GL43.GL_DEBUG_SEVERITY_NOTIFICATION, (IntBuffer) null, false);
        } else if (capabilities.GL_ARB_debug_output) {
            ARBDebugOutput.glDebugMessageControlARB(ARBDebugOutput.GL_DEBUG_SOURCE_API_ARB,
                    ARBDebugOutput.GL_DEBUG_TYPE_OTHER_ARB, ARBDebugOutput.GL_DEBUG_SEVERITY_LOW_ARB, (IntBuffer) null,
                    false);
        }
        String vendor = glGetString(GL_VENDOR);
        logger.debug(vendor);
        String version = glGetString(GL_VERSION);
        logger.debug(version);
        String renderGl = glGetString(GL_RENDERER);
        logger.debug(renderGl);
        String glslString = glGetString(GL_SHADING_LANGUAGE_VERSION);
        logger.debug(glslString);

        float camr = 1000;
        dd.init(camr);

        Mouse mouse = new Mouse(window);
        mouse.addListener(new MouseListener() {

            @Override
            public void scroll(double xoffset, double yoffset) {
                if (yoffset < 0) {
                    // wheel down
                    if (!mouseOverMenu) {
                        scrollZoom += 1.0f;
                    }
                } else {
                    if (!mouseOverMenu) {
                        scrollZoom -= 1.0f;
                    }
                }
            }

            @Override
            public void position(double x, double y) {
                mousePos[0] = (float) x;
                mousePos[1] = (float) y;
                int dx = (int) (mousePos[0] - origMousePos[0]);
                int dy = (int) (mousePos[1] - origMousePos[1]);
                if (rotate) {
                    cameraEulers[0] = origCameraEulers[0] + dy * 0.25f;
                    cameraEulers[1] = origCameraEulers[1] + dx * 0.25f;
                    if (dx * dx + dy * dy > 3 * 3) {
                        movedDuringRotate = true;
                    }
                }
                if (pan) {
                    cameraPos[0] = origCameraPos[0] + dx * 0.25f;
                    if (dx * dx + dy * dy > 3 * 3) {
                        movedDuringPan = true;
                    }
                }
            }

            @Override
            public void button(int button, int mods, boolean down) {
                modState = mods;
                if (down) {
                    if (button == 1) {
                        if (!mouseOverMenu) {
                            // Rotate view
                            rotate = true;
                            movedDuringRotate = false;
                            origMousePos[0] = mousePos[0];
                            origMousePos[1] = mousePos[1];
                            origCameraEulers[0] = cameraEulers[0];
                            origCameraEulers[1] = cameraEulers[1];
                        }
                    } else if (button == 2) {
                        if (!mouseOverMenu) {
                            // Pan view
                            pan = true;
                            movedDuringPan = false;
                            System.out.println("PAN!!");
                            origMousePos[0] = mousePos[0];
                            origMousePos[1] = mousePos[1];
                            origCameraPos[0] = cameraPos[0];
                            origCameraPos[1] = cameraPos[1];
                            origCameraPos[2] = cameraPos[2];
                        }
                    }
                } else {
                    // Handle mouse clicks here.
                    if (button == 1) {
                        rotate = false;
                        if (!mouseOverMenu) {
                            if (!movedDuringRotate) {
                                processHitTest = true;
                                processHitTestShift = true;
                            }
                        }
                    } else if (button == 0) {
                        if (!mouseOverMenu) {
                            processHitTest = true;
                            processHitTestShift = (mods & GLFW_MOD_SHIFT) != 0 ? true : false;
                        }
                    } else if (button == 2) {
                        pan = false;
                    }
                }
            }
        });

        settingsUI = new SettingsUI();
        toolsUI = new ToolsUI(new TestNavmeshTool(), new OffMeshConnectionTool(), new ConvexVolumeTool(),
                new CrowdTool());

        nuklearUI = new NuklearUI(window, mouse, settingsUI, toolsUI);

        DemoInputGeomProvider geom = loadInputMesh(getClass().getClassLoader().getResourceAsStream("nav_test.obj"));

        float timeAcc = 0;
        while (!glfwWindowShouldClose(window)) {

            /*
             * try (MemoryStack stack = stackPush()) { IntBuffer w = stack.mallocInt(1); IntBuffer h =
             * stack.mallocInt(1); glfwGetWindowSize(win, w, h); width = w.get(0); height = h.get(0); }
             */
            if (geom != null) {
                float[] bmin = geom.getMeshBoundsMin();
                float[] bmax = geom.getMeshBoundsMax();
                int[] voxels = Recast.calcGridSize(bmin, bmax, settingsUI.getCellSize());
                settingsUI.setVoxels(voxels);
                settingsUI.setTiles(
                        tileNavMeshBuilder.getTiles(geom, settingsUI.getCellSize(), settingsUI.getTileSize()));
                settingsUI.setMaxTiles(
                        tileNavMeshBuilder.getMaxTiles(geom, settingsUI.getCellSize(), settingsUI.getTileSize()));
                settingsUI.setMaxPolys(tileNavMeshBuilder.getMaxPolysPerTile(geom, settingsUI.getCellSize(),
                        settingsUI.getTileSize()));
            }

            nuklearUI.inputBegin();
            glfwPollEvents();
            nuklearUI.inputEnd(window);

            long time = System.nanoTime() / 1000;
            float dt = (time - prevFrameTime) / 1000000.0f;
            prevFrameTime = time;

            // Update sample simulation.
            float SIM_RATE = 20;
            float DELTA_TIME = 1.0f / SIM_RATE;
            timeAcc = DetourCommon.clamp(timeAcc + dt, -1.0f, 1.0f);
            int simIter = 0;
            while (timeAcc > DELTA_TIME) {
                timeAcc -= DELTA_TIME;
                if (simIter < 5 && sample != null) {
                    toolsUI.handleUpdate(DELTA_TIME);
                }
                simIter++;
            }

            // Set the viewport.
            //glViewport(0, 0, width, height);
            int[] viewport = new int[] {0, 0, width, height};
        //    glGetIntegerv(GL_VIEWPORT, viewport);

            // Clear the screen
            dd.clear();
            float[] projectionMatrix = dd.projectionMatrix(50f, (float) width / (float) height, 1.0f, camr);
            float[] modelviewMatrix = dd.viewMatrix(cameraPos, cameraEulers);

            mouseOverMenu = nuklearUI.layout(window, 0, 0, width, height, (int) mousePos[0], (int) mousePos[1]);

            int dx = 0;
            int dy = 0;
            if (pan) {
                dx = (int) (mousePos[0] - origMousePos[0]);
                dy = (int) (mousePos[1] - origMousePos[1]);
                if (dx * dx + dy * dy > 3 * 3) {
                    movedDuringPan = true;
                }
                cameraPos[0] = origCameraPos[0];
                cameraPos[1] = origCameraPos[1];
                cameraPos[2] = origCameraPos[2];

                cameraPos[0] -= 0.1f * dx * modelviewMatrix[0];
                cameraPos[1] -= 0.1f * dx * modelviewMatrix[4];
                cameraPos[2] -= 0.1f * dx * modelviewMatrix[8];

                cameraPos[0] += 0.1f * dy * modelviewMatrix[1];
                cameraPos[1] += 0.1f * dy * modelviewMatrix[5];
                cameraPos[2] += 0.1f * dy * modelviewMatrix[9];
            }

            cameraPos[0] += scrollZoom * 2.0f * modelviewMatrix[2];
            cameraPos[1] += scrollZoom * 2.0f * modelviewMatrix[6];
            cameraPos[2] += scrollZoom * 2.0f * modelviewMatrix[10];
            scrollZoom = 0;

            NavMesh navMesh;
            if (settingsUI.isMeshInputTrigerred()) {
                try (MemoryStack stack = stackPush()) {
                    PointerBuffer aFilterPatterns = stack.mallocPointer(2);
                    aFilterPatterns.put(stack.UTF8("*.obj"));
                    aFilterPatterns.flip();
                    String filename = TinyFileDialogs.tinyfd_openFileDialog("Open Mesh File", "", aFilterPatterns,
                            "Mesh File (*.obj)", false);
                    try (InputStream stream = new FileInputStream(filename)) {
                        geom = loadInputMesh(stream);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            } else if (settingsUI.isBuildTriggered()) {
                if (!building) {

                    float m_cellSize = settingsUI.getCellSize();
                    float m_cellHeight = settingsUI.getCellHeight();
                    float m_agentHeight = settingsUI.getAgentHeight();
                    float m_agentRadius = settingsUI.getAgentRadius();
                    float m_agentMaxClimb = settingsUI.getAgentMaxClimb();
                    float m_agentMaxSlope = settingsUI.getAgentMaxSlope();
                    int m_regionMinSize = settingsUI.getMinRegionSize();
                    int m_regionMergeSize = settingsUI.getMergedRegionSize();
                    float m_edgeMaxLen = settingsUI.getEdgeMaxLen();
                    float m_edgeMaxError = settingsUI.getEdgeMaxError();
                    int m_vertsPerPoly = settingsUI.getVertsPerPoly();
                    float m_detailSampleDist = settingsUI.getDetailSampleDist();
                    float m_detailSampleMaxError = settingsUI.getDetailSampleMaxError();
                    int m_tileSize = settingsUI.getTileSize();
                    long t = System.nanoTime();

                    Tupple2<List<RecastBuilderResult>, NavMesh> buildResult;
                    if (settingsUI.isTiled()) {
                        buildResult = tileNavMeshBuilder.build(geom, settingsUI.getPartitioning(), m_cellSize,
                                m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope,
                                m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly,
                                m_detailSampleDist, m_detailSampleMaxError, settingsUI.isFilterLowHangingObstacles(),
                                settingsUI.isFilterLedgeSpans(), settingsUI.isFilterWalkableLowHeightSpans(),
                                m_tileSize);

                    } else {
                        buildResult = soloNavMeshBuilder.build(geom, settingsUI.getPartitioning(), m_cellSize,
                                m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope,
                                m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly,
                                m_detailSampleDist, m_detailSampleMaxError, settingsUI.isFilterLowHangingObstacles(),
                                settingsUI.isFilterLedgeSpans(), settingsUI.isFilterWalkableLowHeightSpans());
                    }
                    navMesh = buildResult.second;
                    sample = new Sample(geom, buildResult.first, navMesh, settingsUI, dd);
                    settingsUI.setBuildTime((System.nanoTime() - t) / 1_000_000);
                    toolsUI.setSample(sample);
                }
            } else {
                building = false;
            }

            if (!mouseOverMenu) {
                GLU.glhUnProjectf(mousePos[0], viewport[3] - 1 - mousePos[1], 0.0f, modelviewMatrix, projectionMatrix,
                        viewport, rayStart);
                GLU.glhUnProjectf(mousePos[0], viewport[3] - 1 - mousePos[1], 1.0f, modelviewMatrix, projectionMatrix,
                        viewport, rayEnd);

                // Hit test mesh.
                sample.getInputGeom().raycastMesh(rayStart, rayEnd);
                if (processHitTest && sample != null) {
                    Optional<Float> hit = sample.getInputGeom().raycastMesh(rayStart, rayEnd);
                    if (hit.isPresent()) {
                        float hitTime = hit.get();
                        if ((modState & GLFW_MOD_CONTROL) != 0) {
                            // Marker
                            markerPositionSet = true;
                            markerPosition[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
                            markerPosition[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
                            markerPosition[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
                        } else {
                            float[] pos = new float[3];
                            pos[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
                            pos[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
                            pos[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
                            Tool tool = toolsUI.getTool();
                            if (tool != null) {
                                tool.handleClick(rayStart, pos, processHitTestShift);
                            }
                        }
                    } else {
                        if ((modState & GLFW_MOD_CONTROL) != 0) {
                            // Marker
                            markerPositionSet = false;
                        }
                    }
                }
                processHitTest = false;
            }

            // Draw bounds
            float[] bmin = geom.getMeshBoundsMin();
            float[] bmax = geom.getMeshBoundsMax();

            if (showSample) {
                camr = (float) (Math.sqrt(DemoMath.sqr(bmax[0] - bmin[0]) + DemoMath.sqr(bmax[1] - bmin[1])
                        + DemoMath.sqr(bmax[2] - bmin[2])) / 2);
                cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
                cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
                cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
                camr *= 3;
                cameraEulers[0] = 45;
                cameraEulers[1] = -45;

                showSample = false;
            }
            dd.fog(camr * 0.1f, camr * 1.25f);
            renderer.render(sample);
            Tool tool = toolsUI.getTool();
            if (tool != null) {
                tool.handleRender(renderer);
            }
            dd.fog(false);

            // Render GUI
            nuklearUI.render(window);
            glfwSwapBuffers(window);
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
            }
        }

    }

    private DemoInputGeomProvider loadInputMesh(InputStream stream) {
        DemoInputGeomProvider geom = new ObjImporter().load(stream);
        sample = new Sample(geom, Collections.emptyList(), null, settingsUI, dd);
        toolsUI.setSample(sample);
        toolsUI.setEnabled(true);
        showSample = true;
        return geom;
    }

    public static void main(String[] args) {
        new RecastDemo().start();
    }
}
