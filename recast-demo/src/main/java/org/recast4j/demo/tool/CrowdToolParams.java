package org.recast4j.demo.tool;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import org.lwjgl.BufferUtils;

public class CrowdToolParams {

    final IntBuffer m_expandSelectedDebugDraw = BufferUtils.createIntBuffer(1).put(0, 1);
    boolean m_showCorners;
    boolean m_showCollisionSegments;
    boolean m_showPath;
    boolean m_showVO;
    boolean m_showOpt;
    boolean m_showNeis;

    final IntBuffer m_expandDebugDraw = BufferUtils.createIntBuffer(1).put(0, 0);
    boolean m_showLabels;
    boolean m_showGrid;
    boolean m_showNodes;
    boolean m_showPerfGraph;
    boolean m_showDetailAll;

    final IntBuffer m_expandOptions = BufferUtils.createIntBuffer(1).put(0, 1);
    boolean m_anticipateTurns = true;
    boolean m_optimizeVis = true;
    boolean m_optimizeTopo = true;
    boolean m_obstacleAvoidance = true;
    final IntBuffer m_obstacleAvoidanceType = BufferUtils.createIntBuffer(1).put(0, 3);
    boolean m_separation;
    final FloatBuffer m_separationWeight = BufferUtils.createFloatBuffer(1).put(0, 2f);
}
