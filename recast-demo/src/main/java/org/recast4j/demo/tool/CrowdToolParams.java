/*
recast4j copyright (c) 2020-2021 Piotr Piastucki piotr@jtilia.org

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
