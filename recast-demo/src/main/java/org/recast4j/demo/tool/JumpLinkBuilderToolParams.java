package org.recast4j.demo.tool;

import java.nio.FloatBuffer;

import org.lwjgl.BufferUtils;
import org.recast4j.detour.extras.jumplink.JumpLinkType;

public class JumpLinkBuilderToolParams {

    public static final int DRAW_WALKABLE_SURFACE = 1 << 0;
    public static final int DRAW_WALKABLE_BORDER = 1 << 1;
    public static final int DRAW_SELECTED_EDGE = 1 << 2;
    public static final int DRAW_ANIM_TRAJECTORY = 1 << 3;
    public static final int DRAW_LAND_SAMPLES = 1 << 4;
    public static final int DRAW_COLLISION_SLICES = 1 << 5;
    public static final int DRAW_ANNOTATIONS = 1 << 6;

    int flags = DRAW_WALKABLE_SURFACE | DRAW_WALKABLE_BORDER | DRAW_SELECTED_EDGE | DRAW_ANIM_TRAJECTORY
            | DRAW_LAND_SAMPLES | DRAW_ANNOTATIONS;
    final FloatBuffer groundTolerance = BufferUtils.createFloatBuffer(1).put(0, 0.3f);
    final FloatBuffer climbDownDistance = BufferUtils.createFloatBuffer(1).put(0, 0.4f);
    final FloatBuffer climbDownMaxHeight = BufferUtils.createFloatBuffer(1).put(0, 3.2f);
    final FloatBuffer climbDownMinHeight = BufferUtils.createFloatBuffer(1).put(0, 1.5f);
    final FloatBuffer edgeJumpEndDistance = BufferUtils.createFloatBuffer(1).put(0, 2f);
    final FloatBuffer edgeJumpHeight = BufferUtils.createFloatBuffer(1).put(0, 0.4f);
    final FloatBuffer edgeJumpDownMaxHeight = BufferUtils.createFloatBuffer(1).put(0, 2.5f);
    final FloatBuffer edgeJumpUpMaxHeight = BufferUtils.createFloatBuffer(1).put(0, 0.3f);
    int buildTypes = (1 << JumpLinkType.EDGE_CLIMB_DOWN.ordinal()) | (1 << JumpLinkType.EDGE_JUMP.ordinal());

}
