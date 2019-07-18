package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.nk_button_text;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_text;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

import org.lwjgl.BufferUtils;
import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.draw.DebugDraw;
import org.recast4j.demo.draw.DebugDrawPrimitives;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.sample.Sample;
import org.recast4j.demo.tool.annotation.AnnotationBuilder;
import org.recast4j.demo.tool.annotation.AnnotationBuilder.EdgeSamplerType;
import org.recast4j.demo.tool.annotation.AnnotationBuilderConfig;
import org.recast4j.demo.tool.annotation.EdgeSampler;
import org.recast4j.demo.tool.annotation.JumpLink;
import org.recast4j.demo.tool.annotation.Trajectory2D;

public class AnnotationBuilderTool implements Tool {

    public static final int DRAW_WALKABLE_SURFACE = 1 << 0;
    public static final int DRAW_WALKABLE_BORDER = 1 << 1;
    public static final int DRAW_SELECTED_EDGE = 1 << 2;
    public static final int DRAW_ANIM_TRAJECTORY = 1 << 3;
    public static final int DRAW_LAND_SAMPLES = 1 << 4;
    public static final int DRAW_COLLISION_SLICES = 1 << 5;
    public static final int DRAW_ANNOTATIONS = 1 << 6;

    private final List<JumpLink> links = new ArrayList<>();
    private Sample sample;
    private int flags = DRAW_WALKABLE_SURFACE | DRAW_WALKABLE_BORDER | DRAW_SELECTED_EDGE | DRAW_ANIM_TRAJECTORY
            | DRAW_LAND_SAMPLES | DRAW_ANNOTATIONS;
    private final FloatBuffer startDistance = BufferUtils.createFloatBuffer(1).put(0, -0.25f);
    private final FloatBuffer endDistance = BufferUtils.createFloatBuffer(1).put(0, 2f);
    private final FloatBuffer jumpDownDistance = BufferUtils.createFloatBuffer(1).put(0, 3f);
    private final FloatBuffer groundRange = BufferUtils.createFloatBuffer(1).put(0, 0.5f);
    private AnnotationBuilder annotationBuilder;
    private final int selEdge = -1;

    @Override
    public void setSample(Sample sample) {
        this.sample = sample;
        annotationBuilder = null;
        if (!this.sample.getRecastResults().isEmpty()) {
            annotationBuilder = new AnnotationBuilder(sample.getRecastResults(), sample.getNavMesh(), sample.getNavMeshQuery());
        }
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {

    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        int col0 = DebugDraw.duLerpCol(DebugDraw.duRGBA(32, 255, 96, 255), DebugDraw.duRGBA(255, 255, 255, 255), 200);
        int col1 = DebugDraw.duRGBA(32, 255, 96, 255);
        RecastDebugDraw dd = renderer.getDebugDraw();

        if ((flags & DRAW_WALKABLE_BORDER) != 0) {
            if (annotationBuilder != null) {
                dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                for (int i = 0; i < annotationBuilder.getEdges().length; ++i) {
                    int col = DebugDraw.duRGBA(0, 96, 128, 255);
                    if (i == selEdge)
                        continue;
                    dd.vertex(annotationBuilder.getEdges()[i].sp, col);
                    dd.vertex(annotationBuilder.getEdges()[i].sq, col);
                }
                dd.end();

                dd.begin(DebugDrawPrimitives.POINTS, 8.0f);
                for (int i = 0; i < annotationBuilder.getEdges().length; ++i) {
                    int col = DebugDraw.duRGBA(0, 96, 128, 255);
                    if (i == selEdge)
                        continue;
                    dd.vertex(annotationBuilder.getEdges()[i].sp, col);
                    dd.vertex(annotationBuilder.getEdges()[i].sq, col);
                }
                dd.end();

                if (selEdge >= 0 && selEdge < annotationBuilder.getEdges().length) {
                    int col = DebugDraw.duRGBA(48, 16, 16, 255); // duRGBA(255,192,0,255);
                    dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                    dd.vertex(annotationBuilder.getEdges()[selEdge].sp, col);
                    dd.vertex(annotationBuilder.getEdges()[selEdge].sq, col);
                    dd.end();
                    dd.begin(DebugDrawPrimitives.POINTS, 8.0f);
                    dd.vertex(annotationBuilder.getEdges()[selEdge].sp, col);
                    dd.vertex(annotationBuilder.getEdges()[selEdge].sq, col);
                    dd.end();
                }

                dd.begin(DebugDrawPrimitives.POINTS, 4.0f);
                for (int i = 0; i < annotationBuilder.getEdges().length; ++i) {
                    int col = DebugDraw.duRGBA(190, 190, 190, 255);
                    dd.vertex(annotationBuilder.getEdges()[i].sp, col);
                    dd.vertex(annotationBuilder.getEdges()[i].sq, col);
                }
                dd.end();
            }
        }

        if ((flags & DRAW_ANNOTATIONS) != 0) {
            dd.begin(DebugDrawPrimitives.QUADS);
            for (JumpLink link : links) {
                if (link.flags == 0) {
                    continue;
                }
                for (int j = 0; j < link.nspine - 1; ++j) {
                    int u = (j * 255) / link.nspine;
                    int col = DebugDraw.duTransCol(DebugDraw.duLerpCol(col0, col1, u), 128);
                    if (link.flags == 0) {
                        col = DebugDraw.duRGBA(255, 0, 0, 64);
                    }
                    dd.vertex(link.spine1[j * 3], link.spine1[j * 3 + 1], link.spine1[j * 3 + 2], col);
                    dd.vertex(link.spine1[(j + 1) * 3], link.spine1[(j + 1) * 3 + 1], link.spine1[(j + 1) * 3 + 2],
                            col);
                    dd.vertex(link.spine0[(j + 1) * 3], link.spine0[(j + 1) * 3 + 1], link.spine0[(j + 1) * 3 + 2],
                            col);
                    dd.vertex(link.spine0[j * 3], link.spine0[j * 3 + 1], link.spine0[j * 3 + 2], col);
                }
            }
            dd.end();
            dd.begin(DebugDrawPrimitives.LINES, 3.0f);
            for (JumpLink link : links) {
                if (link.flags == 0) {
                    continue;
                }
                for (int j = 0; j < link.nspine - 1; ++j) {
                    // int u = (j*255)/link.nspine;
                    int col = DebugDraw.duTransCol(DebugDraw.duDarkenCol(col1)/*duDarkenCol(duLerpCol(col0,col1,u))*/,
                            128);

                    dd.vertex(link.spine0[j * 3], link.spine0[j * 3 + 1], link.spine0[j * 3 + 2], col);
                    dd.vertex(link.spine0[(j + 1) * 3], link.spine0[(j + 1) * 3 + 1], link.spine0[(j + 1) * 3 + 2],
                            col);
                    dd.vertex(link.spine1[j * 3], link.spine1[j * 3 + 1], link.spine1[j * 3 + 2], col);
                    dd.vertex(link.spine1[(j + 1) * 3], link.spine1[(j + 1) * 3 + 1], link.spine1[(j + 1) * 3 + 2],
                            col);
                }

                dd.vertex(link.spine0[0], link.spine0[1], link.spine0[2], DebugDraw.duDarkenCol(col1));
                dd.vertex(link.spine1[0], link.spine1[1], link.spine1[2], DebugDraw.duDarkenCol(col1));

                dd.vertex(link.spine0[(link.nspine - 1) * 3], link.spine0[(link.nspine - 1) * 3 + 1],
                        link.spine0[(link.nspine - 1) * 3 + 2], DebugDraw.duDarkenCol(col1));
                dd.vertex(link.spine1[(link.nspine - 1) * 3], link.spine1[(link.nspine - 1) * 3 + 1],
                        link.spine1[(link.nspine - 1) * 3 + 2], DebugDraw.duDarkenCol(col1));
            }
            dd.end();
        }
        if (annotationBuilder != null) {
            EdgeSampler es = annotationBuilder.edgeSampler;
            if (es != null && (flags & DRAW_ANIM_TRAJECTORY) != 0) {
                float r = es.groundRange;

                int col = DebugDraw.duLerpCol(DebugDraw.duRGBA(255, 192, 0, 255), DebugDraw.duRGBA(255, 255, 255, 255),
                        64);
                int cola = DebugDraw.duTransCol(col, 192);
                int colb = DebugDraw.duRGBA(255, 255, 255, 255);

                // Start segment.
                dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                dd.vertex(es.start.p[0], es.start.p[1], es.start.p[2], col);
                dd.vertex(es.start.q[0], es.start.q[1], es.start.q[2], col);
                dd.end();

                dd.begin(DebugDrawPrimitives.LINES, 1.0f);
                dd.vertex(es.start.p[0], es.start.p[1] - r, es.start.p[2], colb);
                dd.vertex(es.start.p[0], es.start.p[1] + r, es.start.p[2], colb);
                dd.vertex(es.start.p[0], es.start.p[1] + r, es.start.p[2], colb);
                dd.vertex(es.start.q[0], es.start.q[1] + r, es.start.q[2], colb);
                dd.vertex(es.start.q[0], es.start.q[1] + r, es.start.q[2], colb);
                dd.vertex(es.start.q[0], es.start.q[1] - r, es.start.q[2], colb);
                dd.vertex(es.start.q[0], es.start.q[1] - r, es.start.q[2], colb);
                dd.vertex(es.start.p[0], es.start.p[1] - r, es.start.p[2], colb);
                dd.end();

                // End segment.
                dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                dd.vertex(es.end.p[0], es.end.p[1], es.end.p[2], col);
                dd.vertex(es.end.q[0], es.end.q[1], es.end.q[2], col);
                dd.end();

                dd.begin(DebugDrawPrimitives.LINES, 1.0f);
                dd.vertex(es.end.p[0], es.end.p[1] - r, es.end.p[2], colb);
                dd.vertex(es.end.p[0], es.end.p[1] + r, es.end.p[2], colb);
                dd.vertex(es.end.p[0], es.end.p[1] + r, es.end.p[2], colb);
                dd.vertex(es.end.q[0], es.end.q[1] + r, es.end.q[2], colb);
                dd.vertex(es.end.q[0], es.end.q[1] + r, es.end.q[2], colb);
                dd.vertex(es.end.q[0], es.end.q[1] - r, es.end.q[2], colb);
                dd.vertex(es.end.q[0], es.end.q[1] - r, es.end.q[2], colb);
                dd.vertex(es.end.p[0], es.end.p[1] - r, es.end.p[2], colb);
                dd.end();

                dd.begin(DebugDrawPrimitives.LINES, 4.0f);
                drawTrajectory(dd, es.start.p, es.end.p, es.trajectory, cola);
                // drawTrajectory(dd, es.start.q, es.end.q, es.trajectory, cola);
                dd.end();

                dd.begin(DebugDrawPrimitives.LINES, 8.0f);
                dd.vertex(es.start.p[0], es.start.p[1], es.start.p[2], DebugDraw.duDarkenCol(col));
                dd.vertex(es.start.q[0], es.start.q[1], es.start.q[2], DebugDraw.duDarkenCol(col));
                dd.vertex(es.end.p[0], es.end.p[1], es.end.p[2], DebugDraw.duDarkenCol(col));
                dd.vertex(es.end.q[0], es.end.q[1], es.end.q[2], DebugDraw.duDarkenCol(col));
                dd.end();

                int colm = DebugDraw.duRGBA(255, 255, 255, 255);
                dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                dd.vertex(es.start.p[0], es.start.p[1], es.start.p[2], colm);
                dd.vertex(es.start.q[0], es.start.q[1], es.start.q[2], colm);
                dd.vertex(es.end.p[0], es.end.p[1], es.end.p[2], colm);
                dd.vertex(es.end.q[0], es.end.q[1], es.end.q[2], colm);
                dd.end();
            }
        }
    }

    private void drawTrajectory(RecastDebugDraw dd, float[] p, float[] p2, Trajectory2D tra, int cola) {
        {
        //  float pa[3], pb[3];
        //  float sa[2], sb[2];

            float startx = tra.spineXOffset.apply(0f);
            float endx = tra.spineXOffset.apply(1f);
            float deltax = endx - startx;

            float starty = tra.spineYMix.apply(0f);
            float endy = tra.spineYMix.apply(1f);


//            float[] pts= new float[3*MAX_SPINE];
//            int npts = tra.nspine;
//            for (int i = 0; i < tra.nspine; ++i)
//            {
//                float* spt = &tra.spine[i*2];
//                float u = (spt[0] - startx)/deltax;
//                float dy = spt[1] - lerp(starty,endy,u);
//                float* p = &pts[i*3];
//                vlerp(p, pa,pb,u);
//                p[1] += dy;
//            }
//
//            float len = getPathLen(pts, npts);
//            int nsegs = ceilf(len / 0.3f);
//
//            for (int i = 0; i < nsegs*2; ++i)
//            {
//                float u = (float)i / (float)(nsegs*2);
//                float pt[3];
//                getPointAlongPath(u*len, pts, npts, pt);
//                dd.vertex(pt, col);
//            }
//


            // Build start spine.
        /*  float p0[3], p1[3];
            for (int i = 0; i < tra.nspine; ++i)
            {
                float* spt = &tra.spine[i*2];
                float u = (spt[0] - startx)/deltax;
                float dy = spt[1] - lerp(starty,endy,u);
                vlerp(p1, pa,pb,u);
                p1[1] += dy;
//              if (i > 0)
                {
                    dd.vertex(p0, col);
//                  dd.vertex(p1, col);
                }
                vcopy(p0,p1);
            }*/
        }
    }

    @Override
    public void handleUpdate(float dt) {

    }

    @Override
    public void layout(NkContext ctx) {
        if (sample != null && !sample.getRecastResults().isEmpty()) {

            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Options", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Start Distance", -1f, startDistance, 0f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "End Distance", 0f, endDistance, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Down Distance", 0f, jumpDownDistance, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Ground Range", 0f, groundRange, 5f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            if (nk_button_text(ctx, "Build All Jump-down")) {
                links.clear();
                if (annotationBuilder != null) {
                    float cellSize = sample.getSettingsUI().getCellSize();
                    float agentHeight = sample.getSettingsUI().getAgentHeight();
                    float agentRadius = sample.getSettingsUI().getAgentRadius();
                    float agentMaxClimb = sample.getSettingsUI().getAgentMaxClimb();
                    AnnotationBuilderConfig config = new AnnotationBuilderConfig(cellSize, agentRadius, agentMaxClimb,
                            agentHeight, startDistance.get(0), endDistance.get(0), jumpDownDistance.get(0),
                            groundRange.get(0));
                    links.addAll(annotationBuilder.buildAllEdges(config, EdgeSamplerType.EDGE_JUMP_DOWN));
                }
            }

            nk_spacing(ctx, 1);
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Draw Options", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            int newFlags = 0;
            newFlags |= nk_option_text(ctx, "Walkable Border", (flags & DRAW_WALKABLE_BORDER) != 0)
                    ? DRAW_WALKABLE_BORDER
                    : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "Selected Edge", (flags & DRAW_SELECTED_EDGE) != 0) ? DRAW_SELECTED_EDGE
                    : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "Anim Trajectory", (flags & DRAW_ANIM_TRAJECTORY) != 0)
                    ? DRAW_ANIM_TRAJECTORY
                    : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "All Annotations", (flags & DRAW_ANNOTATIONS) != 0) ? DRAW_ANNOTATIONS : 0;
            flags = newFlags;
        }

    }

    @Override
    public String getName() {
        return "Annotation Builder";
    }

}
