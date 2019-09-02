package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.nk_button_text;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_text;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.draw.DebugDraw;
import org.recast4j.demo.draw.DebugDrawPrimitives;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.sample.Sample;
import org.recast4j.demo.tool.jumplink.AnnotationBuilderConfig;
import org.recast4j.demo.tool.jumplink.EdgeSampler;
import org.recast4j.demo.tool.jumplink.JumpLink;
import org.recast4j.demo.tool.jumplink.JumpLinkBuilder;
import org.recast4j.demo.tool.jumplink.JumpLinkBuilder.EdgeSamplerType;
import org.recast4j.demo.tool.jumplink.Trajectory2D;

public class JumpLinkBuilderTool implements Tool {

    private final List<JumpLink> links = new ArrayList<>();
    private Sample sample;
    private JumpLinkBuilder annotationBuilder;
    private final int selEdge = -1;
    private final JumpLinkBuilderToolParams params = new JumpLinkBuilderToolParams();

    @Override
    public void setSample(Sample sample) {
        this.sample = sample;
        annotationBuilder = null;
        if (!this.sample.getRecastResults().isEmpty()) {
            annotationBuilder = new JumpLinkBuilder(sample.getRecastResults());
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

        if ((params.flags & JumpLinkBuilderToolParams.DRAW_WALKABLE_BORDER) != 0) {
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

        if ((params.flags & JumpLinkBuilderToolParams.DRAW_ANNOTATIONS) != 0) {
            dd.begin(DebugDrawPrimitives.QUADS);
            for (JumpLink link : links) {
                for (int j = 0; j < link.nspine - 1; ++j) {
                    int u = (j * 255) / link.nspine;
                    int col = DebugDraw.duTransCol(DebugDraw.duLerpCol(col0, col1, u), 128);
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
            for (JumpLink link : links) {
                EdgeSampler es = link.esgeSampler;
                if (es != null && (params.flags & JumpLinkBuilderToolParams.DRAW_ANIM_TRAJECTORY) != 0) {
                    float r = es.heightRange;

                    int col = DebugDraw.duLerpCol(DebugDraw.duRGBA(255, 192, 0, 255),
                            DebugDraw.duRGBA(255, 255, 255, 255), 64);
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
                    drawTrajectory(dd, es.start.q, es.end.q, es.trajectory, cola);
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
    }

    private void drawTrajectory(RecastDebugDraw dd, float[] pa, float[] pb, Trajectory2D tra, int cola) {
        {
            float starty = tra.spineYMix.apply(0f);
            float endy = tra.spineYMix.apply(1f);

            float pts[] = new float[3 * JumpLink.MAX_SPINE];
            for (int i = 0; i < JumpLink.MAX_SPINE; i++) {

            }
            // int npts = tra->nspine;
            // for (int i = 0; i < tra->nspine; ++i)
            // {
            // const float* spt = &tra->spine[i*2];
            // const float u = (spt[0] - startx)/deltax;
            // const float dy = spt[1] - lerp(starty,endy,u);
            // float* p = &pts[i*3];
            // vlerp(p, pa,pb,u);
            // p[1] += dy;
            // }

            // int npts = JumpLink.MAX_SPINE;
            // float[] pts= new float[3*npts];
            // for (int i = 0; i < npts; ++i)
            // {
            // float spt = (float) i / (float) (npts - 1);
            // float u = tra.spineXRemap.apply(spt);
            // float dy = lerp(pa[1], pb[1], tra.spineYMix.apply(u));
            // pts[i * 3] = lerp(pa[0], pb[0], u);
            // pts[i * 3] = lerp(pa[0], pb[0], u);
            // pts[i * 3 + 2] = lerp(pa[0], pb[0], u);
            //
            //
            // float dy = lerp(sp[1], ep[1], es.trajectory.spineYMix.apply(u));
            // float[] p = vLerp(sp, ep, u);
            // p[1] = acfg.agentClimb;
            // p = vMad(p, es.ay, dy);
            // link.spine0[j * 3] = p[0];
            // link.spine0[j * 3 + 1] = p[1];
            // link.spine0[j * 3 + 2] = p[2];
            //
            // float dy = spt[1] - lerp(starty,endy,u);
            // float* p = &pts[i*3];
            // vlerp(p, pa,pb,u);
            // p[1] += dy;
            // }
            ////
            // float len = getPathLen(pts, npts);
            // int nsegs = ceilf(len / 0.3f);
            //
            // for (int i = 0; i < nsegs*2; ++i)
            // {
            // float u = (float)i / (float)(nsegs*2);
            // float pt[3];
            // getPointAlongPath(u*len, pts, npts, pt);
            // dd.vertex(pt, col);
            // }
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
            nk_property_float(ctx, "Ground Tolerance", 0f, params.groundTolerance, 2f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 5, 1);
            nk_spacing(ctx, 1);

            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Climb Down", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Distance", 0f, params.climbDownDistance, 5f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Max Height", 0f, params.climbDownMaxHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Min Height", 0f, params.climbDownMinHeight, 10f, 0.05f, 0.01f);
            // nk_layout_row_dynamic(ctx, 5, 1);
            // nk_spacing(ctx, 1);
            // nk_layout_row_dynamic(ctx, 18, 1);
            // nk_label(ctx, "Climb Up", NK_TEXT_ALIGN_LEFT);
            // nk_layout_row_dynamic(ctx, 20, 1);
            // nk_property_float(ctx, "Start Distance", -5f, startDistance, 5f, 0.05f, 0.01f);
            // nk_layout_row_dynamic(ctx, 20, 1);
            // nk_property_float(ctx, "End Distance", 0f, endDistance, 10f, 0.05f, 0.01f);
            // nk_layout_row_dynamic(ctx, 20, 1);
            // nk_property_float(ctx, "Max Down Distance", 0f, maxDownDistance, 100f, 0.05f, 0.01f);
            // nk_layout_row_dynamic(ctx, 20, 1);
            // nk_property_float(ctx, "Min Down Distance", 0f, minDownDistance, 100f, 0.05f, 0.01f);
            // nk_layout_row_dynamic(ctx, 20, 1);
            nk_layout_row_dynamic(ctx, 5, 1);
            nk_spacing(ctx, 1);

            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Jump Down", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Start Distance", -5f, params.edgeJumpStartDistance, 5f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "End Distance", 0f, params.edgeJumpEndDistance, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Jump Height", 0f, params.edgeJumpHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Max Down Distance", 0f, params.edgeJumpDownMaxHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Min Down Distance", 0f, params.edgeJumpDownMinHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 5, 1);
            nk_spacing(ctx, 1);
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Links", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            int buildTypes = 0;
            buildTypes |= nk_option_text(ctx, "Climb Down",
                    (params.buildTypes & (1 << EdgeSamplerType.EDGE_CLIMB_DOWN.ordinal())) != 0)
                            ? (1 << EdgeSamplerType.EDGE_CLIMB_DOWN.ordinal())
                            : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            buildTypes |= nk_option_text(ctx, "Edge Jump",
                    (params.buildTypes & (1 << EdgeSamplerType.EDGE_JUMP.ordinal())) != 0)
                            ? (1 << EdgeSamplerType.EDGE_JUMP.ordinal())
                            : 0;
            params.buildTypes = buildTypes;
            if (nk_button_text(ctx, "Build")) {
                links.clear();
                if (annotationBuilder != null) {
                    float cellSize = sample.getSettingsUI().getCellSize();
                    float agentHeight = sample.getSettingsUI().getAgentHeight();
                    float agentRadius = sample.getSettingsUI().getAgentRadius();
                    float cellHeight = sample.getSettingsUI().getCellHeight();
                    if ((buildTypes & (1 << EdgeSamplerType.EDGE_CLIMB_DOWN.ordinal())) != 0) {
                        AnnotationBuilderConfig config = new AnnotationBuilderConfig(cellSize, cellHeight, agentRadius,
                                params.groundTolerance.get(0), agentHeight, 0f,
                                cellSize + 2 * agentRadius + params.climbDownDistance.get(0),
                                params.climbDownMaxHeight.get(0), params.climbDownMinHeight.get(0), 0);
                        links.addAll(annotationBuilder.build(config, EdgeSamplerType.EDGE_CLIMB_DOWN));
                    }
                    if ((buildTypes & (1 << EdgeSamplerType.EDGE_JUMP.ordinal())) != 0) {
                        AnnotationBuilderConfig config = new AnnotationBuilderConfig(cellSize, cellHeight, agentRadius,
                                params.groundTolerance.get(0), agentHeight, params.edgeJumpStartDistance.get(0),
                                params.edgeJumpEndDistance.get(0), params.edgeJumpDownMaxHeight.get(0),
                                params.edgeJumpDownMinHeight.get(0), params.edgeJumpHeight.get(0));
                        links.addAll(annotationBuilder.build(config, EdgeSamplerType.EDGE_JUMP));
                    }
                }
            }
            nk_spacing(ctx, 1);
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Debug Draw Options", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            int newFlags = 0;
            newFlags |= nk_option_text(ctx, "Walkable Border",
                    (params.flags & JumpLinkBuilderToolParams.DRAW_WALKABLE_BORDER) != 0)
                            ? JumpLinkBuilderToolParams.DRAW_WALKABLE_BORDER
                            : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "Selected Edge",
                    (params.flags & JumpLinkBuilderToolParams.DRAW_SELECTED_EDGE) != 0)
                            ? JumpLinkBuilderToolParams.DRAW_SELECTED_EDGE
                            : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "Anim Trajectory",
                    (params.flags & JumpLinkBuilderToolParams.DRAW_ANIM_TRAJECTORY) != 0)
                            ? JumpLinkBuilderToolParams.DRAW_ANIM_TRAJECTORY
                            : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "All Annotations",
                    (params.flags & JumpLinkBuilderToolParams.DRAW_ANNOTATIONS) != 0)
                            ? JumpLinkBuilderToolParams.DRAW_ANNOTATIONS
                            : 0;
            params.flags = newFlags;
        }

    }

    @Override
    public String getName() {
        return "Annotation Builder";
    }

}
