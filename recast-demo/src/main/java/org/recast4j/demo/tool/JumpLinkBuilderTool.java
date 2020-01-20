package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.nk_button_text;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_text;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;
import static org.recast4j.demo.draw.DebugDraw.duDarkenCol;
import static org.recast4j.demo.draw.DebugDraw.duLerpCol;
import static org.recast4j.demo.draw.DebugDraw.duRGBA;
import static org.recast4j.demo.draw.DebugDraw.duTransCol;
import static org.recast4j.demo.draw.DebugDrawPrimitives.LINES;
import static org.recast4j.demo.draw.DebugDrawPrimitives.POINTS;
import static org.recast4j.demo.draw.DebugDrawPrimitives.QUADS;
import static org.recast4j.detour.DetourCommon.vDist2D;
import static org.recast4j.detour.DetourCommon.vLerp;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.sample.Sample;
import org.recast4j.detour.extras.jumplink.Edge;
import org.recast4j.detour.extras.jumplink.GroundSample;
import org.recast4j.detour.extras.jumplink.GroundSegment;
import org.recast4j.detour.extras.jumplink.JumpLink;
import org.recast4j.detour.extras.jumplink.JumpLinkBuilder;
import org.recast4j.detour.extras.jumplink.JumpLinkBuilderConfig;
import org.recast4j.detour.extras.jumplink.JumpLinkType;
import org.recast4j.detour.extras.jumplink.Trajectory;

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
        int col0 = duLerpCol(duRGBA(32, 255, 96, 255), duRGBA(255, 255, 255, 255), 200);
        int col1 = duRGBA(32, 255, 96, 255);
        RecastDebugDraw dd = renderer.getDebugDraw();
        dd.depthMask(false);

        if ((params.flags & JumpLinkBuilderToolParams.DRAW_WALKABLE_BORDER) != 0) {
            if (annotationBuilder != null) {
                for (Edge[] edges : annotationBuilder.getEdges()) {
                    dd.begin(LINES, 3.0f);
                    for (int i = 0; i < edges.length; ++i) {
                        int col = duRGBA(0, 96, 128, 255);
                        if (i == selEdge)
                            continue;
                        dd.vertex(edges[i].sp, col);
                        dd.vertex(edges[i].sq, col);
                    }
                    dd.end();

                    dd.begin(POINTS, 8.0f);
                    for (int i = 0; i < edges.length; ++i) {
                        int col = duRGBA(0, 96, 128, 255);
                        if (i == selEdge)
                            continue;
                        dd.vertex(edges[i].sp, col);
                        dd.vertex(edges[i].sq, col);
                    }
                    dd.end();

                    if (selEdge >= 0 && selEdge < edges.length) {
                        int col = duRGBA(48, 16, 16, 255); // duRGBA(255,192,0,255);
                        dd.begin(LINES, 3.0f);
                        dd.vertex(edges[selEdge].sp, col);
                        dd.vertex(edges[selEdge].sq, col);
                        dd.end();
                        dd.begin(POINTS, 8.0f);
                        dd.vertex(edges[selEdge].sp, col);
                        dd.vertex(edges[selEdge].sq, col);
                        dd.end();
                    }

                    dd.begin(POINTS, 4.0f);
                    for (int i = 0; i < edges.length; ++i) {
                        int col = duRGBA(190, 190, 190, 255);
                        dd.vertex(edges[i].sp, col);
                        dd.vertex(edges[i].sq, col);
                    }
                    dd.end();
                }
            }
        }

        if ((params.flags & JumpLinkBuilderToolParams.DRAW_ANNOTATIONS) != 0) {
            dd.begin(QUADS);
            for (JumpLink link : links) {
                for (int j = 0; j < link.nspine - 1; ++j) {
                    int u = (j * 255) / link.nspine;
                    int col = duTransCol(duLerpCol(col0, col1, u), 128);
                    dd.vertex(link.spine1[j * 3], link.spine1[j * 3 + 1], link.spine1[j * 3 + 2], col);
                    dd.vertex(link.spine1[(j + 1) * 3], link.spine1[(j + 1) * 3 + 1], link.spine1[(j + 1) * 3 + 2],
                            col);
                    dd.vertex(link.spine0[(j + 1) * 3], link.spine0[(j + 1) * 3 + 1], link.spine0[(j + 1) * 3 + 2],
                            col);
                    dd.vertex(link.spine0[j * 3], link.spine0[j * 3 + 1], link.spine0[j * 3 + 2], col);
                }
            }
            dd.end();
            dd.begin(LINES, 3.0f);
            for (JumpLink link : links) {
                for (int j = 0; j < link.nspine - 1; ++j) {
                    // int u = (j*255)/link.nspine;
                    int col = duTransCol(duDarkenCol(col1)/*duDarkenCol(duLerpCol(col0,col1,u))*/, 128);

                    dd.vertex(link.spine0[j * 3], link.spine0[j * 3 + 1], link.spine0[j * 3 + 2], col);
                    dd.vertex(link.spine0[(j + 1) * 3], link.spine0[(j + 1) * 3 + 1], link.spine0[(j + 1) * 3 + 2],
                            col);
                    dd.vertex(link.spine1[j * 3], link.spine1[j * 3 + 1], link.spine1[j * 3 + 2], col);
                    dd.vertex(link.spine1[(j + 1) * 3], link.spine1[(j + 1) * 3 + 1], link.spine1[(j + 1) * 3 + 2],
                            col);
                }

                dd.vertex(link.spine0[0], link.spine0[1], link.spine0[2], duDarkenCol(col1));
                dd.vertex(link.spine1[0], link.spine1[1], link.spine1[2], duDarkenCol(col1));

                dd.vertex(link.spine0[(link.nspine - 1) * 3], link.spine0[(link.nspine - 1) * 3 + 1],
                        link.spine0[(link.nspine - 1) * 3 + 2], duDarkenCol(col1));
                dd.vertex(link.spine1[(link.nspine - 1) * 3], link.spine1[(link.nspine - 1) * 3 + 1],
                        link.spine1[(link.nspine - 1) * 3 + 2], duDarkenCol(col1));
            }
            dd.end();
        }
        if (annotationBuilder != null) {
            for (JumpLink link : links) {
                if ((params.flags & JumpLinkBuilderToolParams.DRAW_ANIM_TRAJECTORY) != 0) {
                    float r = link.start.height;

                    int col = duLerpCol(duRGBA(255, 192, 0, 255),
                            duRGBA(255, 255, 255, 255), 64);
                    int cola = duTransCol(col, 192);
                    int colb = duRGBA(255, 255, 255, 255);

                    // Start segment.
                    dd.begin(LINES, 3.0f);
                    dd.vertex(link.start.p, col);
                    dd.vertex(link.start.q, col);
                    dd.end();

                    dd.begin(LINES, 1.0f);
                    dd.vertex(link.start.p[0], link.start.p[1], link.start.p[2], colb);
                    dd.vertex(link.start.p[0], link.start.p[1] + r, link.start.p[2], colb);
                    dd.vertex(link.start.p[0], link.start.p[1] + r, link.start.p[2], colb);
                    dd.vertex(link.start.q[0], link.start.q[1] + r, link.start.q[2], colb);
                    dd.vertex(link.start.q[0], link.start.q[1] + r, link.start.q[2], colb);
                    dd.vertex(link.start.q[0], link.start.q[1], link.start.q[2], colb);
                    dd.vertex(link.start.q[0], link.start.q[1], link.start.q[2], colb);
                    dd.vertex(link.start.p[0], link.start.p[1], link.start.p[2], colb);
                    dd.end();

                    GroundSegment end = link.end;
                    r = end.height;
                    // End segment.
                    dd.begin(LINES, 3.0f);
                    dd.vertex(end.p, col);
                    dd.vertex(end.q, col);
                    dd.end();

                    dd.begin(LINES, 1.0f);
                    dd.vertex(end.p[0], end.p[1], end.p[2], colb);
                    dd.vertex(end.p[0], end.p[1] + r, end.p[2], colb);
                    dd.vertex(end.p[0], end.p[1] + r, end.p[2], colb);
                    dd.vertex(end.q[0], end.q[1] + r, end.q[2], colb);
                    dd.vertex(end.q[0], end.q[1] + r, end.q[2], colb);
                    dd.vertex(end.q[0], end.q[1], end.q[2], colb);
                    dd.vertex(end.q[0], end.q[1], end.q[2], colb);
                    dd.vertex(end.p[0], end.p[1], end.p[2], colb);
                    dd.end();

                    dd.begin(LINES, 4.0f);
                    drawTrajectory(dd, link, link.start.p, end.p, link.trajectory, cola);
                    drawTrajectory(dd, link, link.start.q, end.q, link.trajectory, cola);
                    dd.end();

                    dd.begin(LINES, 8.0f);
                    dd.vertex(link.start.p, duDarkenCol(col));
                    dd.vertex(link.start.q, duDarkenCol(col));
                    dd.vertex(end.p, duDarkenCol(col));
                    dd.vertex(end.q, duDarkenCol(col));
                    dd.end();

                    int colm = duRGBA(255, 255, 255, 255);
                    dd.begin(LINES, 3.0f);
                    dd.vertex(link.start.p, colm);
                    dd.vertex(link.start.q, colm);
                    dd.vertex(end.p, colm);
                    dd.vertex(end.q, colm);
                    dd.end();
                }
                if ((params.flags & JumpLinkBuilderToolParams.DRAW_LAND_SAMPLES) != 0) {
                    dd.begin(POINTS, 8.0f);
                    for (int i = 0; i < link.start.gsamples.length; ++i) {
                        GroundSample s = link.start.gsamples[i];
                        float u = i / (float) (link.start.gsamples.length - 1);
                        float[] spt = vLerp(link.start.p, link.start.q, u);
                        int col = duRGBA(48, 16, 16, 255); // duRGBA(255,(s->flags & 4)?255:0,0,255);
                        float off = 0.1f;
                        if (!s.validHeight) {
                            off = 0;
                            col = duRGBA(220, 32, 32, 255);
                        }
                        spt[1] = s.p[1] + off;
                        dd.vertex(spt, col);
                    }
                    dd.end();

                    dd.begin(POINTS, 4.0f);
                    for (int i = 0; i < link.start.gsamples.length; ++i) {
                        GroundSample s = link.start.gsamples[i];
                        float u = i / (float) (link.start.gsamples.length - 1);
                        float[] spt = vLerp(link.start.p, link.start.q, u);
                        int col = duRGBA(255, 255, 255, 255);
                        float off = 0;
                        if (s.validHeight) {
                            off = 0.1f;
                        }
                        spt[1] = s.p[1] + off;
                        dd.vertex(spt, col);
                    }
                    dd.end();
                    {
                        GroundSegment end = link.end;
                        dd.begin(POINTS, 8.0f);
                        for (int i = 0; i < end.gsamples.length; ++i) {
                            GroundSample s = end.gsamples[i];
                            float u = i / (float) (end.gsamples.length - 1);
                            float[] spt = vLerp(end.p, end.q, u);
                            int col = duRGBA(48, 16, 16, 255); // duRGBA(255,(s->flags & 4)?255:0,0,255);
                            float off = 0.1f;
                            if (!s.validHeight) {
                                off = 0;
                                col = duRGBA(220, 32, 32, 255);
                            }
                            spt[1] = s.p[1] + off;
                            dd.vertex(spt, col);
                        }
                        dd.end();
                        dd.begin(POINTS, 4.0f);
                        for (int i = 0; i < end.gsamples.length; ++i) {
                            GroundSample s = end.gsamples[i];
                            float u = i / (float) (end.gsamples.length - 1);
                            float[] spt = vLerp(end.p, end.q, u);
                            int col = duRGBA(255, 255, 255, 255);
                            float off = 0;
                            if (s.validHeight) {
                                off = 0.1f;
                            }
                            spt[1] = s.p[1] + off;
                            dd.vertex(spt, col);
                        }
                        dd.end();
                    }
                }
            }
        }
        dd.depthMask(true);
    }

    private void drawTrajectory(RecastDebugDraw dd, JumpLink link, float[] pa, float[] pb, Trajectory tra, int cola) {

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
            nk_property_float(ctx, "Min Cliff Height", 0f, params.climbDownMinHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Max Cliff Height", 0f, params.climbDownMaxHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 5, 1);
            nk_spacing(ctx, 1);

            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Jump Down", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Max Distance", 0f, params.edgeJumpEndDistance, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Jump Height", 0f, params.edgeJumpHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Max Jump Down", 0f, params.edgeJumpDownMaxHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Max Jump Up", 0f, params.edgeJumpUpMaxHeight, 10f, 0.05f, 0.01f);
            nk_layout_row_dynamic(ctx, 5, 1);
            nk_spacing(ctx, 1);
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, "Mode", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            int buildTypes = 0;
            buildTypes |= nk_option_text(ctx, "Climb Down",
                    (params.buildTypes & (1 << JumpLinkType.EDGE_CLIMB_DOWN.ordinal())) != 0)
                            ? (1 << JumpLinkType.EDGE_CLIMB_DOWN.ordinal())
                            : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            buildTypes |= nk_option_text(ctx, "Edge Jump",
                    (params.buildTypes & (1 << JumpLinkType.EDGE_JUMP.ordinal())) != 0)
                            ? (1 << JumpLinkType.EDGE_JUMP.ordinal())
                            : 0;
            params.buildTypes = buildTypes;
            boolean build = false;
            boolean buildOffMeshConnections = false;
            if (nk_button_text(ctx, "Build")) {
                build = true;
            }
            if (nk_button_text(ctx, "Build Off-Mesh Links")) {
                buildOffMeshConnections = true;
            }
            if (build || buildOffMeshConnections) {
                links.clear();
                if (annotationBuilder != null) {
                    float cellSize = sample.getSettingsUI().getCellSize();
                    float agentHeight = sample.getSettingsUI().getAgentHeight();
                    float agentRadius = sample.getSettingsUI().getAgentRadius();
                    float agentClimb = sample.getSettingsUI().getAgentMaxClimb();
                    float cellHeight = sample.getSettingsUI().getCellHeight();
                    if ((buildTypes & (1 << JumpLinkType.EDGE_CLIMB_DOWN.ordinal())) != 0) {
                        JumpLinkBuilderConfig config = new JumpLinkBuilderConfig(cellSize, cellHeight, agentRadius,
                                agentHeight, agentClimb, params.groundTolerance.get(0), -agentRadius * 0.2f,
                                cellSize + 2 * agentRadius + params.climbDownDistance.get(0),
                                -params.climbDownMaxHeight.get(0), -params.climbDownMinHeight.get(0), 0);
                        links.addAll(annotationBuilder.build(config, JumpLinkType.EDGE_CLIMB_DOWN));
                    }
                    if ((buildTypes & (1 << JumpLinkType.EDGE_JUMP.ordinal())) != 0) {
                        JumpLinkBuilderConfig config = new JumpLinkBuilderConfig(cellSize, cellHeight, agentRadius,
                                agentHeight, agentClimb, params.groundTolerance.get(0), -agentRadius * 0.2f,
                                params.edgeJumpEndDistance.get(0), -params.edgeJumpDownMaxHeight.get(0),
                                params.edgeJumpUpMaxHeight.get(0), params.edgeJumpHeight.get(0));
                        links.addAll(annotationBuilder.build(config, JumpLinkType.EDGE_JUMP));
                    }
                    if (buildOffMeshConnections) {
                        DemoInputGeomProvider geom = sample.getInputGeom();
                        if (geom != null) {
                            int area = SampleAreaModifications.SAMPLE_POLYAREA_TYPE_JUMP_AUTO;
                            geom.removeOffMeshConnections(c -> c.area == area);
                            links.forEach(l -> addOffMeshLink(l, geom, agentRadius));
                        }
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
            newFlags |= nk_option_text(ctx, "Land Samples",
                    (params.flags & JumpLinkBuilderToolParams.DRAW_LAND_SAMPLES) != 0)
                            ? JumpLinkBuilderToolParams.DRAW_LAND_SAMPLES
                            : 0;
            nk_layout_row_dynamic(ctx, 20, 1);
            newFlags |= nk_option_text(ctx, "All Annotations",
                    (params.flags & JumpLinkBuilderToolParams.DRAW_ANNOTATIONS) != 0)
                            ? JumpLinkBuilderToolParams.DRAW_ANNOTATIONS
                            : 0;
            params.flags = newFlags;
        }

    }

    private void addOffMeshLink(JumpLink link, DemoInputGeomProvider geom, float agentRadius) {
        int area = SampleAreaModifications.SAMPLE_POLYAREA_TYPE_JUMP_AUTO;
        int flags = SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP;
        float[] prev = new float[3];
        for (int i = 0; i < link.startSamples.length; i++) {
            float[] p = link.startSamples[i].p;
            float[] q = link.endSamples[i].p;
            if (i == 0 || vDist2D(prev, p) > agentRadius) {
                geom.addOffMeshConnection(p, q, agentRadius, false, area, flags);
                prev = p;
            }
        }
    }

    @Override
    public String getName() {
        return "Annotation Builder";
    }

}
