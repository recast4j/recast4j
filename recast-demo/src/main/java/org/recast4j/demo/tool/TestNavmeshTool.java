package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_label;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;
import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vLerp;
import static org.recast4j.detour.DetourCommon.vMad;
import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vSub;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.DebugDraw;
import org.recast4j.demo.draw.DebugDrawPrimitives;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.math.DemoMath;
import org.recast4j.demo.sample.Sample;
import org.recast4j.demo.tool.PathUtils.SteerTarget;
import org.recast4j.detour.ClosestPointOnPolyResult;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.DetourCommon;
import org.recast4j.detour.FindDistanceToWallResult;
import org.recast4j.detour.FindLocalNeighbourhoodResult;
import org.recast4j.detour.FindPolysAroundResult;
import org.recast4j.detour.GetPolyWallSegmentsResult;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.MoveAlongSurfaceResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.Poly;
import org.recast4j.detour.RaycastHit;
import org.recast4j.detour.Result;
import org.recast4j.detour.Status;
import org.recast4j.detour.StraightPathItem;
import org.recast4j.detour.Tupple2;

public class TestNavmeshTool implements Tool {

    private final static int MAX_POLYS = 256;
    private final static int MAX_SMOOTH = 2048;
    private Sample m_sample;
    private ToolMode m_toolMode = ToolMode.PATHFIND_FOLLOW;
    private boolean m_sposSet;
    private boolean m_eposSet;
    private float[] m_spos;
    private float[] m_epos;
    private final DefaultQueryFilter m_filter;
    private final float[] m_polyPickExt = new float[] { 2, 4, 2 };
    private long m_startRef;
    private long m_endRef;
    private float[] m_hitPos;
    private float m_distanceToWall;
    private float[] m_hitNormal;
    private List<StraightPathItem> m_straightPath;
    private int m_straightPathOptions;
    private List<Long> m_polys;
    private boolean m_hitResult;
    private List<Long> m_parent;
    private float m_neighbourhoodRadius;
    private final float[] m_queryPoly = new float[12];
    private List<float[]> m_smoothPath;
    private Status m_pathFindStatus = Status.FAILURE;

    private enum ToolMode {
        PATHFIND_FOLLOW,
        PATHFIND_STRAIGHT,
        PATHFIND_SLICED,
        DISTANCE_TO_WALL,
        RAYCAST,
        FIND_POLYS_IN_CIRCLE,
        FIND_POLYS_IN_SHAPE,
        FIND_LOCAL_NEIGHBOURHOOD
    }

    public TestNavmeshTool() {
        m_filter = new DefaultQueryFilter(SampleAreaModifications.SAMPLE_POLYFLAGS_ALL,
                SampleAreaModifications.SAMPLE_POLYFLAGS_DISABLED, new float[] { 1f, 10f, 1f, 1f, 2f, 1.5f });
    }

    @Override
    public void setSample(Sample m_sample) {
        this.m_sample = m_sample;
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {
        if (shift) {
            m_sposSet = true;
            m_spos = Arrays.copyOf(p, p.length);
        } else {
            m_eposSet = true;
            m_epos = Arrays.copyOf(p, p.length);
        }
        recalc();
    }

    @Override
    public void layout(NkContext ctx) {
        ToolMode previousToolMode = m_toolMode;
        int previousStraightPathOptions = m_straightPathOptions;
        int previousIncludeFlags = m_filter.getIncludeFlags();
        int previousExcludeFlags = m_filter.getExcludeFlags();

        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Pathfind Follow", m_toolMode == ToolMode.PATHFIND_FOLLOW)) {
            m_toolMode = ToolMode.PATHFIND_FOLLOW;
        }
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Pathfind Straight", m_toolMode == ToolMode.PATHFIND_STRAIGHT)) {
            m_toolMode = ToolMode.PATHFIND_STRAIGHT;
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_label(ctx, "Vertices at crossings", NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 20, 1);
            if (nk_option_label(ctx, "None", m_straightPathOptions == 0)) {
                m_straightPathOptions = 0;
            }
            nk_layout_row_dynamic(ctx, 20, 1);
            if (nk_option_label(ctx, "Area", m_straightPathOptions == NavMeshQuery.DT_STRAIGHTPATH_AREA_CROSSINGS)) {
                m_straightPathOptions = NavMeshQuery.DT_STRAIGHTPATH_AREA_CROSSINGS;
            }
            nk_layout_row_dynamic(ctx, 20, 1);
            if (nk_option_label(ctx, "All", m_straightPathOptions == NavMeshQuery.DT_STRAIGHTPATH_ALL_CROSSINGS)) {
                m_straightPathOptions = NavMeshQuery.DT_STRAIGHTPATH_ALL_CROSSINGS;
            }
            nk_layout_row_dynamic(ctx, 5, 1);
            nk_spacing(ctx, 1);
        }
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Pathfind Sliced", m_toolMode == ToolMode.PATHFIND_SLICED)) {
            m_toolMode = ToolMode.PATHFIND_SLICED;
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Distance to Wall", m_toolMode == ToolMode.DISTANCE_TO_WALL)) {
            m_toolMode = ToolMode.DISTANCE_TO_WALL;
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Raycast", m_toolMode == ToolMode.RAYCAST)) {
            m_toolMode = ToolMode.RAYCAST;
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Find Polys in Circle", m_toolMode == ToolMode.FIND_POLYS_IN_CIRCLE)) {
            m_toolMode = ToolMode.FIND_POLYS_IN_CIRCLE;
        }
        if (nk_option_label(ctx, "Find Polys in Shape", m_toolMode == ToolMode.FIND_POLYS_IN_SHAPE)) {
            m_toolMode = ToolMode.FIND_POLYS_IN_SHAPE;
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Find Local Neighbourhood", m_toolMode == ToolMode.FIND_LOCAL_NEIGHBOURHOOD)) {
            m_toolMode = ToolMode.FIND_LOCAL_NEIGHBOURHOOD;
        }

        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_label(ctx, "Include Flags", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        int includeFlags = 0;
        if (nk_option_label(ctx, "Walk",
                (m_filter.getIncludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_WALK) != 0)) {
            includeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_WALK;
        }
        if (nk_option_label(ctx, "Swim",
                (m_filter.getIncludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM) != 0)) {
            includeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM;
        }
        if (nk_option_label(ctx, "Door",
                (m_filter.getIncludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR) != 0)) {
            includeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR;
        }
        if (nk_option_label(ctx, "Jump",
                (m_filter.getIncludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP) != 0)) {
            includeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP;
        }
        m_filter.setIncludeFlags(includeFlags);

        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_label(ctx, "Exclude Flags", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        int excludeFlags = 0;
        if (nk_option_label(ctx, "Walk",
                (m_filter.getExcludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_WALK) != 0)) {
            excludeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_WALK;
        }
        if (nk_option_label(ctx, "Swim",
                (m_filter.getExcludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM) != 0)) {
            excludeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM;
        }
        if (nk_option_label(ctx, "Door",
                (m_filter.getExcludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR) != 0)) {
            excludeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR;
        }
        if (nk_option_label(ctx, "Jump",
                (m_filter.getExcludeFlags() & SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP) != 0)) {
            excludeFlags |= SampleAreaModifications.SAMPLE_POLYFLAGS_JUMP;
        }
        m_filter.setExcludeFlags(excludeFlags);

        if (previousToolMode != m_toolMode || m_straightPathOptions != previousStraightPathOptions
                || previousIncludeFlags != includeFlags || previousExcludeFlags != excludeFlags) {
            recalc();
        }
    }

    @Override
    public String getName() {
        return "Test Navmesh";
    }

    private void recalc() {
        if (m_sample == null || m_sample.getNavMesh() == null) {
            return;
        }
        NavMeshQuery m_navQuery = m_sample.getNavMeshQuery();
        if (m_sposSet) {
            m_startRef = m_navQuery.findNearestPoly(m_spos, m_polyPickExt, m_filter).result.getNearestRef();
        } else {
            m_startRef = 0;
        }
        if (m_eposSet) {
            m_endRef = m_navQuery.findNearestPoly(m_epos, m_polyPickExt, m_filter).result.getNearestRef();
        } else {
            m_endRef = 0;
        }
        NavMesh m_navMesh = m_sample.getNavMesh();
        if (m_toolMode == ToolMode.PATHFIND_FOLLOW) {
            if (m_sposSet && m_eposSet && m_startRef != 0 && m_endRef != 0) {
                m_polys = m_navQuery.findPath(m_startRef, m_endRef, m_spos, m_epos, m_filter).result;
                if (!m_polys.isEmpty()) {
                    List<Long> polys = new ArrayList<>(m_polys);
                    // Iterate over the path to find smooth path on the detail mesh surface.
                    float[] iterPos = m_navQuery.closestPointOnPoly(m_startRef, m_spos).result.getClosest();
                    float[] targetPos = m_navQuery.closestPointOnPoly(polys.get(polys.size() - 1), m_epos).result
                            .getClosest();

                    float STEP_SIZE = 0.5f;
                    float SLOP = 0.01f;

                    m_smoothPath = new ArrayList<>();
                    m_smoothPath.add(iterPos);

                    // Move towards target a small advancement at a time until target reached or
                    // when ran out of memory to store the path.
                    while (!polys.isEmpty() && m_smoothPath.size() < MAX_SMOOTH) {
                        // Find location to steer towards.
                        Optional<SteerTarget> steerTarget = PathUtils.getSteerTarget(m_navQuery, iterPos, targetPos,
                                SLOP, polys);
                        if (!steerTarget.isPresent()) {
                            break;
                        }
                        boolean endOfPath = (steerTarget.get().steerPosFlag & NavMeshQuery.DT_STRAIGHTPATH_END) != 0
                                ? true
                                : false;
                        boolean offMeshConnection = (steerTarget.get().steerPosFlag
                                & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0 ? true : false;

                        // Find movement delta.
                        float[] delta = vSub(steerTarget.get().steerPos, iterPos);
                        float len = (float) Math.sqrt(DemoMath.vDot(delta, delta));
                        // If the steer target is end of path or off-mesh link, do not move past the location.
                        if ((endOfPath || offMeshConnection) && len < STEP_SIZE) {
                            len = 1;
                        } else {
                            len = STEP_SIZE / len;
                        }
                        float[] moveTgt = vMad(iterPos, delta, len);
                        // Move
                        Result<MoveAlongSurfaceResult> result = m_navQuery.moveAlongSurface(polys.get(0), iterPos,
                                moveTgt, m_filter);
                        MoveAlongSurfaceResult moveAlongSurface = result.result;

                        iterPos = new float[3];
                        iterPos[0] = moveAlongSurface.getResultPos()[0];
                        iterPos[2] = moveAlongSurface.getResultPos()[2];

                        List<Long> visited = result.result.getVisited();
                        polys = PathUtils.fixupCorridor(polys, visited);
                        polys = PathUtils.fixupShortcuts(polys, m_navQuery);

                        iterPos[1] = m_navQuery.getPolyHeight(polys.get(0), moveAlongSurface.getResultPos()).result;

                        // Handle end of path and off-mesh links when close enough.
                        if (endOfPath && PathUtils.inRange(iterPos, steerTarget.get().steerPos, SLOP, 1.0f)) {
                            // Reached end of path.
                            vCopy(iterPos, targetPos);
                            if (m_smoothPath.size() < MAX_SMOOTH) {
                                m_smoothPath.add(iterPos);
                            }
                            break;
                        } else if (offMeshConnection
                                && PathUtils.inRange(iterPos, steerTarget.get().steerPos, SLOP, 1.0f)) {
                            // Reached off-mesh connection.
                            // Advance the path up to and over the off-mesh connection.
                            long prevRef = 0;
                            long polyRef = polys.get(0);
                            int npos = 0;
                            while (npos < polys.size() && polyRef != steerTarget.get().steerPosRef) {
                                prevRef = polyRef;
                                polyRef = polys.get(npos);
                                npos++;
                            }
                            polys = polys.subList(npos, polys.size());

                            // Handle the connection.
                            Result<Tupple2<float[], float[]>> offMeshCon = m_navMesh
                                    .getOffMeshConnectionPolyEndPoints(prevRef, polyRef);
                            if (offMeshCon.succeeded()) {
                                float[] startPos = offMeshCon.result.first;
                                float[] endPos = offMeshCon.result.second;
                                if (m_smoothPath.size() < MAX_SMOOTH) {
                                    m_smoothPath.add(startPos);
                                    // Hack to make the dotted path not visible during off-mesh connection.
                                    if ((m_smoothPath.size() & 1) != 0) {
                                        m_smoothPath.add(startPos);
                                    }
                                }
                                // Move position at the other side of the off-mesh link.
                                vCopy(iterPos, endPos);
                                iterPos[1] = m_navQuery.getPolyHeight(polys.get(0), iterPos).result;
                            }
                        }

                        // Store results.
                        if (m_smoothPath.size() < MAX_SMOOTH) {
                            m_smoothPath.add(iterPos);
                        }
                    }
                }
            } else {
                m_polys = null;
                m_smoothPath = null;
            }
        } else if (m_toolMode == ToolMode.PATHFIND_STRAIGHT) {
            if (m_sposSet && m_eposSet && m_startRef != 0 && m_endRef != 0) {
                m_polys = m_navQuery.findPath(m_startRef, m_endRef, m_spos, m_epos, m_filter).result;
                if (!m_polys.isEmpty()) {
                    // In case of partial path, make sure the end point is clamped to the last polygon.
                    float[] epos = new float[] { m_epos[0], m_epos[1], m_epos[2] };
                    if (m_polys.get(m_polys.size() - 1) != m_endRef) {
                        Result<ClosestPointOnPolyResult> result = m_navQuery
                                .closestPointOnPoly(m_polys.get(m_polys.size() - 1), m_epos);
                        if (result.succeeded()) {
                            epos = result.result.getClosest();
                        }
                    }
                    m_straightPath = m_navQuery.findStraightPath(m_spos, epos, m_polys, MAX_POLYS,
                            m_straightPathOptions).result;
                }
            } else {
                m_straightPath = null;
            }
        } else if (m_toolMode == ToolMode.PATHFIND_SLICED) {
            m_polys = null;
            m_straightPath = null;
            if (m_sposSet && m_eposSet && m_startRef != 0 && m_endRef != 0) {
                m_pathFindStatus = m_navQuery.initSlicedFindPath(m_startRef, m_endRef, m_spos, m_epos, m_filter,
                        NavMeshQuery.DT_FINDPATH_ANY_ANGLE);
            }
        } else if (m_toolMode == ToolMode.RAYCAST) {
            m_straightPath = null;
            if (m_sposSet && m_eposSet && m_startRef != 0) {
                {
                    Result<RaycastHit> hit = m_navQuery.raycast(m_startRef, m_spos, m_epos, m_filter, 0, 0);
                    if (hit.succeeded()) {
                        m_polys = hit.result.path;
                        if (hit.result.t > 1) {
                            // No hit
                            m_hitPos = Arrays.copyOf(m_epos, m_epos.length);
                            m_hitResult = false;
                        } else {
                            // Hit
                            m_hitPos = vLerp(m_spos, m_epos, hit.result.t);
                            m_hitNormal = Arrays.copyOf(hit.result.hitNormal, hit.result.hitNormal.length);
                            m_hitResult = true;
                        }
                        // Adjust height.
                        if (hit.result.path.size() > 0) {
                            Result<Float> result = m_navQuery
                                    .getPolyHeight(hit.result.path.get(hit.result.path.size() - 1), m_hitPos);
                            if (result.succeeded()) {
                                m_hitPos[1] = result.result;
                            }
                        }
                    }
                    m_straightPath = new ArrayList<>();
                    m_straightPath.add(new StraightPathItem(m_spos, 0, 0));
                    m_straightPath.add(new StraightPathItem(m_hitPos, 0, 0));
                }
            }
        } else if (m_toolMode == ToolMode.DISTANCE_TO_WALL) {
            m_distanceToWall = 0;
            if (m_sposSet && m_startRef != 0) {
                m_distanceToWall = 0.0f;
                Result<FindDistanceToWallResult> result = m_navQuery.findDistanceToWall(m_startRef, m_spos, 100.0f,
                        m_filter);
                if (result.succeeded()) {
                    m_distanceToWall = result.result.getDistance();
                    m_hitPos = result.result.getPosition();
                    m_hitNormal = result.result.getNormal();
                }
            }
        } else if (m_toolMode == ToolMode.FIND_POLYS_IN_CIRCLE) {
            if (m_sposSet && m_startRef != 0 && m_eposSet) {
                float dx = m_epos[0] - m_spos[0];
                float dz = m_epos[2] - m_spos[2];
                float dist = (float) Math.sqrt(dx * dx + dz * dz);
                Result<FindPolysAroundResult> result = m_navQuery.findPolysAroundCircle(m_startRef, m_spos, dist,
                        m_filter);
                if (result.succeeded()) {
                    m_polys = result.result.getRefs();
                    m_parent = result.result.getParentRefs();
                }
            }
        } else if (m_toolMode == ToolMode.FIND_POLYS_IN_SHAPE) {
            if (m_sposSet && m_startRef != 0 && m_eposSet) {
                float nx = (m_epos[2] - m_spos[2]) * 0.25f;
                float nz = -(m_epos[0] - m_spos[0]) * 0.25f;
                float agentHeight = m_sample != null ? m_sample.getSettingsUI().getAgentHeight() : 0;

                m_queryPoly[0] = m_spos[0] + nx * 1.2f;
                m_queryPoly[1] = m_spos[1] + agentHeight / 2;
                m_queryPoly[2] = m_spos[2] + nz * 1.2f;

                m_queryPoly[3] = m_spos[0] - nx * 1.3f;
                m_queryPoly[4] = m_spos[1] + agentHeight / 2;
                m_queryPoly[5] = m_spos[2] - nz * 1.3f;

                m_queryPoly[6] = m_epos[0] - nx * 0.8f;
                m_queryPoly[7] = m_epos[1] + agentHeight / 2;
                m_queryPoly[8] = m_epos[2] - nz * 0.8f;

                m_queryPoly[9] = m_epos[0] + nx;
                m_queryPoly[10] = m_epos[1] + agentHeight / 2;
                m_queryPoly[11] = m_epos[2] + nz;

                Result<FindPolysAroundResult> result = m_navQuery.findPolysAroundShape(m_startRef, m_queryPoly, 4,
                        m_filter);
                if (result.succeeded()) {
                    m_polys = result.result.getRefs();
                    m_parent = result.result.getParentRefs();
                }
            }
        } else if (m_toolMode == ToolMode.FIND_LOCAL_NEIGHBOURHOOD) {
            if (m_sposSet && m_startRef != 0) {
                m_neighbourhoodRadius = m_sample.getSettingsUI().getAgentRadius() * 20.0f;
                Result<FindLocalNeighbourhoodResult> result = m_navQuery.findLocalNeighbourhood(m_startRef, m_spos,
                        m_neighbourhoodRadius, m_filter);
                if (result.succeeded()) {
                    m_polys = result.result.getRefs();
                    m_parent = result.result.getParentRefs();
                }
            }
        }
    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        if (m_sample == null) {
            return;
        }
        RecastDebugDraw dd = renderer.getDebugDraw();
        int startCol = DebugDraw.duRGBA(128, 25, 0, 192);
        int endCol = DebugDraw.duRGBA(51, 102, 0, 129);
        int pathCol = DebugDraw.duRGBA(0, 0, 0, 64);

        float agentRadius = m_sample.getSettingsUI().getAgentRadius();
        float agentHeight = m_sample.getSettingsUI().getAgentHeight();
        float agentClimb = m_sample.getSettingsUI().getAgentMaxClimb();

        if (m_sposSet) {
            drawAgent(dd, m_spos, agentRadius, agentHeight, agentClimb, startCol);
        }
        if (m_eposSet) {
            drawAgent(dd, m_epos, agentRadius, agentHeight, agentClimb, endCol);
        }
        dd.depthMask(true);

        NavMesh m_navMesh = m_sample.getNavMesh();
        if (m_navMesh == null) {
            return;
        }

        if (m_toolMode == ToolMode.PATHFIND_FOLLOW) {
            dd.debugDrawNavMeshPoly(m_navMesh, m_startRef, startCol);
            dd.debugDrawNavMeshPoly(m_navMesh, m_endRef, endCol);

            if (m_polys != null) {
                for (Long poly : m_polys) {
                    if (poly == m_startRef || poly == m_endRef) {
                        continue;
                    }
                    dd.debugDrawNavMeshPoly(m_navMesh, poly, pathCol);
                }
            }
            if (m_smoothPath != null) {
                dd.depthMask(false);
                int spathCol = DebugDraw.duRGBA(0, 0, 0, 220);
                dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                for (int i = 0; i < m_smoothPath.size(); ++i) {
                    dd.vertex(m_smoothPath.get(i)[0], m_smoothPath.get(i)[1] + 0.1f, m_smoothPath.get(i)[2], spathCol);
                }
                dd.end();
                dd.depthMask(true);
            }
            /*
            if (m_pathIterNum)
            {
                duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_pathIterPolys[0], DebugDraw.duRGBA(255,255,255,128));

                dd.depthMask(false);
                dd.begin(DebugDrawPrimitives.LINES, 1.0f);

                int prevCol = DebugDraw.duRGBA(255,192,0,220);
                int curCol = DebugDraw.duRGBA(255,255,255,220);
                int steerCol = DebugDraw.duRGBA(0,192,255,220);

                dd.vertex(m_prevIterPos[0],m_prevIterPos[1]-0.3f,m_prevIterPos[2], prevCol);
                dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);

                dd.vertex(m_iterPos[0],m_iterPos[1]-0.3f,m_iterPos[2], curCol);
                dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], curCol);

                dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);
                dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], prevCol);

                dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], steerCol);
                dd.vertex(m_steerPos[0],m_steerPos[1]+0.3f,m_steerPos[2], steerCol);

                for (int i = 0; i < m_steerPointCount-1; ++i)
                {
                    dd.vertex(m_steerPoints[i*3+0],m_steerPoints[i*3+1]+0.2f,m_steerPoints[i*3+2], duDarkenCol(steerCol));
                    dd.vertex(m_steerPoints[(i+1)*3+0],m_steerPoints[(i+1)*3+1]+0.2f,m_steerPoints[(i+1)*3+2], duDarkenCol(steerCol));
                }

                dd.end();
                dd.depthMask(true);
            }
            */
        } else if (m_toolMode == ToolMode.PATHFIND_STRAIGHT || m_toolMode == ToolMode.PATHFIND_SLICED) {
            dd.debugDrawNavMeshPoly(m_navMesh, m_startRef, startCol);
            dd.debugDrawNavMeshPoly(m_navMesh, m_endRef, endCol);

            if (m_polys != null) {
                for (Long poly : m_polys) {
                    dd.debugDrawNavMeshPoly(m_navMesh, poly, pathCol);
                }
            }
            if (m_straightPath != null) {
                dd.depthMask(false);
                int spathCol = DebugDraw.duRGBA(64, 16, 0, 220);
                int offMeshCol = DebugDraw.duRGBA(128, 96, 0, 220);
                dd.begin(DebugDrawPrimitives.LINES, 2.0f);
                for (int i = 0; i < m_straightPath.size() - 1; ++i) {
                    StraightPathItem straightPathItem = m_straightPath.get(i);
                    StraightPathItem straightPathItem2 = m_straightPath.get(i + 1);
                    int col;
                    if ((straightPathItem.getFlags() & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0) {
                        col = offMeshCol;
                    } else {
                        col = spathCol;
                    }
                    dd.vertex(straightPathItem.getPos()[0], straightPathItem.getPos()[1] + 0.4f,
                            straightPathItem.getPos()[2], col);
                    dd.vertex(straightPathItem2.getPos()[0], straightPathItem2.getPos()[1] + 0.4f,
                            straightPathItem2.getPos()[2], col);
                }
                dd.end();
                dd.begin(DebugDrawPrimitives.POINTS, 6.0f);
                for (int i = 0; i < m_straightPath.size(); ++i) {
                    StraightPathItem straightPathItem = m_straightPath.get(i);
                    int col;
                    if ((straightPathItem.getFlags() & NavMeshQuery.DT_STRAIGHTPATH_START) != 0) {
                        col = startCol;
                    } else if ((straightPathItem.getFlags() & NavMeshQuery.DT_STRAIGHTPATH_END) != 0) {
                        col = endCol;
                    } else if ((straightPathItem.getFlags() & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0) {
                        col = offMeshCol;
                    } else {
                        col = spathCol;
                    }
                    dd.vertex(straightPathItem.getPos()[0], straightPathItem.getPos()[1] + 0.4f,
                            straightPathItem.getPos()[2], col);
                }
                dd.end();
                dd.depthMask(true);
            }
        } else if (m_toolMode == ToolMode.RAYCAST) {
            dd.debugDrawNavMeshPoly(m_navMesh, m_startRef, startCol);

            if (m_straightPath != null) {
                if (m_polys != null) {
                    for (Long poly : m_polys) {
                        dd.debugDrawNavMeshPoly(m_navMesh, poly, pathCol);
                    }
                }

                dd.depthMask(false);
                int spathCol = m_hitResult ? DebugDraw.duRGBA(64, 16, 0, 220) : DebugDraw.duRGBA(240, 240, 240, 220);
                dd.begin(DebugDrawPrimitives.LINES, 2.0f);
                for (int i = 0; i < m_straightPath.size() - 1; ++i) {
                    StraightPathItem straightPathItem = m_straightPath.get(i);
                    StraightPathItem straightPathItem2 = m_straightPath.get(i + 1);
                    dd.vertex(straightPathItem.getPos()[0], straightPathItem.getPos()[1] + 0.4f,
                            straightPathItem.getPos()[2], spathCol);
                    dd.vertex(straightPathItem2.getPos()[0], straightPathItem2.getPos()[1] + 0.4f,
                            straightPathItem2.getPos()[2], spathCol);
                }
                dd.end();
                dd.begin(DebugDrawPrimitives.POINTS, 4.0f);
                for (int i = 0; i < m_straightPath.size(); ++i) {
                    StraightPathItem straightPathItem = m_straightPath.get(i);
                    dd.vertex(straightPathItem.getPos()[0], straightPathItem.getPos()[1] + 0.4f,
                            straightPathItem.getPos()[2], spathCol);
                }
                dd.end();

                if (m_hitResult) {
                    int hitCol = DebugDraw.duRGBA(0, 0, 0, 128);
                    dd.begin(DebugDrawPrimitives.LINES, 2.0f);
                    dd.vertex(m_hitPos[0], m_hitPos[1] + 0.4f, m_hitPos[2], hitCol);
                    dd.vertex(m_hitPos[0] + m_hitNormal[0] * agentRadius,
                            m_hitPos[1] + 0.4f + m_hitNormal[1] * agentRadius,
                            m_hitPos[2] + m_hitNormal[2] * agentRadius, hitCol);
                    dd.end();
                }
                dd.depthMask(true);
            }
        } else if (m_toolMode == ToolMode.DISTANCE_TO_WALL) {
            dd.debugDrawNavMeshPoly(m_navMesh, m_startRef, startCol);
            dd.depthMask(false);
            if (m_spos != null) {
                dd.debugDrawCircle(m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], m_distanceToWall,
                        DebugDraw.duRGBA(64, 16, 0, 220), 2.0f);
            }
            if (m_hitPos != null) {
                dd.begin(DebugDrawPrimitives.LINES, 3.0f);
                dd.vertex(m_hitPos[0], m_hitPos[1] + 0.02f, m_hitPos[2], DebugDraw.duRGBA(0, 0, 0, 192));
                dd.vertex(m_hitPos[0], m_hitPos[1] + agentHeight, m_hitPos[2], DebugDraw.duRGBA(0, 0, 0, 192));
                dd.end();
            }
            dd.depthMask(true);
        } else if (m_toolMode == ToolMode.FIND_POLYS_IN_CIRCLE) {
            if (m_polys != null) {
                for (int i = 0; i < m_polys.size(); i++) {
                    dd.debugDrawNavMeshPoly(m_navMesh, m_polys.get(i), pathCol);
                    dd.depthMask(false);
                    if (m_parent.get(i) != 0) {
                        dd.depthMask(false);
                        float[] p0 = getPolyCenter(m_navMesh, m_parent.get(i));
                        float[] p1 = getPolyCenter(m_navMesh, m_polys.get(i));
                        dd.debugDrawArc(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.25f, 0.0f, 0.4f,
                                DebugDraw.duRGBA(0, 0, 0, 128), 2.0f);
                        dd.depthMask(true);
                    }
                    dd.depthMask(true);
                }
            }

            if (m_sposSet && m_eposSet) {
                dd.depthMask(false);
                float dx = m_epos[0] - m_spos[0];
                float dz = m_epos[2] - m_spos[2];
                float dist = (float) Math.sqrt(dx * dx + dz * dz);
                dd.debugDrawCircle(m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], dist,
                        DebugDraw.duRGBA(64, 16, 0, 220), 2.0f);
                dd.depthMask(true);
            }
        } else if (m_toolMode == ToolMode.FIND_POLYS_IN_SHAPE) {
            if (m_polys != null) {
                for (int i = 0; i < m_polys.size(); i++) {
                    dd.debugDrawNavMeshPoly(m_navMesh, m_polys.get(i), pathCol);
                    dd.depthMask(false);
                    if (m_parent.get(i) != 0) {
                        dd.depthMask(false);
                        float[] p0 = getPolyCenter(m_navMesh, m_parent.get(i));
                        float[] p1 = getPolyCenter(m_navMesh, m_polys.get(i));
                        dd.debugDrawArc(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.25f, 0.0f, 0.4f,
                                DebugDraw.duRGBA(0, 0, 0, 128), 2.0f);
                        dd.depthMask(true);
                    }
                    dd.depthMask(true);
                }
            }

            if (m_sposSet && m_eposSet) {
                dd.depthMask(false);
                int col = DebugDraw.duRGBA(64, 16, 0, 220);
                dd.begin(DebugDrawPrimitives.LINES, 2.0f);
                for (int i = 0, j = 3; i < 4; j = i++) {
                    dd.vertex(m_queryPoly[j * 3], m_queryPoly[j * 3 + 1], m_queryPoly[j * 3 + 2], col);
                    dd.vertex(m_queryPoly[i * 3], m_queryPoly[i * 3 + 1], m_queryPoly[i * 3 + 2], col);
                }
                dd.end();
                dd.depthMask(true);
            }
        } else if (m_toolMode == ToolMode.FIND_LOCAL_NEIGHBOURHOOD) {
            if (m_polys != null) {
                for (int i = 0; i < m_polys.size(); i++) {
                    dd.debugDrawNavMeshPoly(m_navMesh, m_polys.get(i), pathCol);
                    dd.depthMask(false);
                    if (m_parent.get(i) != 0) {
                        dd.depthMask(false);
                        float[] p0 = getPolyCenter(m_navMesh, m_parent.get(i));
                        float[] p1 = getPolyCenter(m_navMesh, m_polys.get(i));
                        dd.debugDrawArc(p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.25f, 0.0f, 0.4f,
                                DebugDraw.duRGBA(0, 0, 0, 128), 2.0f);
                        dd.depthMask(true);
                    }
                    dd.depthMask(true);
                    if (m_sample.getNavMeshQuery() != null) {
                        Result<GetPolyWallSegmentsResult> result = m_sample.getNavMeshQuery()
                                .getPolyWallSegments(m_polys.get(i), false, m_filter);
                        if (result.succeeded()) {
                            dd.begin(DebugDrawPrimitives.LINES, 2.0f);
                            GetPolyWallSegmentsResult wallSegments = result.result;
                            for (int j = 0; j < wallSegments.getSegmentVerts().size(); ++j) {
                                float[] s = wallSegments.getSegmentVerts().get(j);
                                float[] s3 = new float[] { s[3], s[4], s[5] };
                                // Skip too distant segments.
                                Tupple2<Float, Float> distSqr = DetourCommon.distancePtSegSqr2D(m_spos, s, 0, 3);
                                if (distSqr.first > DemoMath.sqr(m_neighbourhoodRadius)) {
                                    continue;
                                }
                                float[] delta = vSub(s3, s);
                                float[] p0 = vMad(s, delta, 0.5f);
                                float[] norm = new float[] { delta[2], 0, -delta[0] };
                                vNormalize(norm);
                                float[] p1 = vMad(p0, norm, agentRadius * 0.5f);
                                // Skip backfacing segments.
                                if (wallSegments.getSegmentRefs().get(j) != 0) {
                                    int col = DebugDraw.duRGBA(255, 255, 255, 32);
                                    dd.vertex(s[0], s[1] + agentClimb, s[2], col);
                                    dd.vertex(s[3], s[4] + agentClimb, s[5], col);
                                } else {
                                    int col = DebugDraw.duRGBA(192, 32, 16, 192);
                                    if (DetourCommon.triArea2D(m_spos, s, s3) < 0.0f) {
                                        col = DebugDraw.duRGBA(96, 32, 16, 192);
                                    }
                                    dd.vertex(p0[0], p0[1] + agentClimb, p0[2], col);
                                    dd.vertex(p1[0], p1[1] + agentClimb, p1[2], col);

                                    dd.vertex(s[0], s[1] + agentClimb, s[2], col);
                                    dd.vertex(s[3], s[4] + agentClimb, s[5], col);
                                }
                            }
                            dd.end();
                        }
                    }

                    dd.depthMask(true);
                }

                if (m_sposSet) {
                    dd.depthMask(false);
                    dd.debugDrawCircle(m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], m_neighbourhoodRadius,
                            DebugDraw.duRGBA(64, 16, 0, 220), 2.0f);
                    dd.depthMask(true);
                }
            }
        }
    }

    private void drawAgent(RecastDebugDraw dd, float[] pos, float r, float h, float c, int col) {
        dd.depthMask(false);
        // Agent dimensions.
        dd.debugDrawCylinderWire(pos[0] - r, pos[1] + 0.02f, pos[2] - r, pos[0] + r, pos[1] + h, pos[2] + r, col, 2.0f);
        dd.debugDrawCircle(pos[0], pos[1] + c, pos[2], r, DebugDraw.duRGBA(0, 0, 0, 64), 1.0f);
        int colb = DebugDraw.duRGBA(0, 0, 0, 196);
        dd.begin(DebugDrawPrimitives.LINES);
        dd.vertex(pos[0], pos[1] - c, pos[2], colb);
        dd.vertex(pos[0], pos[1] + c, pos[2], colb);
        dd.vertex(pos[0] - r / 2, pos[1] + 0.02f, pos[2], colb);
        dd.vertex(pos[0] + r / 2, pos[1] + 0.02f, pos[2], colb);
        dd.vertex(pos[0], pos[1] + 0.02f, pos[2] - r / 2, colb);
        dd.vertex(pos[0], pos[1] + 0.02f, pos[2] + r / 2, colb);
        dd.end();
        dd.depthMask(true);
    }

    private float[] getPolyCenter(NavMesh navMesh, long ref) {
        float[] center = new float[3];
        center[0] = 0;
        center[1] = 0;
        center[2] = 0;
        Result<Tupple2<MeshTile, Poly>> tileAndPoly = navMesh.getTileAndPolyByRef(ref);
        if (tileAndPoly.succeeded()) {
            MeshTile tile = tileAndPoly.result.first;
            Poly poly = tileAndPoly.result.second;
            for (int i = 0; i < poly.vertCount; ++i) {
                int v = poly.verts[i] * 3;
                center[0] += tile.data.verts[v];
                center[1] += tile.data.verts[v + 1];
                center[2] += tile.data.verts[v + 2];
            }
            float s = 1.0f / poly.vertCount;
            center[0] *= s;
            center[1] *= s;
            center[2] *= s;
        }
        return center;
    }

    @Override
    public void handleUpdate(float dt) {
        // TODO Auto-generated method stub
        if (m_toolMode == ToolMode.PATHFIND_SLICED) {
            NavMeshQuery m_navQuery = m_sample.getNavMeshQuery();
            if (m_pathFindStatus.isInProgress()) {
                m_pathFindStatus = m_navQuery.updateSlicedFindPath(1).status;
            }
            if (m_pathFindStatus.isSuccess()) {
                m_polys = m_navQuery.finalizeSlicedFindPath().result;
                m_straightPath = null;
                if (m_polys != null) {
                    // In case of partial path, make sure the end point is clamped to the last polygon.
                    float[] epos = new float[3];
                    DetourCommon.vCopy(epos, m_epos);
                    if (m_polys.get(m_polys.size() - 1) != m_endRef) {
                        Result<ClosestPointOnPolyResult> result = m_navQuery
                                .closestPointOnPoly(m_polys.get(m_polys.size() - 1), m_epos);
                        if (result.succeeded()) {
                            epos = result.result.getClosest();
                        }
                    }
                    Result<List<StraightPathItem>> result = m_navQuery.findStraightPath(m_spos, epos, m_polys,
                            MAX_POLYS, NavMeshQuery.DT_STRAIGHTPATH_ALL_CROSSINGS);
                    if (result.succeeded()) {
                        m_straightPath = result.result;
                    }
                }
                m_pathFindStatus = Status.FAILURE;
            }

        }
    }
}
