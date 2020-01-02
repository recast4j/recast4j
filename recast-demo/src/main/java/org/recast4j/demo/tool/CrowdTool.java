package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_option_label;
import static org.lwjgl.nuklear.Nuklear.nk_option_text;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;
import static org.lwjgl.nuklear.Nuklear.nk_property_int;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;
import static org.lwjgl.nuklear.Nuklear.nk_tree_state_pop;
import static org.lwjgl.nuklear.Nuklear.nk_tree_state_push;
import static org.recast4j.demo.draw.DebugDraw.duDarkenCol;
import static org.recast4j.demo.draw.DebugDraw.duLerpCol;
import static org.recast4j.demo.draw.DebugDraw.duRGBA;
import static org.recast4j.demo.draw.DebugDrawPrimitives.LINES;
import static org.recast4j.demo.draw.DebugDrawPrimitives.QUADS;

import java.util.List;
import java.util.Optional;

import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.sample.Sample;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.DetourCommon;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Tupple2;
import org.recast4j.detour.crowd.Crowd;
import org.recast4j.detour.crowd.CrowdAgent;
import org.recast4j.detour.crowd.CrowdAgent.MoveRequestState;
import org.recast4j.detour.crowd.CrowdAgentParams;
import org.recast4j.detour.crowd.ObstacleAvoidanceQuery.ObstacleAvoidanceParams;
import org.recast4j.detour.crowd.ProximityGrid;
import org.recast4j.detour.crowd.debug.CrowdAgentDebugInfo;
import org.recast4j.detour.crowd.debug.ObstacleAvoidanceDebugData;

public class CrowdTool implements Tool {

    private enum ToolMode {
        CREATE, MOVE_TARGET, SELECT, TOGGLE_POLYS
    }

    private final CrowdToolParams toolParams = new CrowdToolParams();
    private Sample sample;
    private NavMesh m_nav;
    private Crowd crowd;
    private final CrowdAgentDebugInfo m_agentDebug = new CrowdAgentDebugInfo();

    private static final int AGENT_MAX_TRAIL = 64;
    private static final int MAX_AGENTS = 128;

    private class AgentTrail {
        float[] trail = new float[AGENT_MAX_TRAIL * 3];
        int htrail;
    };

    private final AgentTrail[] m_trails = new AgentTrail[MAX_AGENTS];
    private float[] m_targetPos;
    private long m_targetRef;
    private ToolMode m_mode = ToolMode.CREATE;
    private final boolean m_run = true;

    public CrowdTool() {
        for (int i = 0; i < m_trails.length; i++) {
            m_trails[i] = new AgentTrail();
        }
        m_agentDebug.vod = new ObstacleAvoidanceDebugData(2048);
        m_agentDebug.idx = -1;
    }

    @Override
    public void setSample(Sample psample) {
        // TODO Auto-generated method stub
        if (sample != psample) {
            sample = psample;
        }

        NavMesh nav = sample.getNavMesh();

        if (nav != null && m_nav != nav) {
            m_nav = nav;

            crowd = new Crowd(MAX_AGENTS, sample.getSettingsUI().getAgentRadius(), nav,
                    __ -> new DefaultQueryFilter(SampleAreaModifications.SAMPLE_POLYFLAGS_ALL,
                            SampleAreaModifications.SAMPLE_POLYFLAGS_DISABLED,
                            new float[] { 1f, 10f, 1f, 1f, 2f, 1.5f }));

            // Setup local avoidance params to different qualities.
            // Use mostly default settings, copy from dtCrowd.
            ObstacleAvoidanceParams params = new ObstacleAvoidanceParams(crowd.getObstacleAvoidanceParams(0));

            // Low (11)
            params.velBias = 0.5f;
            params.adaptiveDivs = 5;
            params.adaptiveRings = 2;
            params.adaptiveDepth = 1;
            crowd.setObstacleAvoidanceParams(0, params);

            // Medium (22)
            params.velBias = 0.5f;
            params.adaptiveDivs = 5;
            params.adaptiveRings = 2;
            params.adaptiveDepth = 2;
            crowd.setObstacleAvoidanceParams(1, params);

            // Good (45)
            params.velBias = 0.5f;
            params.adaptiveDivs = 7;
            params.adaptiveRings = 2;
            params.adaptiveDepth = 3;
            crowd.setObstacleAvoidanceParams(2, params);

            // High (66)
            params.velBias = 0.5f;
            params.adaptiveDivs = 7;
            params.adaptiveRings = 3;
            params.adaptiveDepth = 3;

            crowd.setObstacleAvoidanceParams(3, params);
        }
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {
        if (sample == null || crowd == null)
            return;
        DemoInputGeomProvider geom = sample.getInputGeom();
        if (geom == null)
            return;

        if (m_mode == ToolMode.CREATE) {
            if (shift) {
                // Delete
                int ahit = hitTestAgents(s, p);
                if (ahit != -1) {
                    removeAgent(ahit);
                }
            } else {
                // Add
                addAgent(p);
            }
        } else if (m_mode == ToolMode.MOVE_TARGET) {
            setMoveTarget(p, shift);
        } else if (m_mode == ToolMode.SELECT) {
            // Highlight
            int ahit = hitTestAgents(s, p);
            hilightAgent(ahit);
        } else if (m_mode == ToolMode.TOGGLE_POLYS) {
            NavMesh nav = sample.getNavMesh();
            NavMeshQuery navquery = sample.getNavMeshQuery();
            if (nav != null && navquery != null) {
                QueryFilter filter = new DefaultQueryFilter();
                float[] halfExtents = crowd.getQueryExtents();
                Result<FindNearestPolyResult> result = navquery.findNearestPoly(p, halfExtents, filter);
                long ref = result.result.getNearestRef();
                if (ref != 0) {
                    Result<Integer> flags = nav.getPolyFlags(ref);
                    if (flags.succeeded()) {
                        nav.setPolyFlags(ref, flags.result ^ SampleAreaModifications.SAMPLE_POLYFLAGS_DISABLED);
                    }
                }
            }
        }
    }

    private void removeAgent(int idx) {
        crowd.removeAgent(idx);
        if (idx == m_agentDebug.idx) {
            m_agentDebug.idx = -1;
        }
    }

    private void addAgent(float[] p) {

        CrowdAgentParams ap = new CrowdAgentParams();
        ap.radius = sample.getSettingsUI().getAgentRadius();
        ap.height = sample.getSettingsUI().getAgentHeight();
        ap.maxAcceleration = 8.0f;
        ap.maxSpeed = 3.5f;
        ap.collisionQueryRange = ap.radius * 12.0f;
        ap.pathOptimizationRange = ap.radius * 30.0f;
        ap.updateFlags = getUpdateFlags();
        ap.obstacleAvoidanceType = toolParams.m_obstacleAvoidanceType.get(0);
        ap.separationWeight = toolParams.m_separationWeight.get(0);

        int idx = crowd.addAgent(p, ap);
        if (idx != -1) {
            if (m_targetRef != 0)
                crowd.requestMoveTarget(idx, m_targetRef, m_targetPos);

            // Init trail
            AgentTrail trail = m_trails[idx];
            for (int i = 0; i < AGENT_MAX_TRAIL; ++i) {
                trail.trail[i * 3] = p[0];
                trail.trail[i * 3 + 1] = p[1];
                trail.trail[i * 3 + 2] = p[2];
            }
            trail.htrail = 0;
        }

    }

    private int hitTestAgents(float[] s, float[] p) {
        if (sample == null)
            return -1;

        int isel = -1;
        float tsel = Float.MAX_VALUE;

        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;
            float[] bmin = new float[3], bmax = new float[3];
            getAgentBounds(ag, bmin, bmax);
            Optional<Tupple2<Float, Float>> isect = isectSegAABB(s, p, bmin, bmax);
            if (isect.isPresent()) {
                float tmin = isect.get().first;
                float tmax = isect.get().second;
                if (tmin > 0 && tmin < tsel) {
                    isel = i;
                    tsel = tmin;
                }
            }
        }

        return isel;
    }

    private void getAgentBounds(CrowdAgent ag, float[] bmin, float[] bmax) {
        float[] p = ag.npos;
        float r = ag.params.radius;
        float h = ag.params.height;
        bmin[0] = p[0] - r;
        bmin[1] = p[1];
        bmin[2] = p[2] - r;
        bmax[0] = p[0] + r;
        bmax[1] = p[1] + h;
        bmax[2] = p[2] + r;
    }

    static Optional<Tupple2<Float, Float>> isectSegAABB(float[] sp, float[] sq, float[] amin, float[] amax) {
        float EPS = 1e-6f;

        float tmin;
        float tmax;
        float[] d = DetourCommon.vSub(sq, sp);
        tmin = 0; // set to -FLT_MAX to get first hit on line
        tmax = Float.MAX_VALUE; // set to max distance ray can travel (for segment)

        // For all three slabs
        for (int i = 0; i < 3; i++) {
            if (Math.abs(d[i]) < EPS) {
                // Ray is parallel to slab. No hit if origin not within slab
                if (sp[i] < amin[i] || sp[i] > amax[i])
                    return Optional.empty();
            } else {
                // Compute intersection t value of ray with near and far plane of slab
                float ood = 1.0f / d[i];
                float t1 = (amin[i] - sp[i]) * ood;
                float t2 = (amax[i] - sp[i]) * ood;
                // Make t1 be intersection with near plane, t2 with far plane
                if (t1 > t2) {
                    float tt = t1;
                    t1 = t2;
                    t2 = tt;
                }
                // Compute the intersection of slab intersections intervals
                if (t1 > tmin)
                    tmin = t1;
                if (t2 < tmax)
                    tmax = t2;
                // Exit with no collision as soon as slab intersection becomes empty
                if (tmin > tmax)
                    return Optional.empty();
            }
        }

        return Optional.of(new Tupple2<>(tmin, tmax));
    }

    void setMoveTarget(float[] p, boolean adjust) {
        if (sample == null || crowd == null)
            return;

        // Find nearest point on navmesh and set move request to that location.
        NavMeshQuery navquery = sample.getNavMeshQuery();
        QueryFilter filter = crowd.getFilter(0);
        float[] halfExtents = crowd.getQueryExtents();

        if (adjust) {
            // Request velocity
            if (m_agentDebug.idx != -1) {
                CrowdAgent ag = crowd.getAgent(m_agentDebug.idx);
                if (ag != null && ag.active) {
                    float[] vel = calcVel(ag.npos, p, ag.params.maxSpeed);
                    crowd.requestMoveVelocity(m_agentDebug.idx, vel);
                }
            } else {
                for (int i = 0; i < crowd.getAgentCount(); ++i) {
                    CrowdAgent ag = crowd.getAgent(i);
                    if (!ag.active)
                        continue;
                    float[] vel = calcVel(ag.npos, p, ag.params.maxSpeed);
                    crowd.requestMoveVelocity(i, vel);
                }
            }
        } else {
            Result<FindNearestPolyResult> result = navquery.findNearestPoly(p, halfExtents, filter);
            m_targetRef = result.result.getNearestRef();
            m_targetPos = result.result.getNearestPos();
            if (m_agentDebug.idx != -1) {
                CrowdAgent ag = crowd.getAgent(m_agentDebug.idx);
                if (ag != null && ag.active) {
                    crowd.requestMoveTarget(m_agentDebug.idx, m_targetRef, m_targetPos);
                }
            } else {
                for (int i = 0; i < crowd.getAgentCount(); ++i) {
                    CrowdAgent ag = crowd.getAgent(i);
                    if (!ag.active) {
                        continue;
                    }
                    crowd.requestMoveTarget(i, m_targetRef, m_targetPos);
                }
            }
        }
    }

    private float[] calcVel(float[] pos, float[] tgt, float speed) {
        float[] vel = DetourCommon.vSub(tgt, pos);
        vel[1] = 0.0f;
        DetourCommon.vNormalize(vel);
        return DetourCommon.vScale(vel, speed);
    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        RecastDebugDraw dd = renderer.getDebugDraw();
        float rad = sample.getSettingsUI().getAgentRadius();
        NavMesh nav = sample.getNavMesh();
        if (nav == null || crowd == null)
            return;

        if (toolParams.m_showNodes && crowd.getPathQueue() != null) {
            NavMeshQuery navquery = crowd.getPathQueue().getNavQuery();
            if (navquery != null) {
                dd.debugDrawNavMeshNodes(navquery);
            }
        }
        dd.depthMask(false);

        // Draw paths
        if (toolParams.m_showPath) {
            for (int i = 0; i < crowd.getAgentCount(); i++) {
                if (!toolParams.m_showDetailAll && i != m_agentDebug.idx)
                    continue;
                CrowdAgent ag = crowd.getAgent(i);
                if (!ag.active)
                    continue;
                List<Long> path = ag.corridor.getPath();
                int npath = ag.corridor.getPathCount();
                for (int j = 0; j < npath; ++j) {
                    dd.debugDrawNavMeshPoly(nav, path.get(j), duRGBA(255, 255, 255, 24));
                }
            }
        }

        if (m_targetRef != 0)
            dd.debugDrawCross(m_targetPos[0], m_targetPos[1] + 0.1f, m_targetPos[2], rad,
                    duRGBA(255, 255, 255, 192), 2.0f);

        // Occupancy grid.
        if (toolParams.m_showGrid) {
            float gridy = -Float.MAX_VALUE;
            for (int i = 0; i < crowd.getAgentCount(); ++i) {
                CrowdAgent ag = crowd.getAgent(i);
                if (!ag.active)
                    continue;
                float[] pos = ag.corridor.getPos();
                gridy = Math.max(gridy, pos[1]);
            }
            gridy += 1.0f;

            dd.begin(QUADS);
            ProximityGrid grid = crowd.getGrid();
            int[] bounds = grid.getBounds();
            float cs = grid.getCellSize();
            for (int y = bounds[1]; y <= bounds[3]; ++y) {
                for (int x = bounds[0]; x <= bounds[2]; ++x) {
                    int count = grid.getItemCountAt(x, y);
                    if (count == 0)
                        continue;
                    int col = duRGBA(128, 0, 0, Math.min(count * 40, 255));
                    dd.vertex(x * cs, gridy, y * cs, col);
                    dd.vertex(x * cs, gridy, y * cs + cs, col);
                    dd.vertex(x * cs + cs, gridy, y * cs + cs, col);
                    dd.vertex(x * cs + cs, gridy, y * cs, col);
                }
            }
            dd.end();
        }

        // Trail
        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;

            AgentTrail trail = m_trails[i];
            float[] pos = ag.npos;

            dd.begin(LINES, 3.0f);
            float[] prev = new float[3];
            float preva = 1;
            DetourCommon.vCopy(prev, pos);
            for (int j = 0; j < AGENT_MAX_TRAIL - 1; ++j) {
                int idx = (trail.htrail + AGENT_MAX_TRAIL - j) % AGENT_MAX_TRAIL;
                int v = idx * 3;
                float a = 1 - j / (float) AGENT_MAX_TRAIL;
                dd.vertex(prev[0], prev[1] + 0.1f, prev[2], duRGBA(0, 0, 0, (int) (128 * preva)));
                dd.vertex(trail.trail[v], trail.trail[v + 1] + 0.1f, trail.trail[v + 2],
                        duRGBA(0, 0, 0, (int) (128 * a)));
                preva = a;
                DetourCommon.vCopy(prev, trail.trail, v);
            }
            dd.end();

        }

        // Corners & co
        for (int i = 0; i < crowd.getAgentCount(); i++) {
            if (toolParams.m_showDetailAll == false && i != m_agentDebug.idx)
                continue;
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;

            float radius = ag.params.radius;
            float[] pos = ag.npos;

            if (toolParams.m_showCorners) {
                if (!ag.corners.isEmpty()) {
                    dd.begin(LINES, 2.0f);
                    for (int j = 0; j < ag.corners.size(); ++j) {
                        float[] va = j == 0 ? pos : ag.corners.get(j - 1).getPos();
                        float[] vb = ag.corners.get(j).getPos();
                        dd.vertex(va[0], va[1] + radius, va[2], duRGBA(128, 0, 0, 192));
                        dd.vertex(vb[0], vb[1] + radius, vb[2], duRGBA(128, 0, 0, 192));
                    }
                    if ((ag.corners.get(ag.corners.size() - 1).getFlags()
                            & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0) {
                        float[] v = ag.corners.get(ag.corners.size() - 1).getPos();
                        dd.vertex(v[0], v[1], v[2], duRGBA(192, 0, 0, 192));
                        dd.vertex(v[0], v[1] + radius * 2, v[2], duRGBA(192, 0, 0, 192));
                    }

                    dd.end();

                    if (toolParams.m_anticipateTurns) {
                        /*                  float dvel[3], pos[3];
                         calcSmoothSteerDirection(ag.pos, ag.cornerVerts, ag.ncorners, dvel);
                         pos[0] = ag.pos[0] + dvel[0];
                         pos[1] = ag.pos[1] + dvel[1];
                         pos[2] = ag.pos[2] + dvel[2];

                         float off = ag.radius+0.1f;
                         float[] tgt = &ag.cornerVerts[0];
                         float y = ag.pos[1]+off;

                         dd.begin(DU_DRAW_LINES, 2.0f);

                         dd.vertex(ag.pos[0],y,ag.pos[2], duRGBA(255,0,0,192));
                         dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));

                         dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
                         dd.vertex(tgt[0],y,tgt[2], duRGBA(255,0,0,192));

                         dd.end();*/
                    }
                }
            }

            if (toolParams.m_showCollisionSegments) {
                float[] center = ag.boundary.getCenter();
                dd.debugDrawCross(center[0], center[1] + radius, center[2], 0.2f, duRGBA(192, 0, 128, 255),
                        2.0f);
                dd.debugDrawCircle(center[0], center[1] + radius, center[2], ag.params.collisionQueryRange,
                        duRGBA(192, 0, 128, 128), 2.0f);

                dd.begin(LINES, 3.0f);
                for (int j = 0; j < ag.boundary.getSegmentCount(); ++j) {
                    int col = duRGBA(192, 0, 128, 192);
                    float[] s = ag.boundary.getSegment(j);
                    float[] s0 = new float[] { s[0], s[1], s[2] };
                    float[] s3 = new float[] { s[3], s[4], s[5] };
                    if (DetourCommon.triArea2D(pos, s0, s3) < 0.0f)
                        col = duDarkenCol(col);

                    dd.appendArrow(s[0], s[1] + 0.2f, s[2], s[3], s[4] + 0.2f, s[5], 0.0f, 0.3f, col);
                }
                dd.end();
            }

            if (toolParams.m_showNeis) {
                dd.debugDrawCircle(pos[0], pos[1] + radius, pos[2], ag.params.collisionQueryRange,
                        duRGBA(0, 192, 128, 128), 2.0f);

                dd.begin(LINES, 2.0f);
                for (int j = 0; j < ag.neis.size(); ++j) {
                    // Get 'n'th active agent.
                    // TODO: fix this properly.
                    CrowdAgent nei = crowd.getAgent(ag.neis.get(j).idx);
                    if (nei != null) {
                        dd.vertex(pos[0], pos[1] + radius, pos[2], duRGBA(0, 192, 128, 128));
                        dd.vertex(nei.npos[0], nei.npos[1] + radius, nei.npos[2], duRGBA(0, 192, 128, 128));
                    }
                }
                dd.end();
            }

            if (toolParams.m_showOpt) {
                dd.begin(LINES, 2.0f);
                dd.vertex(m_agentDebug.optStart[0], m_agentDebug.optStart[1] + 0.3f, m_agentDebug.optStart[2],
                        duRGBA(0, 128, 0, 192));
                dd.vertex(m_agentDebug.optEnd[0], m_agentDebug.optEnd[1] + 0.3f, m_agentDebug.optEnd[2],
                        duRGBA(0, 128, 0, 192));
                dd.end();
            }
        }

        // Agent cylinders.
        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;

            float radius = ag.params.radius;
            float[] pos = ag.npos;

            int col = duRGBA(0, 0, 0, 32);
            if (m_agentDebug.idx == i)
                col = duRGBA(255, 0, 0, 128);

            dd.debugDrawCircle(pos[0], pos[1], pos[2], radius, col, 2.0f);
        }

        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;

            float height = ag.params.height;
            float radius = ag.params.radius;
            float[] pos = ag.npos;

            int col = duRGBA(220, 220, 220, 128);
            if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING
                    || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
                col = duLerpCol(col, duRGBA(128, 0, 255, 128), 32);
            else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
                col = duLerpCol(col, duRGBA(128, 0, 255, 128), 128);
            else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_FAILED)
                col = duRGBA(255, 32, 16, 128);
            else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
                col = duLerpCol(col, duRGBA(64, 255, 0, 128), 128);

            dd.debugDrawCylinder(pos[0] - radius, pos[1] + radius * 0.1f, pos[2] - radius, pos[0] + radius,
                    pos[1] + height, pos[2] + radius, col);
        }

        if (toolParams.m_showVO) {
            for (int i = 0; i < crowd.getAgentCount(); i++) {
                if (toolParams.m_showDetailAll == false && i != m_agentDebug.idx)
                    continue;
                CrowdAgent ag = crowd.getAgent(i);
                if (!ag.active)
                    continue;

                // Draw detail about agent sela
                ObstacleAvoidanceDebugData vod = m_agentDebug.vod;

                float dx = ag.npos[0];
                float dy = ag.npos[1] + ag.params.height;
                float dz = ag.npos[2];

                dd.debugDrawCircle(dx, dy, dz, ag.params.maxSpeed, duRGBA(255, 255, 255, 64), 2.0f);

                dd.begin(QUADS);
                for (int j = 0; j < vod.getSampleCount(); ++j) {
                    float[] p = vod.getSampleVelocity(j);
                    float sr = vod.getSampleSize(j);
                    float pen = vod.getSamplePenalty(j);
                    float pen2 = vod.getSamplePreferredSidePenalty(j);
                    int col = duLerpCol(duRGBA(255, 255, 255, 220),
                            duRGBA(128, 96, 0, 220), (int) (pen * 255));
                    col = duLerpCol(col, duRGBA(128, 0, 0, 220), (int) (pen2 * 128));
                    dd.vertex(dx + p[0] - sr, dy, dz + p[2] - sr, col);
                    dd.vertex(dx + p[0] - sr, dy, dz + p[2] + sr, col);
                    dd.vertex(dx + p[0] + sr, dy, dz + p[2] + sr, col);
                    dd.vertex(dx + p[0] + sr, dy, dz + p[2] - sr, col);
                }
                dd.end();
            }
        }

        // Velocity stuff.
        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;

            float radius = ag.params.radius;
            float height = ag.params.height;
            float[] pos = ag.npos;
            float[] vel = ag.vel;
            float[] dvel = ag.dvel;

            int col = duRGBA(220, 220, 220, 192);
            if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING
                    || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
                col = duLerpCol(col, duRGBA(128, 0, 255, 192), 32);
            else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
                col = duLerpCol(col, duRGBA(128, 0, 255, 192), 128);
            else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_FAILED)
                col = duRGBA(255, 32, 16, 192);
            else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
                col = duLerpCol(col, duRGBA(64, 255, 0, 192), 128);

            dd.debugDrawCircle(pos[0], pos[1] + height, pos[2], radius, col, 2.0f);

            dd.debugDrawArrow(pos[0], pos[1] + height, pos[2], pos[0] + dvel[0], pos[1] + height + dvel[1],
                    pos[2] + dvel[2], 0.0f, 0.4f, duRGBA(0, 192, 255, 192),
                    (m_agentDebug.idx == i) ? 2.0f : 1.0f);

            dd.debugDrawArrow(pos[0], pos[1] + height, pos[2], pos[0] + vel[0], pos[1] + height + vel[1],
                    pos[2] + vel[2], 0.0f, 0.4f, duRGBA(0, 0, 0, 160), 2.0f);
        }

        dd.depthMask(true);
    }

    @Override
    public void handleUpdate(float dt) {
        if (m_run) {
            updateTick(dt);
        }
    }

    private void updateTick(float dt) {
        if (crowd == null)
            return;
        NavMesh nav = sample.getNavMesh();
        if (nav == null)
            return;

        // TimeVal startTime = getPerfTime();

        crowd.update(dt, m_agentDebug);

        // TimeVal endTime = getPerfTime();

        // Update agent trails
        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            AgentTrail trail = m_trails[i];
            if (!ag.active) {
                continue;
            }
            // Update agent movement trail.
            trail.htrail = (trail.htrail + 1) % AGENT_MAX_TRAIL;
            trail.trail[trail.htrail * 3] = ag.npos[0];
            trail.trail[trail.htrail * 3 + 1] = ag.npos[1];
            trail.trail[trail.htrail * 3 + 2] = ag.npos[2];
        }

        m_agentDebug.vod.normalizeSamples();

        // m_crowdSampleCount.addSample((float) crowd.getVelocitySampleCount());
        // m_crowdTotalTime.addSample(getPerfTimeUsec(endTime - startTime) / 1000.0f);
    }

    private void hilightAgent(int idx) {
        m_agentDebug.idx = idx;
    }

    @Override
    public void layout(NkContext ctx) {
        ToolMode previousToolMode = m_mode;
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Create Agents", m_mode == ToolMode.CREATE)) {
            m_mode = ToolMode.CREATE;
        }
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Move Target", m_mode == ToolMode.MOVE_TARGET)) {
            m_mode = ToolMode.MOVE_TARGET;
        }
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Select Agent", m_mode == ToolMode.SELECT)) {
            m_mode = ToolMode.SELECT;
        }
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Toggle Polys", m_mode == ToolMode.TOGGLE_POLYS)) {
            m_mode = ToolMode.TOGGLE_POLYS;
        }
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_tree_state_push(ctx, 0, "Options", toolParams.m_expandOptions)) {
            boolean m_optimizeVis = toolParams.m_optimizeVis;
            boolean m_optimizeTopo = toolParams.m_optimizeTopo;
            boolean m_anticipateTurns = toolParams.m_anticipateTurns;
            boolean m_obstacleAvoidance = toolParams.m_obstacleAvoidance;
            boolean m_separation = toolParams.m_separation;
            int m_obstacleAvoidanceType = toolParams.m_obstacleAvoidanceType.get(0);
            float m_separationWeight = toolParams.m_separationWeight.get(0);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_optimizeVis = nk_option_text(ctx, "Optimize Visibility", toolParams.m_optimizeVis);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_optimizeTopo = nk_option_text(ctx, "Optimize Topology", toolParams.m_optimizeTopo);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_anticipateTurns = nk_option_text(ctx, "Anticipate Turns", toolParams.m_anticipateTurns);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_obstacleAvoidance = nk_option_text(ctx, "Obstacle Avoidance",
                    toolParams.m_obstacleAvoidance);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_int(ctx, "Avoidance Quality", 0, toolParams.m_obstacleAvoidanceType, 3, 1, 0.1f);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_separation = nk_option_text(ctx, "Separation", toolParams.m_separation);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Separation Weight", 0f, toolParams.m_separationWeight, 20f, 0.01f, 0.01f);
            if (m_optimizeVis != toolParams.m_optimizeVis || m_optimizeTopo != toolParams.m_optimizeTopo
                    || m_anticipateTurns != toolParams.m_anticipateTurns
                    || m_obstacleAvoidance != toolParams.m_obstacleAvoidance
                    || m_separation != toolParams.m_separation
                    || m_obstacleAvoidanceType != toolParams.m_obstacleAvoidanceType.get(0)
                    || m_separationWeight != toolParams.m_separationWeight.get(0)) {
                updateAgentParams();
            }
            nk_tree_state_pop(ctx);
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        if (nk_tree_state_push(ctx, 0, "Selected Debug Draw", toolParams.m_expandSelectedDebugDraw)) {
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_showCorners = nk_option_text(ctx, "Show Corners", toolParams.m_showCorners);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_showCollisionSegments = nk_option_text(ctx, "Show Collision Segs",
                    toolParams.m_showCollisionSegments);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_showPath = nk_option_text(ctx, "Show Path", toolParams.m_showPath);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_showVO = nk_option_text(ctx, "Show VO", toolParams.m_showVO);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_showOpt = nk_option_text(ctx, "Show Path Optimization", toolParams.m_showOpt);
            nk_layout_row_dynamic(ctx, 20, 1);
            toolParams.m_showNeis = nk_option_text(ctx, "Show Neighbours", toolParams.m_showNeis);
            nk_tree_state_pop(ctx);
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
        if (nk_tree_state_push(ctx, 0, "Debug Draw", toolParams.m_expandDebugDraw)) {
            nk_tree_state_pop(ctx);
        }
        nk_layout_row_dynamic(ctx, 5, 1);
        nk_spacing(ctx, 1);
    }

    private void updateAgentParams() {
        if (crowd == null) {
            return;
        }

        int updateFlags = getUpdateFlags();
        for (int i = 0; i < crowd.getAgentCount(); ++i) {
            CrowdAgent ag = crowd.getAgent(i);
            if (!ag.active)
                continue;
            CrowdAgentParams params = new CrowdAgentParams();
            params.radius = ag.params.radius;
            params.height = ag.params.height;
            params.maxAcceleration = ag.params.maxAcceleration;
            params.maxSpeed = ag.params.maxSpeed;
            params.collisionQueryRange = ag.params.collisionQueryRange;
            params.pathOptimizationRange = ag.params.pathOptimizationRange;
            params.obstacleAvoidanceType = ag.params.obstacleAvoidanceType;
            params.queryFilterType = ag.params.queryFilterType;
            params.userData = ag.params.userData;
            params.updateFlags = updateFlags;
            params.obstacleAvoidanceType = toolParams.m_obstacleAvoidanceType.get(0);
            params.separationWeight = toolParams.m_separationWeight.get(0);
            crowd.updateAgentParameters(i, params);
        }
    }

    private int getUpdateFlags() {
        int updateFlags = 0;
        if (toolParams.m_anticipateTurns) {
            updateFlags |= CrowdAgentParams.DT_CROWD_ANTICIPATE_TURNS;
        }
        if (toolParams.m_optimizeVis) {
            updateFlags |= CrowdAgentParams.DT_CROWD_OPTIMIZE_VIS;
        }
        if (toolParams.m_optimizeTopo) {
            updateFlags |= CrowdAgentParams.DT_CROWD_OPTIMIZE_TOPO;
        }
        if (toolParams.m_obstacleAvoidance) {
            updateFlags |= CrowdAgentParams.DT_CROWD_OBSTACLE_AVOIDANCE;
        }
        if (toolParams.m_separation) {
            updateFlags |= CrowdAgentParams.DT_CROWD_SEPARATION;
        }
        return updateFlags;
    }

    @Override
    public String getName() {
        return "Crowd";
    }

}
