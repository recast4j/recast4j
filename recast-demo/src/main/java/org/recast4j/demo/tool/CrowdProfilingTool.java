/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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

import static java.util.stream.Collectors.toList;
import static org.lwjgl.nuklear.Nuklear.NK_TEXT_ALIGN_LEFT;
import static org.lwjgl.nuklear.Nuklear.nk_button_text;
import static org.lwjgl.nuklear.Nuklear.nk_label;
import static org.lwjgl.nuklear.Nuklear.nk_layout_row_dynamic;
import static org.lwjgl.nuklear.Nuklear.nk_property_float;
import static org.lwjgl.nuklear.Nuklear.nk_property_int;
import static org.lwjgl.nuklear.Nuklear.nk_spacing;
import static org.lwjgl.nuklear.Nuklear.nk_tree_state_pop;
import static org.lwjgl.nuklear.Nuklear.nk_tree_state_push;
import static org.recast4j.demo.draw.DebugDraw.duLerpCol;
import static org.recast4j.demo.draw.DebugDraw.duRGBA;
import static org.recast4j.demo.math.DemoMath.vDistSqr;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import org.lwjgl.BufferUtils;
import org.lwjgl.nuklear.NkContext;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.draw.RecastDebugDraw;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.FindRandomPointResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.NavMeshQuery.FRand;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Result;
import org.recast4j.detour.Tupple2;
import org.recast4j.detour.crowd.Crowd;
import org.recast4j.detour.crowd.CrowdAgent;
import org.recast4j.detour.crowd.CrowdAgent.MoveRequestState;
import org.recast4j.detour.crowd.CrowdAgentParams;
import org.recast4j.detour.crowd.CrowdConfig;
import org.recast4j.detour.crowd.ObstacleAvoidanceQuery.ObstacleAvoidanceParams;
import org.recast4j.recast.RecastVectors;

public class CrowdProfilingTool {

    private final Supplier<CrowdAgentParams> agentParamsSupplier;
    private final IntBuffer expandDemoOptions = BufferUtils.createIntBuffer(1).put(0, 1);
    private final IntBuffer agents = BufferUtils.createIntBuffer(1).put(0, 1000);
    private final IntBuffer randomSeed = BufferUtils.createIntBuffer(1).put(0, 270);
    private final IntBuffer numberOfZones = BufferUtils.createIntBuffer(1).put(0, 4);
    private final FloatBuffer zoneRadius = BufferUtils.createFloatBuffer(1).put(0, 20);
    private final FloatBuffer percentMobs = BufferUtils.createFloatBuffer(1).put(0, 80);
    private final FloatBuffer percentTravellers = BufferUtils.createFloatBuffer(1).put(0, 15);
    private final IntBuffer pathQueueSize = BufferUtils.createIntBuffer(1).put(0, 32);
    private final IntBuffer maxIterations = BufferUtils.createIntBuffer(1).put(0, 300);
    private Crowd crowd;
    private NavMesh navMesh;
    private CrowdConfig config;
    private FRand rnd;
    private final List<FindRandomPointResult> zones = new ArrayList<>();
    private long crowdUpdateTime;

    public CrowdProfilingTool(Supplier<CrowdAgentParams> agentParamsSupplier) {
        this.agentParamsSupplier = agentParamsSupplier;
    }

    public void layout(NkContext ctx) {
        nk_layout_row_dynamic(ctx, 2, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_tree_state_push(ctx, 0, "Simulation Options", expandDemoOptions)) {
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_int(ctx, "Agents", 0, agents, 10000, 1, 1);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_int(ctx, "Random Seed", 0, randomSeed, 1024, 1, 1);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_int(ctx, "Number of Zones", 0, numberOfZones, 10, 1, 1);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Zone Radius", 0, zoneRadius, 100, 1, 1);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Mobs %", 0, percentMobs, 100, 1, 1);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_float(ctx, "Travellers %", 0, percentTravellers, 100, 1, 1);
            nk_tree_state_pop(ctx);
        }
        if (nk_tree_state_push(ctx, 0, "Crowd Options", expandDemoOptions)) {
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_int(ctx, "Path Queue Size", 0, pathQueueSize, 1024, 1, 1);
            nk_layout_row_dynamic(ctx, 20, 1);
            nk_property_int(ctx, "Max Iterations", 0, maxIterations, 4000, 1, 1);
            nk_tree_state_pop(ctx);
        }
        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_button_text(ctx, "Start")) {
            if (navMesh != null) {
                rnd = new FRand(randomSeed.get(0));
                createCrowd();
                createZones();
                NavMeshQuery navquery = new NavMeshQuery(navMesh);
                QueryFilter filter = new DefaultQueryFilter();
                for (int i = 0; i < agents.get(0); i++) {
                    float tr = rnd.frand();
                    AgentType type = AgentType.MOB;
                    float mobsPcnt = percentMobs.get(0) / 100f;
                    if (tr > mobsPcnt) {
                        tr = rnd.frand();
                        float travellerPcnt = percentTravellers.get(0) / 100f;
                        if (tr > travellerPcnt) {
                            type = AgentType.VILLAGER;
                        } else {
                            type = AgentType.TRAVELLER;
                        }
                    }
                    float[] pos = null;
                    switch (type) {
                    case MOB:
                        pos = getMobPosition(navquery, filter, pos);
                        break;
                    case VILLAGER:
                        pos = getVillagerPosition(navquery, filter, pos);
                        break;
                    case TRAVELLER:
                        pos = getVillagerPosition(navquery, filter, pos);
                        break;
                    }
                    if (pos != null) {
                        addAgent(pos, type);
                    }
                }
            }
        }
        if (crowd != null) {
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, String.format("Max time to enqueue request: %.3f s", crowd.telemetry().maxTimeToEnqueueRequest()),
                    NK_TEXT_ALIGN_LEFT);
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, String.format("Max time to find path: %.3f s", crowd.telemetry().maxTimeToFindPath()),
                    NK_TEXT_ALIGN_LEFT);
            List<Tupple2<String, Long>> timings = crowd.telemetry().executionTimings().entrySet().stream()
                    .map(e -> new Tupple2<>(e.getKey(), e.getValue())).sorted((t1, t2) -> Long.compare(t2.second, t1.second))
                    .collect(toList());
            for (Tupple2<String, Long> e : timings) {
                nk_layout_row_dynamic(ctx, 18, 1);
                nk_label(ctx, String.format("%s: %d us", e.first, e.second / 1_000), NK_TEXT_ALIGN_LEFT);
            }
            nk_layout_row_dynamic(ctx, 1, 1);
            nk_spacing(ctx, 1);
            nk_layout_row_dynamic(ctx, 18, 1);
            nk_label(ctx, String.format("Update Time: %d ms", crowdUpdateTime), NK_TEXT_ALIGN_LEFT);
        }
    }

    private float[] getMobPosition(NavMeshQuery navquery, QueryFilter filter, float[] pos) {
        Result<FindRandomPointResult> result = navquery.findRandomPoint(filter, rnd);
        if (result.succeeded()) {
            pos = result.result.getRandomPt();
        }
        return pos;
    }

    private float[] getVillagerPosition(NavMeshQuery navquery, QueryFilter filter, float[] pos) {
        if (!zones.isEmpty()) {
            int zone = (int) (rnd.frand() * zones.size());
            Result<FindRandomPointResult> result = navquery.findRandomPointWithinCircle(zones.get(zone).getRandomRef(),
                    zones.get(zone).getRandomPt(), zoneRadius.get(0), filter, rnd);
            if (result.succeeded()) {
                pos = result.result.getRandomPt();
            }
        }
        return pos;
    }

    private void createZones() {
        zones.clear();
        QueryFilter filter = new DefaultQueryFilter();
        NavMeshQuery navquery = new NavMeshQuery(navMesh);
        for (int i = 0; i < numberOfZones.get(0); i++) {
            float zoneSeparation = zoneRadius.get(0) * zoneRadius.get(0) * 16;
            for (int k = 0; k < 100; k++) {
                Result<FindRandomPointResult> result = navquery.findRandomPoint(filter, rnd);
                if (result.succeeded()) {
                    boolean valid = true;
                    for (FindRandomPointResult zone : zones) {
                        if (vDistSqr(zone.getRandomPt(), result.result.getRandomPt(), 0) < zoneSeparation) {
                            valid = false;
                            break;
                        }
                    }
                    if (valid) {
                        zones.add(result.result);
                        break;
                    }
                }
            }
        }
    }

    private void createCrowd() {
        crowd = new Crowd(config, navMesh, __ -> new DefaultQueryFilter(SampleAreaModifications.SAMPLE_POLYFLAGS_ALL,
                SampleAreaModifications.SAMPLE_POLYFLAGS_DISABLED, new float[] { 1f, 10f, 1f, 1f, 2f, 1.5f }));

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

    public void update(float dt) {
        long startTime = System.nanoTime();
        if (crowd != null) {
            crowd.config().pathQueueSize = pathQueueSize.get(0);
            crowd.config().maxFindPathIterations = maxIterations.get(0);
            crowd.update(dt, null);
        }
        long endTime = System.nanoTime();
        if (crowd != null) {

            NavMeshQuery navquery = new NavMeshQuery(navMesh);
            QueryFilter filter = new DefaultQueryFilter();
            for (CrowdAgent ag : crowd.getActiveAgents()) {
                if (needsNewTarget(ag)) {
                    AgentData agentData = (AgentData) ag.params.userData;
                    switch (agentData.type) {
                    case MOB:
                        moveMob(navquery, filter, ag, agentData);
                        break;
                    case VILLAGER:
                        moveVillager(navquery, filter, ag, agentData);
                        break;
                    case TRAVELLER:
                        moveTraveller(navquery, filter, ag, agentData);
                        break;
                    }
                }
            }
        }
        crowdUpdateTime = (endTime - startTime) / 1_000_000;
    }

    private void moveMob(NavMeshQuery navquery, QueryFilter filter, CrowdAgent ag, AgentData agentData) {
        // Move somewhere
        Result<FindNearestPolyResult> nearestPoly = navquery.findNearestPoly(ag.npos, crowd.getQueryExtents(), filter);
        if (nearestPoly.succeeded()) {
            Result<FindRandomPointResult> result = navquery.findRandomPointAroundCircle(nearestPoly.result.getNearestRef(),
                    agentData.home, zoneRadius.get(0) * 2f, filter, rnd);
            if (result.succeeded()) {
                crowd.requestMoveTarget(ag, result.result.getRandomRef(), result.result.getRandomPt());
            }
        }
    }

    private void moveVillager(NavMeshQuery navquery, QueryFilter filter, CrowdAgent ag, AgentData agentData) {
        // Move somewhere close
        Result<FindNearestPolyResult> nearestPoly = navquery.findNearestPoly(ag.npos, crowd.getQueryExtents(), filter);
        if (nearestPoly.succeeded()) {
            Result<FindRandomPointResult> result = navquery.findRandomPointAroundCircle(nearestPoly.result.getNearestRef(),
                    agentData.home, zoneRadius.get(0) * 0.2f, filter, rnd);
            if (result.succeeded()) {
                crowd.requestMoveTarget(ag, result.result.getRandomRef(), result.result.getRandomPt());
            }
        }
    }

    private void moveTraveller(NavMeshQuery navquery, QueryFilter filter, CrowdAgent ag, AgentData agentData) {
        // Move to another zone
        List<FindRandomPointResult> potentialTargets = new ArrayList<>();
        for (FindRandomPointResult zone : zones) {
            if (vDistSqr(zone.getRandomPt(), ag.npos, 0) > zoneRadius.get(0) * zoneRadius.get(0)) {
                potentialTargets.add(zone);
            }
        }
        if (!potentialTargets.isEmpty()) {
            Collections.shuffle(potentialTargets);
            crowd.requestMoveTarget(ag, potentialTargets.get(0).getRandomRef(), potentialTargets.get(0).getRandomPt());
        }
    }

    private boolean needsNewTarget(CrowdAgent ag) {
        if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE
                || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_FAILED) {
            return true;
        }
        if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VALID) {
            float dx = ag.targetPos[0] - ag.npos[0];
            float dy = ag.targetPos[1] - ag.npos[1];
            float dz = ag.targetPos[2] - ag.npos[2];
            return dx * dx + dy * dy + dz * dz < 0.3f;
        }
        return false;
    }

    public void setup(float maxAgentRadius, NavMesh nav) {
        navMesh = nav;
        if (nav != null) {
            config = new CrowdConfig(maxAgentRadius);
        }
    }

    public void handleRender(NavMeshRenderer renderer) {
        RecastDebugDraw dd = renderer.getDebugDraw();
        dd.depthMask(false);
        if (crowd != null) {
            for (CrowdAgent ag : crowd.getActiveAgents()) {
                float radius = ag.params.radius;
                float[] pos = ag.npos;
                dd.debugDrawCircle(pos[0], pos[1], pos[2], radius, duRGBA(0, 0, 0, 32), 2.0f);
            }

            for (CrowdAgent ag : crowd.getActiveAgents()) {
                AgentData agentData = (AgentData) ag.params.userData;

                float height = ag.params.height;
                float radius = ag.params.radius;
                float[] pos = ag.npos;

                int col = duRGBA(220, 220, 220, 128);
                if (agentData.type == AgentType.TRAVELLER) {
                    col = duRGBA(100, 160, 100, 128);
                }
                if (agentData.type == AgentType.VILLAGER) {
                    col = duRGBA(120, 80, 160, 128);
                }
                if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING
                        || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
                    col = duLerpCol(col, duRGBA(255, 255, 32, 128), 128);
                else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
                    col = duLerpCol(col, duRGBA(255, 64, 32, 128), 128);
                else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_FAILED)
                    col = duRGBA(255, 32, 16, 128);
                else if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
                    col = duLerpCol(col, duRGBA(64, 255, 0, 128), 128);

                dd.debugDrawCylinder(pos[0] - radius, pos[1] + radius * 0.1f, pos[2] - radius, pos[0] + radius, pos[1] + height,
                        pos[2] + radius, col);
            }
        }

        dd.depthMask(true);
    }

    private CrowdAgent addAgent(float[] p, AgentType type) {
        CrowdAgentParams ap = agentParamsSupplier.get();
        ap.userData = new AgentData(type, p);
        return crowd.addAgent(p, ap);
    }

    private enum AgentType {
        VILLAGER, TRAVELLER, MOB,
    }

    private static class AgentData {
        private final AgentType type;
        private final float[] home = new float[3];

        public AgentData(AgentType type, float[] home) {
            this.type = type;
            RecastVectors.copy(this.home, home);
        }

    }

    public void updateAgentParams(int updateFlags, int obstacleAvoidanceType, float separationWeight) {
        for (CrowdAgent ag : crowd.getActiveAgents()) {
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
            params.obstacleAvoidanceType = obstacleAvoidanceType;
            params.separationWeight = separationWeight;
            crowd.updateAgentParameters(ag, params);
        }
    }
}
