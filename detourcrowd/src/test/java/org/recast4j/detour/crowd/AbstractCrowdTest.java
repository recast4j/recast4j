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
package org.recast4j.detour.crowd;

import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vScale;
import static org.recast4j.detour.DetourCommon.vSub;

import org.junit.Before;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.RecastTestMeshBuilder;
import org.recast4j.detour.Result;
import org.recast4j.detour.crowd.ObstacleAvoidanceQuery.ObstacleAvoidanceParams;

public class AbstractCrowdTest {

    protected final long[] startRefs = { 281474976710696L, 281474976710773L, 281474976710680L, 281474976710753L,
            281474976710733L };

    protected final long[] endRefs = { 281474976710721L, 281474976710767L, 281474976710758L, 281474976710731L,
            281474976710772L };

    protected final float[][] startPoss = { { 22.60652f, 10.197294f, -45.918674f },
            { 22.331268f, 10.197294f, -1.0401875f }, { 18.694363f, 15.803535f, -73.090416f },
            { 0.7453353f, 10.197294f, -5.94005f }, { -20.651257f, 5.904126f, -13.712508f } };

    protected final float[][] endPoss = { { 6.4576626f, 10.197294f, -18.33406f },
            { -5.8023443f, 0.19729415f, 3.008419f }, { 38.423977f, 10.197294f, -0.116066754f },
            { 0.8635526f, 10.197294f, -10.31032f }, { 18.784092f, 10.197294f, 3.0543678f } };

    protected MeshData nmd;
    protected NavMeshQuery query;
    protected NavMesh navmesh;
    protected Crowd crowd;

    @Before
    public void setUp() {
        nmd = new RecastTestMeshBuilder().getMeshData();
        navmesh = new NavMesh(nmd, 6, 0);
        query = new NavMeshQuery(navmesh);
        crowd = new Crowd(50, 0.6f, navmesh);
        ObstacleAvoidanceParams params = new ObstacleAvoidanceParams();
        params.velBias = 0.5f;
        params.adaptiveDivs = 5;
        params.adaptiveRings = 2;
        params.adaptiveDepth = 1;
        crowd.setObstacleAvoidanceParams(0, params);
        params = new ObstacleAvoidanceParams();
        params.velBias = 0.5f;
        params.adaptiveDivs = 5;
        params.adaptiveRings = 2;
        params.adaptiveDepth = 2;
        crowd.setObstacleAvoidanceParams(1, params);
        params = new ObstacleAvoidanceParams();
        params.velBias = 0.5f;
        params.adaptiveDivs = 7;
        params.adaptiveRings = 2;
        params.adaptiveDepth = 3;
        crowd.setObstacleAvoidanceParams(2, params);
        params = new ObstacleAvoidanceParams();
        params.velBias = 0.5f;
        params.adaptiveDivs = 7;
        params.adaptiveRings = 3;
        params.adaptiveDepth = 3;
        crowd.setObstacleAvoidanceParams(3, params);
    }

    protected CrowdAgentParams getAgentParams(int updateFlags, int obstacleAvoidanceType) {
        CrowdAgentParams ap = new CrowdAgentParams();
        ap.radius = 0.6f;
        ap.height = 2f;
        ap.maxAcceleration = 8.0f;
        ap.maxSpeed = 3.5f;
        ap.collisionQueryRange = ap.radius * 12f;
        ap.pathOptimizationRange = ap.radius * 30f;
        ap.updateFlags = updateFlags;
        ap.obstacleAvoidanceType = obstacleAvoidanceType;
        ap.separationWeight = 2f;
        return ap;
    }

    protected void addAgentGrid(int size, float distance, int updateFlags, int obstacleAvoidanceType,
            float[] startPos) {
        CrowdAgentParams ap = getAgentParams(updateFlags, obstacleAvoidanceType);
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                float[] pos = new float[3];
                pos[0] = startPos[0] + i * distance;
                pos[1] = startPos[1];
                pos[2] = startPos[2] + j * distance;
                crowd.addAgent(pos, ap);
            }
        }
    }

    protected void setMoveTarget(float[] pos, boolean adjust) {
        float[] ext = crowd.getQueryExtents();
        QueryFilter filter = crowd.getFilter(0);
        if (adjust) {
            for (int i = 0; i < crowd.getAgentCount(); i++) {
                CrowdAgent ag = crowd.getAgent(i);
                if (!ag.isActive()) {
                    continue;
                }
                float[] vel = calcVel(ag.npos, pos, ag.params.maxSpeed);
                crowd.requestMoveVelocity(i, vel);
            }
        } else {
            Result<FindNearestPolyResult> nearest = query.findNearestPoly(pos, ext, filter);
            for (int i = 0; i < crowd.getAgentCount(); i++) {
                CrowdAgent ag = crowd.getAgent(i);
                if (!ag.isActive()) {
                    continue;
                }
                crowd.requestMoveTarget(i, nearest.result.getNearestRef(), nearest.result.getNearestPos());
            }
        }
    }

    protected float[] calcVel(float[] pos, float[] tgt, float speed) {
        float[] vel = vSub(tgt, pos);
        vel[1] = 0.0f;
        vNormalize(vel);
        vel = vScale(vel, speed);
        return vel;
    }

    protected void dumpActiveAgents(int i) {
        System.out.println(crowd.getActiveAgents().size());
        for (CrowdAgent ag : crowd.getActiveAgents()) {
            System.out.println(ag.state + ", " + ag.targetState);
            System.out.println(ag.npos[0] + ", " + ag.npos[1] + ", " + ag.npos[2]);
            System.out.println(ag.nvel[0] + ", " + ag.nvel[1] + ", " + ag.nvel[2]);
        }
    }

}
