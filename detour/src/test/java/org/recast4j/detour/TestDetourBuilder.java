/*
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.detour;

import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.geom.InputGeomProvider;

public class TestDetourBuilder extends DetourBuilder {

    public MeshData build(InputGeomProvider geom, RecastBuilderConfig rcConfig, float agentHeight, float agentRadius,
            float agentMaxClimb, int x, int y, boolean applyRecastDemoFlags) {
        RecastBuilder rcBuilder = new RecastBuilder();
        RecastBuilderResult rcResult = rcBuilder.build(geom, rcConfig);
        PolyMesh pmesh = rcResult.getMesh();

        if (applyRecastDemoFlags) {
            // Update poly flags from areas.
            for (int i = 0; i < pmesh.npolys; ++i) {
                if (pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GROUND
                        || pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GRASS
                        || pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD) {
                    pmesh.flags[i] = SampleAreaModifications.SAMPLE_POLYFLAGS_WALK;
                } else if (pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER) {
                    pmesh.flags[i] = SampleAreaModifications.SAMPLE_POLYFLAGS_SWIM;
                } else if (pmesh.areas[i] == SampleAreaModifications.SAMPLE_POLYAREA_TYPE_DOOR) {
                    pmesh.flags[i] = SampleAreaModifications.SAMPLE_POLYFLAGS_WALK
                            | SampleAreaModifications.SAMPLE_POLYFLAGS_DOOR;
                }
                if (pmesh.areas[i] > 0) {
                    pmesh.areas[i]--;
                }
            }
        }
        PolyMeshDetail dmesh = rcResult.getMeshDetail();
        NavMeshDataCreateParams params = getNavMeshCreateParams(rcConfig.cfg, pmesh, dmesh, agentHeight, agentRadius,
                agentMaxClimb);
        return build(params, x, y);
    }

    public NavMeshDataCreateParams getNavMeshCreateParams(RecastConfig rcConfig, PolyMesh pmesh, PolyMeshDetail dmesh,
            float agentHeight, float agentRadius, float agentMaxClimb) {
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        params.verts = pmesh.verts;
        params.vertCount = pmesh.nverts;
        params.polys = pmesh.polys;
        params.polyAreas = pmesh.areas;
        params.polyFlags = pmesh.flags;
        params.polyCount = pmesh.npolys;
        params.nvp = pmesh.nvp;
        if (dmesh != null) {
            params.detailMeshes = dmesh.meshes;
            params.detailVerts = dmesh.verts;
            params.detailVertsCount = dmesh.nverts;
            params.detailTris = dmesh.tris;
            params.detailTriCount = dmesh.ntris;
        }
        params.walkableHeight = agentHeight;
        params.walkableRadius = agentRadius;
        params.walkableClimb = agentMaxClimb;
        params.bmin = pmesh.bmin;
        params.bmax = pmesh.bmax;
        params.cs = rcConfig.cs;
        params.ch = rcConfig.ch;
        params.buildBvTree = true;
        /*
         * params.offMeshConVerts = m_geom->getOffMeshConnectionVerts(); params.offMeshConRad =
         * m_geom->getOffMeshConnectionRads(); params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
         * params.offMeshConAreas = m_geom->getOffMeshConnectionAreas(); params.offMeshConFlags =
         * m_geom->getOffMeshConnectionFlags(); params.offMeshConUserID = m_geom->getOffMeshConnectionId();
         * params.offMeshConCount = m_geom->getOffMeshConnectionCount();
         */
        return params;

    }
}
