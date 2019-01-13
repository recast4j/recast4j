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

import static org.recast4j.recast.RecastVectors.copy;

import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.PolyMesh;
import org.recast4j.recast.PolyMeshDetail;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.geom.InputGeomProvider;

public class TestTiledNavMeshBuilder {

    private final NavMesh navMesh;
    private final static float m_cellSize = 0.3f;
    private final static float m_cellHeight = 0.2f;
    private final static float m_agentHeight = 2.0f;
    private final static float m_agentRadius = 0.6f;
    private final static float m_agentMaxClimb = 0.9f;
    private final static float m_agentMaxSlope = 45.0f;
    private final static int m_regionMinSize = 8;
    private final static int m_regionMergeSize = 20;
    private final static float m_edgeMaxLen = 12.0f;
    private final static float m_edgeMaxError = 1.3f;
    private final static int m_vertsPerPoly = 6;
    private final static float m_detailSampleDist = 6.0f;
    private final static float m_detailSampleMaxError = 1.0f;
    private final static int m_tileSize = 32;

    public TestTiledNavMeshBuilder() {
        this(new ObjImporter().load(ObjImporter.class.getResourceAsStream("dungeon.obj")), PartitionType.WATERSHED,
                m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope,
                m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist,
                m_detailSampleMaxError, m_tileSize);
    }

    public TestTiledNavMeshBuilder(InputGeomProvider m_geom, PartitionType m_partitionType, float m_cellSize,
            float m_cellHeight, float m_agentHeight, float m_agentRadius, float m_agentMaxClimb, float m_agentMaxSlope,
            int m_regionMinSize, int m_regionMergeSize, float m_edgeMaxLen, float m_edgeMaxError, int m_vertsPerPoly,
            float m_detailSampleDist, float m_detailSampleMaxError, int m_tileSize) {

        // Create empty nav mesh
        NavMeshParams navMeshParams = new NavMeshParams();
        copy(navMeshParams.orig, m_geom.getMeshBoundsMin());
        navMeshParams.tileWidth = m_tileSize * m_cellSize;
        navMeshParams.tileHeight = m_tileSize * m_cellSize;
        navMeshParams.maxTiles = 128;
        navMeshParams.maxPolys = 32768;
        navMesh = new NavMesh(navMeshParams, 6);

        // Build all tiles
        RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, m_tileSize,
                SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
        RecastBuilder rcBuilder = new RecastBuilder();
        RecastBuilderResult[][] rcResult = rcBuilder.buildTiles(m_geom, cfg, 1);

        // Add tiles to nav mesh
        int tw = rcResult.length;
        int th = rcResult[0].length;
        for (int y = 0; y < th; y++) {
            for (int x = 0; x < tw; x++) {
                PolyMesh pmesh = rcResult[x][y].getMesh();
                if (pmesh.npolys == 0) {
                    continue;
                }
                for (int i = 0; i < pmesh.npolys; ++i) {
                    pmesh.flags[i] = 1;
                }
                NavMeshDataCreateParams params = new NavMeshDataCreateParams();
                params.verts = pmesh.verts;
                params.vertCount = pmesh.nverts;
                params.polys = pmesh.polys;
                params.polyAreas = pmesh.areas;
                params.polyFlags = pmesh.flags;
                params.polyCount = pmesh.npolys;
                params.nvp = pmesh.nvp;
                PolyMeshDetail dmesh = rcResult[x][y].getMeshDetail();
                params.detailMeshes = dmesh.meshes;
                params.detailVerts = dmesh.verts;
                params.detailVertsCount = dmesh.nverts;
                params.detailTris = dmesh.tris;
                params.detailTriCount = dmesh.ntris;
                params.walkableHeight = m_agentHeight;
                params.walkableRadius = m_agentRadius;
                params.walkableClimb = m_agentMaxClimb;
                params.bmin = pmesh.bmin;
                params.bmax = pmesh.bmax;
                params.cs = m_cellSize;
                params.ch = m_cellHeight;
                params.tileX = x;
                params.tileY = y;
                params.buildBvTree = true;
                navMesh.addTile(NavMeshBuilder.createNavMeshData(params), 0, 0);
            }
        }
    }

    public NavMesh getNavMesh() {
        return navMesh;
    }

}
