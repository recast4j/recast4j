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
package org.recast4j.detour.tilecache;

import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.recast.RecastVectors.copy;

import java.nio.ByteOrder;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.NavMeshParams;
import org.recast4j.detour.tilecache.io.compress.TileCacheCompressorFactory;
import org.recast4j.recast.Recast;
import org.recast4j.recast.geom.InputGeomProvider;

public class AbstractTileCacheTest {

    private static final int EXPECTED_LAYERS_PER_TILE = 4;
    private final float m_cellSize = 0.3f;
    private final float m_cellHeight = 0.2f;
    private final float m_agentHeight = 2.0f;
    private final float m_agentRadius = 0.6f;
    private final float m_agentMaxClimb = 0.9f;
    private final float m_edgeMaxError = 1.3f;
    private final int m_tileSize = 48;

    protected static class TestTileCacheMeshProcess implements TileCacheMeshProcess {
        @Override
        public void process(NavMeshDataCreateParams params) {
            for (int i = 0; i < params.polyCount; ++i) {
                params.polyFlags[i] = 1;
            }
        }

    }

    public TileCache getTileCache(InputGeomProvider geom, ByteOrder order, boolean cCompatibility) {
        TileCacheParams params = new TileCacheParams();
        int[] twh = Recast.calcTileCount(geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), m_cellSize, m_tileSize);
        params.ch = m_cellHeight;
        params.cs = m_cellSize;
        vCopy(params.orig, geom.getMeshBoundsMin());
        params.height = m_tileSize;
        params.width = m_tileSize;
        params.walkableHeight = m_agentHeight;
        params.walkableRadius = m_agentRadius;
        params.walkableClimb = m_agentMaxClimb;
        params.maxSimplificationError = m_edgeMaxError;
        params.maxTiles = twh[0] * twh[1] * EXPECTED_LAYERS_PER_TILE;
        params.maxObstacles = 128;
        NavMeshParams navMeshParams = new NavMeshParams();
        copy(navMeshParams.orig, geom.getMeshBoundsMin());
        navMeshParams.tileWidth = m_tileSize * m_cellSize;
        navMeshParams.tileHeight = m_tileSize * m_cellSize;
        navMeshParams.maxTiles = 256;
        navMeshParams.maxPolys = 16384;
        NavMesh navMesh = new NavMesh(navMeshParams, 6);
        TileCache tc = new TileCache(params, new TileCacheStorageParams(order, cCompatibility), navMesh,
                TileCacheCompressorFactory.get(cCompatibility), new TestTileCacheMeshProcess());
        return tc;
    }

}
