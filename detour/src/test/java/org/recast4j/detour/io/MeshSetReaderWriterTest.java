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
package org.recast4j.detour.io;

import static org.junit.Assert.assertEquals;
import static org.recast4j.detour.DetourCommon.vCopy;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.SampleAreaModifications;
import org.recast4j.detour.TestDetourBuilder;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.Recast;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.geom.InputGeomProvider;

public class MeshSetReaderWriterTest {

    private final MeshSetWriter writer = new MeshSetWriter();
    private final MeshSetReader reader = new MeshSetReader();
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
    private final static int m_maxTiles = 128;
    private final static int m_maxPolysPerTile = 0x8000;

    @Test
    public void test() throws IOException {

        InputGeomProvider geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));

        NavMeshSetHeader header = new NavMeshSetHeader();
        header.magic = NavMeshSetHeader.NAVMESHSET_MAGIC;
        header.version = NavMeshSetHeader.NAVMESHSET_VERSION;
        vCopy(header.params.orig, geom.getMeshBoundsMin());
        header.params.tileWidth = m_tileSize * m_cellSize;
        header.params.tileHeight = m_tileSize * m_cellSize;
        header.params.maxTiles = m_maxTiles;
        header.params.maxPolys = m_maxPolysPerTile;
        header.numTiles = 0;
        NavMesh mesh = new NavMesh(header.params, 6);

        float[] bmin = geom.getMeshBoundsMin();
        float[] bmax = geom.getMeshBoundsMax();
        int[] twh = Recast.calcTileCount(bmin, bmax, m_cellSize, m_tileSize);
        int tw = twh[0];
        int th = twh[1];
        for (int y = 0; y < th; ++y) {
            for (int x = 0; x < tw; ++x) {
                RecastConfig cfg = new RecastConfig(PartitionType.WATERSHED, m_cellSize, m_cellHeight, m_agentHeight,
                        m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize,
                        m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError,
                        m_tileSize, SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
                RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, bmin, bmax, x, y, true);
                TestDetourBuilder db = new TestDetourBuilder();
                MeshData data = db.build(geom, bcfg, m_agentHeight, m_agentRadius, m_agentMaxClimb, x, y, true);
                if (data != null) {
                    mesh.removeTile(mesh.getTileRefAt(x, y, 0));
                    mesh.addTile(data, 0, 0);
                }
            }
        }
        ByteArrayOutputStream os = new ByteArrayOutputStream();
        writer.write(os, mesh, ByteOrder.LITTLE_ENDIAN, true);
        ByteArrayInputStream is = new ByteArrayInputStream(os.toByteArray());
        mesh = reader.read(is, 6);
        assertEquals(128, mesh.getMaxTiles());
        assertEquals(0x8000, mesh.getParams().maxPolys);
        assertEquals(9.6, mesh.getParams().tileWidth, 0.001);
        List<MeshTile> tiles = mesh.getTilesAt(6, 9);
        assertEquals(1, tiles.size());
        assertEquals(2, tiles.get(0).data.polys.length);
        assertEquals(7 * 3, tiles.get(0).data.verts.length);
        tiles = mesh.getTilesAt(2, 9);
        assertEquals(1, tiles.size());
        assertEquals(2, tiles.get(0).data.polys.length);
        assertEquals(9 * 3, tiles.get(0).data.verts.length);
        tiles = mesh.getTilesAt(4, 3);
        assertEquals(1, tiles.size());
        assertEquals(3, tiles.get(0).data.polys.length);
        assertEquals(6 * 3, tiles.get(0).data.verts.length);
        tiles = mesh.getTilesAt(2, 8);
        assertEquals(1, tiles.size());
        assertEquals(5, tiles.get(0).data.polys.length);
        assertEquals(17 * 3, tiles.get(0).data.verts.length);

    }
}
