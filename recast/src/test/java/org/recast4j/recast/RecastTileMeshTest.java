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
package org.recast4j.recast;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.geom.InputGeomProvider;

public class RecastTileMeshTest {

    private final float m_cellSize = 0.3f;
    private final float m_cellHeight = 0.2f;
    private final float m_agentHeight = 2.0f;
    private final float m_agentRadius = 0.6f;
    private final float m_agentMaxClimb = 0.9f;
    private final float m_agentMaxSlope = 45.0f;
    private final int m_regionMinSize = 8;
    private final int m_regionMergeSize = 20;
    private final float m_edgeMaxLen = 12.0f;
    private final float m_edgeMaxError = 1.3f;
    private final int m_vertsPerPoly = 6;
    private final float m_detailSampleDist = 6.0f;
    private final float m_detailSampleMaxError = 1.0f;
    private final PartitionType m_partitionType = PartitionType.WATERSHED;
    private final int m_tileSize = 32;

    @Test
    public void testDungeon() {
        testBuild("dungeon.obj");
    }

    public void testBuild(String filename) {
        ObjImporter importer = new ObjImporter();
        InputGeomProvider geom = importer.load(getClass().getResourceAsStream(filename));
        RecastBuilder builder = new RecastBuilder();
        RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, m_tileSize,
                SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
        RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 7, 8,
                true);
        RecastBuilderResult rcResult = builder.build(geom, bcfg);
        assertEquals(1, rcResult.getMesh().npolys);
        assertEquals(5, rcResult.getMesh().nverts);
        bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 6, 9, true);
        rcResult = builder.build(geom, bcfg);
        assertEquals(2, rcResult.getMesh().npolys);
        assertEquals(7, rcResult.getMesh().nverts);
        bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 2, 9, true);
        rcResult = builder.build(geom, bcfg);
        assertEquals(2, rcResult.getMesh().npolys);
        assertEquals(9, rcResult.getMesh().nverts);
        bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 4, 3, true);
        rcResult = builder.build(geom, bcfg);
        assertEquals(3, rcResult.getMesh().npolys);
        assertEquals(6, rcResult.getMesh().nverts);
        bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 2, 8, true);
        rcResult = builder.build(geom, bcfg);
        assertEquals(5, rcResult.getMesh().npolys);
        assertEquals(17, rcResult.getMesh().nverts);
        bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 0, 8, true);
        rcResult = builder.build(geom, bcfg);
        assertEquals(6, rcResult.getMesh().npolys);
        assertEquals(15, rcResult.getMesh().nverts);
    }

    @Test
    public void testPerformance() {
        ObjImporter importer = new ObjImporter();
        InputGeomProvider geom = importer.load(getClass().getResourceAsStream("dungeon.obj"));
        RecastBuilder builder = new RecastBuilder();
        RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, m_tileSize,
                SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
        for (int i = 0; i < 10; i++) {
            build(geom, builder, cfg, 1, true);
            build(geom, builder, cfg, 4, true);
        }
        long t1 = System.nanoTime();
        for (int i = 0; i < 20; i++) {
            build(geom, builder, cfg, 1, false);
        }
        long t2 = System.nanoTime();
        for (int i = 0; i < 20; i++) {
            build(geom, builder, cfg, 4, false);
        }
        long t3 = System.nanoTime();
        System.out.println(" Time ST : " + (t2 - t1) / 1000000);
        System.out.println(" Time MT : " + (t3 - t2) / 1000000);
    }

    private void build(InputGeomProvider geom, RecastBuilder builder, RecastConfig cfg, int threads, boolean validate) {
        RecastBuilderResult[][] tiles = builder.buildTiles(geom, cfg, threads);
        if (validate) {
            RecastBuilderResult rcResult = tiles[7][8];
            assertEquals(1, rcResult.getMesh().npolys);
            assertEquals(5, rcResult.getMesh().nverts);
            rcResult = tiles[6][9];
            assertEquals(2, rcResult.getMesh().npolys);
            assertEquals(7, rcResult.getMesh().nverts);
            rcResult = tiles[2][9];
            assertEquals(2, rcResult.getMesh().npolys);
            assertEquals(9, rcResult.getMesh().nverts);
            rcResult = tiles[4][3];
            assertEquals(3, rcResult.getMesh().npolys);
            assertEquals(6, rcResult.getMesh().nverts);
            rcResult = tiles[2][8];
            assertEquals(5, rcResult.getMesh().npolys);
            assertEquals(17, rcResult.getMesh().nverts);
            rcResult = tiles[0][8];
            assertEquals(6, rcResult.getMesh().npolys);
            assertEquals(15, rcResult.getMesh().nverts);
        }
    }

}
