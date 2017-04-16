package org.recast4j.recast;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.recast4j.recast.RecastConstants.PartitionType;

public class RecastLayersTest {
	private float m_cellSize = 0.3f;
	private float m_cellHeight = 0.2f;
	private float m_agentHeight = 2.0f;
	private float m_agentRadius = 0.6f;
	private float m_agentMaxClimb = 0.9f;
	private float m_agentMaxSlope = 45.0f;
	private int m_regionMinSize = 8;
	private int m_regionMergeSize = 20;
	private float m_edgeMaxLen = 12.0f;
	private float m_edgeMaxError = 1.3f;
	private int m_vertsPerPoly = 6;
	private float m_detailSampleDist = 6.0f;
	private float m_detailSampleMaxError = 1.0f;
	private PartitionType m_partitionType = PartitionType.WATERSHED;
	private int m_tileSize = 48;

	@Test
	public void testDungeon() {
		HeightfieldLayerSet lset = build("dungeon.obj",3,2);
		assertEquals(1, lset.layers.length);
		assertEquals(48, lset.layers[0].width);
		assertEquals(51, lset.layers[0].hmin);
		assertEquals(67, lset.layers[0].hmax);
		assertEquals(17, lset.layers[0].heights[7]);
		assertEquals(15, lset.layers[0].heights[107]);
		assertEquals(13, lset.layers[0].heights[257]);
		assertEquals(255, lset.layers[0].heights[1814]);
		assertEquals(135, lset.layers[0].cons[12]);
		assertEquals(15, lset.layers[0].cons[109]);
		assertEquals(15, lset.layers[0].cons[530]);
		assertEquals(0, lset.layers[0].cons[1600]);
	}

	@Test
	public void test() {
		HeightfieldLayerSet lset = build("nav_test.obj",3,2);
		assertEquals(3, lset.layers.length);
		assertEquals(48, lset.layers[0].width);
		assertEquals(13, lset.layers[0].hmin);
		assertEquals(30, lset.layers[0].hmax);
		assertEquals(0, lset.layers[0].heights[7]);
		assertEquals(255, lset.layers[0].heights[107]);
		assertEquals(0, lset.layers[0].heights[257]);
		assertEquals(255, lset.layers[0].heights[1814]);
		assertEquals(133, lset.layers[0].cons[12]);
		assertEquals(0, lset.layers[0].cons[109]);
		assertEquals(0, lset.layers[0].cons[530]);
		assertEquals(15, lset.layers[0].cons[1600]);

		assertEquals(48, lset.layers[1].width);
		assertEquals(13, lset.layers[1].hmin);
		assertEquals(13, lset.layers[1].hmax);
		assertEquals(255, lset.layers[1].heights[7]);
		assertEquals(255, lset.layers[1].heights[107]);
		assertEquals(255, lset.layers[1].heights[257]);
		assertEquals(255, lset.layers[1].heights[1814]);
		assertEquals(0, lset.layers[1].cons[12]);
		assertEquals(0, lset.layers[1].cons[109]);
		assertEquals(0, lset.layers[1].cons[530]);
		assertEquals(0, lset.layers[1].cons[1600]);

		assertEquals(48, lset.layers[2].width);
		assertEquals(76, lset.layers[2].hmin);
		assertEquals(76, lset.layers[2].hmax);
		assertEquals(255, lset.layers[2].heights[7]);
		assertEquals(255, lset.layers[2].heights[107]);
		assertEquals(255, lset.layers[2].heights[257]);
		assertEquals(255, lset.layers[2].heights[1814]);
		assertEquals(0, lset.layers[2].cons[12]);
		assertEquals(0, lset.layers[2].cons[109]);
		assertEquals(0, lset.layers[2].cons[530]);
		assertEquals(0, lset.layers[2].cons[1600]);
	}

	@Test
	public void test2() {
		HeightfieldLayerSet lset = build("nav_test.obj",2,4);
		assertEquals(2, lset.layers.length);
		assertEquals(48, lset.layers[0].width);
		assertEquals(13, lset.layers[0].hmin);
		assertEquals(13, lset.layers[0].hmax);
		assertEquals(0, lset.layers[0].heights[7]);
		assertEquals(0, lset.layers[0].heights[107]);
		assertEquals(0, lset.layers[0].heights[257]);
		assertEquals(0, lset.layers[0].heights[1814]);
		assertEquals(135, lset.layers[0].cons[12]);
		assertEquals(15, lset.layers[0].cons[109]);
		assertEquals(0, lset.layers[0].cons[530]);
		assertEquals(15, lset.layers[0].cons[1600]);

		assertEquals(48, lset.layers[1].width);
		assertEquals(68, lset.layers[1].hmin);
		assertEquals(101, lset.layers[1].hmax);
		assertEquals(33, lset.layers[1].heights[7]);
		assertEquals(255, lset.layers[1].heights[107]);
		assertEquals(255, lset.layers[1].heights[257]);
		assertEquals(3, lset.layers[1].heights[1814]);
		assertEquals(0, lset.layers[1].cons[12]);
		assertEquals(0, lset.layers[1].cons[109]);
		assertEquals(15, lset.layers[1].cons[530]);
		assertEquals(0, lset.layers[1].cons[1600]);

	}

	private HeightfieldLayerSet build(String filename, int x, int y) {
		ObjImporter importer = new ObjImporter();
		InputGeom geom = importer.load(getClass().getResourceAsStream(filename));
		RecastBuilder builder = new RecastBuilder();
		RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
				m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
				m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, m_tileSize, SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
		RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), x, y, true);
		HeightfieldLayerSet lset = builder.buildLayers(geom, bcfg);
		return lset;
	}
}
