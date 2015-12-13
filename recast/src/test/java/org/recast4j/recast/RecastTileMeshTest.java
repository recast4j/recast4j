package org.recast4j.recast;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.recast4j.recast.RecastConstants.PartitionType;

public class RecastTileMeshTest {

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

	@Test
	public void testDungeon() {
		testBuild("dungeon.obj");
	}

	public void testBuild(String filename) {
		ObjImporter importer = new ObjImporter();
		InputGeom geom = importer.load(getClass().getResourceAsStream(filename));
		RecastBuilder builder = new RecastBuilder(geom);
		RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 32, 7, 8);
		builder.build(cfg);
		assertEquals(1, builder.getMesh().npolys);
		assertEquals(5, builder.getMesh().nverts);
		cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 32, 6, 9);
		builder.build(cfg);
		assertEquals(2, builder.getMesh().npolys);
		assertEquals(7, builder.getMesh().nverts);
		cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 32, 2, 9);
		builder.build(cfg);
		assertEquals(2, builder.getMesh().npolys);
		assertEquals(9, builder.getMesh().nverts);
		cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 32, 4, 3);
		builder.build(cfg);
		assertEquals(3, builder.getMesh().npolys);
		assertEquals(6, builder.getMesh().nverts);
		cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 32, 2, 8);
		builder.build(cfg);
		assertEquals(5, builder.getMesh().npolys);
		assertEquals(17, builder.getMesh().nverts);
		cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 32, 0, 8);
		builder.build(cfg);
		assertEquals(6, builder.getMesh().npolys);
		assertEquals(15, builder.getMesh().nverts);
	}

}
