package org.recast4j.recast;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
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
		RecastBuilder builder = new RecastBuilder();
		RecastConfig cfg = new RecastConfig(m_partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius, m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, 
				m_edgeMaxLen, m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, 32);
		RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), 7, 8, true);
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

}
