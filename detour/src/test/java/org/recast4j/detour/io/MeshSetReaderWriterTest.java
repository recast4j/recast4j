package org.recast4j.detour.io;

import static org.recast4j.detour.DetourCommon.vCopy;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteOrder;

import org.junit.Test;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.TestDetourBuilder;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.Recast;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;

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

		InputGeom geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));

		NavMeshSetHeader header = new NavMeshSetHeader();
		header.magic = NavMeshSetHeader.NAVMESHSET_MAGIC;
		header.version = NavMeshSetHeader.NAVMESHSET_VERSION;
		vCopy(header.params.orig, geom.getMeshBoundsMin());
		header.params.tileWidth = m_tileSize*m_cellSize;
		header.params.tileHeight = m_tileSize*m_cellSize;
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
				RecastConfig cfg = new RecastConfig(PartitionType.WATERSHED, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
						m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen,
						m_edgeMaxError, m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError,
						m_tileSize);
				RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, bmin, bmax, x, y, true);
				TestDetourBuilder db = new TestDetourBuilder();
				MeshData data = db.build(geom, bcfg, m_agentHeight, m_agentRadius, m_agentMaxClimb, x, y);
				if (data != null) {
					mesh.removeTile(mesh.getTileRefAt(x, y, 0));
					mesh.addTile(data, 0, 0);
				}
			}
		}
		ByteArrayOutputStream os = new ByteArrayOutputStream();
		writer.write(os, mesh, ByteOrder.LITTLE_ENDIAN, true);
		ByteArrayInputStream is = new ByteArrayInputStream(os.toByteArray());
		NavMesh nm = reader.read(is, 6, ByteOrder.LITTLE_ENDIAN, true);
	}
}
