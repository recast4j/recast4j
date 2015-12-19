package org.recast4j.detour.tilecache;

import org.recast4j.recast.InputGeom;
import org.recast4j.recast.Recast;

public class AbstractTileCacheTest {

	private static final int EXPECTED_LAYERS_PER_TILE = 4;
	private final float m_cellSize = 0.3f;
	private final float m_cellHeight = 0.2f;
	private final float m_agentHeight = 2.0f;
	private final float m_agentRadius = 0.6f;
	private final float m_agentMaxClimb = 0.9f;
	private final float m_edgeMaxError = 1.3f;
	private final int m_tileSize = 48;

	public TileCache getTileCache(InputGeom geom, TileCacheCompressor compressor) {
		TileCache tc = new TileCache();
		TileCacheParams params = new TileCacheParams();
		int[] twh = Recast.calcTileCount(geom.getMeshBoundsMin(), geom.getMeshBoundsMax(), m_cellSize, m_tileSize);
		params.ch = m_cellHeight;
		params.cs = m_cellSize;
		params.height = m_tileSize;
		params.width = m_tileSize;
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.maxSimplificationError = m_edgeMaxError;
		params.maxTiles = twh[0] * twh[1] * EXPECTED_LAYERS_PER_TILE;
		params.maxObstacles = 128;
		tc.init(params, compressor, null);
		return tc;
	}

}
