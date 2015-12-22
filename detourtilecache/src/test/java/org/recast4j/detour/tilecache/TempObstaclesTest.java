package org.recast4j.detour.tilecache;

import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.tilecache.io.compress.LZ4TileCacheCompressor;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;

public class TempObstaclesTest extends AbstractTileCacheTest {

	@Test
	public void testDungeon() throws IOException {
		TileCacheCompressor compressor = new LZ4TileCacheCompressor();
		boolean cCompatibility = true;
		InputGeom geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
		RecastTileLayersBuilder layerBuilder = new RecastTileLayersBuilder(geom);
		List<byte[]> layers = layerBuilder.build(compressor, ByteOrder.LITTLE_ENDIAN, cCompatibility);
		TileCache tc = getTileCache(geom, compressor, ByteOrder.LITTLE_ENDIAN, cCompatibility);
		for (byte[] data : layers) {
			long ref = tc.addTile(data, 0);
			tc.buildNavMeshTile(ref);
		}
	}
}
