package org.recast4j.detour.tilecache;

import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.tilecache.io.compress.FastLzTileCacheCompressor;
import org.recast4j.detour.tilecache.io.compress.LZ4TileCacheCompressor;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;

public class TileCacheTest extends AbstractTileCacheTest {

	@Test
	public void testFastLz() throws IOException {
		test(new FastLzTileCacheCompressor());
	}

	@Test
	public void testLZ4() throws IOException {
		test(new LZ4TileCacheCompressor());
	}

	private void test(TileCacheCompressor compressor) throws IOException {
		InputGeom geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
		TileCache tc = getTileCache(geom, compressor);
		RecastTileLayersBuilder layerBuilder = new RecastTileLayersBuilder(geom, tc);
		List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, true);
		for (byte[] data : layers) {
			tc.addTile(data, 0);
		}
	}

}
