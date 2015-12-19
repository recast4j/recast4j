package org.recast4j.detour.tilecache;

import java.nio.ByteOrder;
import java.util.List;

import org.junit.Test;
import org.recast4j.detour.tilecache.io.compress.FastLzTileCacheCompressor;
import org.recast4j.detour.tilecache.io.compress.LZ4TileCacheCompressor;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;

public class TileCacheCompressionTest extends AbstractTileCacheTest {

	@Test
	public void testFastLz() {
		test(new FastLzTileCacheCompressor(), "dungeon.obj");
	}

	@Test
	public void testLZ4() {
		test(new LZ4TileCacheCompressor(), "dungeon.obj");
	}

	private void test(TileCacheCompressor compressor, String filename) {
		InputGeom geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream(filename));
		RecastTileLayersBuilder layerBuilder = new RecastTileLayersBuilder(geom, getTileCache(geom, compressor));
		List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, true);
		int cacheLayerCount = 0;
		int cacheCompressedSize = 0;
		int cacheRawSize = 0;
		for (byte[] data : layers) {
			cacheLayerCount++;
			cacheCompressedSize += data.length;
			cacheRawSize += 4 * 48 * 48 + 56;
		}
		System.out.println(compressor.getClass().getSimpleName() + " Layers: " + cacheLayerCount + " Raw Size: " + cacheRawSize + " Compressed: "
				+ cacheCompressedSize);
	}

}
