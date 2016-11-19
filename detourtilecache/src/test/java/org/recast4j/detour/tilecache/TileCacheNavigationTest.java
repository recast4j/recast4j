package org.recast4j.detour.tilecache;

import java.io.IOException;
import java.nio.ByteOrder;
import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.recast4j.detour.FindPathResult;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Status;
import org.recast4j.recast.InputGeom;
import org.recast4j.recast.ObjImporter;
import org.recast4j.recast.RecastBuilder;

public class TileCacheNavigationTest extends AbstractTileCacheTest {

	protected final long[] startRefs = { 281475006070787L };
	protected final long[] endRefs = { 281474986147841L };
	protected final float[][] startPoss = { { 39.447338f, 9.998177f, -0.784811f } };
	protected final float[][] endPoss = { { 19.292645f, 11.611748f, -57.750366f } };
	private final Status[] statuses = { Status.SUCCSESS, Status.PARTIAL_RESULT, Status.SUCCSESS, Status.SUCCSESS, Status.SUCCSESS };
	private final long[][] results = { { 281475006070787L, 281475006070785L, 281475005022208L, 281475005022209L, 281475003973633L,
			281475003973634L, 281475003973632L, 281474996633604L, 281474996633605L, 281474996633603L, 281474995585027L,
			281474995585029L, 281474995585026L, 281474995585028L, 281474995585024L, 281474991390721L, 281474991390722L,
			281474991390725L, 281474991390720L, 281474987196418L, 281474987196417L, 281474988244995L, 281474988245001L,
			281474988244997L, 281474988244998L, 281474988245002L, 281474988245000L, 281474988244999L, 281474988244994L,
			281474985099264L, 281474985099266L, 281474986147841L
			} };

	protected NavMesh navmesh;
	protected NavMeshQuery query;

	@Before
	public void setUp() throws IOException {

		boolean cCompatibility = true;
		InputGeom geom = new ObjImporter().load(RecastBuilder.class.getResourceAsStream("dungeon.obj"));
		TestTileLayerBuilder layerBuilder = new TestTileLayerBuilder(geom);
		List<byte[]> layers = layerBuilder.build(ByteOrder.LITTLE_ENDIAN, cCompatibility, 1);
		TileCache tc = getTileCache(geom, ByteOrder.LITTLE_ENDIAN, cCompatibility);
		for (byte[] data : layers) {
			tc.addTile(data, 0);
		}
		for (int y = 0; y < layerBuilder.getTh(); ++y) {
			for (int x = 0; x < layerBuilder.getTw(); ++x) {
				for (Long ref : tc.getTilesAt(x, y)) {
					tc.buildNavMeshTile(ref);
				}
			}
		}
		navmesh = tc.getNavMesh();
		query = new NavMeshQuery(navmesh);

	}

	@Test
	public void testFindPath() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			long startRef = startRefs[i];
			long endRef = endRefs[i];
			float[] startPos = startPoss[i];
			float[] endPos = endPoss[i];
			FindPathResult path = query.findPath(startRef, endRef, startPos, endPos, filter);
			Assert.assertEquals(statuses[i], path.getStatus());
			Assert.assertEquals(results[i].length, path.getRefs().size());
			for (int j = 0; j < results[i].length; j++) {
				Assert.assertEquals(results[i][j], path.getRefs().get(j).longValue());
			}
		}
	}

}
