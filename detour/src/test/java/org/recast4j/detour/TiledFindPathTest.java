package org.recast4j.detour;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class TiledFindPathTest {

	private final Status[] statuses = { Status.SUCCSESS };
	private final long[][] results = { { 281475015507969L, 281475014459393L, 281475014459392L, 281475013410816L,
			281475012362240L, 281475012362241L, 281475012362242L, 281475003973634L, 281475003973635L, 281475003973633L,
			281475002925059L, 281475002925057L, 281475002925056L, 281474998730753L, 281474998730754L, 281474994536450L,
			281474994536451L, 281474994536452L, 281474994536448L, 281474990342146L, 281474990342145L, 281474991390723L,
			281474991390724L, 281474991390725L, 281474992439298L, 281474992439300L, 281474992439299L, 281474992439297L,
			281474988244996L, 281474988244995L, 281474988244997L, 281474985099266L } };
	protected final long[] startRefs = { 281475015507969L };
	protected final long[] endRefs = { 281474985099266L };
	protected final float[][] startPoss = { { 39.447338f, 9.998177f, -0.784811f } };
	protected final float[][] endPoss = { { 19.292645f, 11.611748f, -57.750366f } };

	protected NavMeshQuery query;
	protected NavMesh navmesh;

	@Before
	public void setUp() {
		navmesh = createNavMesh();
		query = new NavMeshQuery(navmesh);
	}

	protected NavMesh createNavMesh() {
		return new TestTiledNavMeshBuilder().getNavMesh();
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
