package org.recast4j.detour;

import org.junit.Assert;
import org.junit.Test;

public class FindNearestPolyTest extends AbstractDetourTest {

	long[] findNearestPolyRefs = { 281474976710696L, 281474976710773L, 281474976710680L, 281474976710753L,
			281474976710733L };
	float[][] findNearestPolyPos = { { 22.606520f, 10.197294f, -45.918674f }, { 22.331268f, 10.197294f, -1.040187f },
			{ 18.694363f, 15.803535f, -73.090416f }, { 0.745335f, 10.197294f, -5.940050f },
			{ -20.651257f, 5.904126f, -13.712508f } };

	@Test
	public void testFindNearestPoly() {
		QueryFilter filter = new QueryFilter();
		float[] extents = { 2, 4, 2 };
		for (int i = 0; i < startRefs.length; i++) {
			float[] startPos = startPoss[i];
			FindNearestPolyResult poly = query.findNearestPoly(startPos, extents, filter);
			Assert.assertEquals(findNearestPolyRefs[i], poly.getNearestRef());
			for (int v = 0; v < findNearestPolyPos[i].length; v++) {
				Assert.assertEquals(findNearestPolyPos[i][v], poly.getNearestPos()[v], 0.001f);
			}
		}

	}
}
