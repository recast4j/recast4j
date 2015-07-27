package org.recast4j.detour;

import org.junit.Assert;
import org.junit.Test;

public class FindLocalNeighbourhoodTest extends AbstractDetourTest {

	long[][] findLocalNeighbourhoodRefs = { { 281474976710696L, 281474976710695L, 281474976710691L, 281474976710697L },
			{ 281474976710773L, 281474976710769L, 281474976710772L },
			{ 281474976710680L, 281474976710674L, 281474976710679L, 281474976710684L, 281474976710683L,
					281474976710678L, 281474976710677L, 281474976710676L },
			{ 281474976710753L, 281474976710748L, 281474976710750L, 281474976710752L },
			{ 281474976710733L, 281474976710735L, 281474976710736L }

	};
	long[][] findLocalNeighbourhoodParentRefs = { { 0L, 281474976710696L, 281474976710695L, 281474976710695L },
			{ 0L, 281474976710773L, 281474976710773L },
			{ 0L, 281474976710680L, 281474976710680L, 281474976710680L, 281474976710680L, 281474976710679L,
					281474976710683L, 281474976710678L },
			{ 0L, 281474976710753L, 281474976710753L, 281474976710748L }, { 0L, 281474976710733L, 281474976710733L } };

	@Test
	public void testFindNearestPoly() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			float[] startPos = startPoss[i];
			FindLocalNeighbourhoodResult poly = query.findLocalNeighbourhood(startRefs[i], startPos, 3.5f, filter);
			Assert.assertEquals(findLocalNeighbourhoodRefs[i].length, poly.getRefs().size());
			for (int v = 0; v < findLocalNeighbourhoodRefs[i].length; v++) {
				Assert.assertEquals(findLocalNeighbourhoodRefs[i][v], poly.getRefs().get(v).longValue());
			}
		}

	}

}
