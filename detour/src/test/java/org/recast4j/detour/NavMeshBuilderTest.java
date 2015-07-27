package org.recast4j.detour;

import org.junit.Assert;
import org.junit.Test;

public class NavMeshBuilderTest extends AbstractDetourTest {

	@Test
	public void testBVTree() {
		Assert.assertTrue(nmd.header.bvNodeCount <= nmd.navBvtree.length);
		for (int i = 0; i < nmd.header.bvNodeCount; i++) {
			Assert.assertNotNull(nmd.navBvtree[i]);
		}
		//nmd.navBvtree
	}
}