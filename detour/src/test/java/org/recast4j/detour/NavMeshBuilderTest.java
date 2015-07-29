/*
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour;

import org.junit.Assert;
import org.junit.Test;

public class NavMeshBuilderTest extends AbstractDetourTest {

	@Test
	public void testBVTree() {
		Assert.assertEquals(223, nmd.navVerts.length / 3);
		Assert.assertEquals(118, nmd.navPolys.length);
		Assert.assertEquals(453, nmd.header.maxLinkCount);
		Assert.assertEquals(59, nmd.navDVerts.length / 3);
		Assert.assertEquals(289, nmd.navDTris.length / 4);
		Assert.assertEquals(118, nmd.navDMeshes.length);
		Assert.assertEquals(0, nmd.offMeshCons.length);
		Assert.assertEquals(118, nmd.header.offMeshBase);
		Assert.assertEquals(236, nmd.navBvtree.length);
		Assert.assertTrue(nmd.header.bvNodeCount <= nmd.navBvtree.length);
		for (int i = 0; i < nmd.header.bvNodeCount; i++) {
			Assert.assertNotNull(nmd.navBvtree[i]);
		}
		//nmd.navBvtree
	}
}