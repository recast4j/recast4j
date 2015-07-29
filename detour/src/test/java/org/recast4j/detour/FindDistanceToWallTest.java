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

public class FindDistanceToWallTest extends AbstractDetourTest {

	float[] distancesToWall = { 0.597511f, 3.201085f, 0.603713f, 2.791475f, 2.815544f };
	float[][] hitPosition = { { 23.177608f, 10.197294f, -45.742954f }, { 22.331268f, 10.197294f, -4.241272f },
			{ 18.108675f, 15.743596f, -73.236839f }, { 1.984785f, 10.197294f, -8.441269f },
			{ -22.315216f, 4.997294f, -11.441269f } };
	float[][] hitNormal = { { -0.955779f, 0.000000f, -0.294087f }, { 0.000000f, 0.000000f, 1.000000f },
			{ 0.965395f, 0.098799f, 0.241351f }, { -0.444012f, 0.000000f, 0.896021f },
			{ 0.562533f, 0.306572f, -0.767835f } };

	@Test
	public void testFindDistanceToWall() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			float[] startPos = startPoss[i];
			FindDistanceToWallResult hit = query.findDistanceToWall(startRefs[i], startPos, 3.5f, filter);
			Assert.assertEquals(distancesToWall[i], hit.getDistance(), 0.001f);
			for (int v = 0; v < 3; v++) {
				Assert.assertEquals(hitPosition[i][v], hit.getPosition()[v], 0.001f);
			}
			for (int v = 0; v < 3; v++) {
				Assert.assertEquals(hitNormal[i][v], hit.getNormal()[v], 0.001f);
			}
		}

	}
}
