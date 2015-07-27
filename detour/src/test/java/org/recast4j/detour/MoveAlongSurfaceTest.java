package org.recast4j.detour;

import org.junit.Assert;
import org.junit.Test;

public class MoveAlongSurfaceTest extends AbstractDetourTest {

	long[][] moveAlongVisited = {
			new long[] { 281474976710696L, 281474976710695L, 281474976710694L, 281474976710703L, 281474976710706L,
					281474976710705L, 281474976710702L, 281474976710701L, 281474976710714L, 281474976710713L,
					281474976710712L, 281474976710727L, 281474976710730L, 281474976710717L, 281474976710721L },
			new long[] { 281474976710773L, 281474976710772L, 281474976710768L, 281474976710754L, 281474976710755L,
					281474976710753L },
			new long[] { 281474976710680L, 281474976710684L, 281474976710688L, 281474976710687L, 281474976710686L,
					281474976710697L, 281474976710695L, 281474976710694L, 281474976710703L, 281474976710706L,
					281474976710705L, 281474976710702L, 281474976710701L, 281474976710714L, 281474976710713L,
					281474976710712L, 281474976710727L, 281474976710730L, 281474976710717L, 281474976710721L,
					281474976710718L },
			new long[] { 281474976710753L, 281474976710748L, 281474976710752L, 281474976710731L },
			new long[] { 281474976710733L, 281474976710736L, 281474976710738L, 281474976710737L, 281474976710728L,
					281474976710724L, 281474976710717L, 281474976710729L, 281474976710731L, 281474976710752L,
					281474976710748L, 281474976710753L, 281474976710755L, 281474976710754L, 281474976710768L,
					281474976710772L } };
	float[][] moveAlongPosition = { { 6.457663f, 10.197294f, -18.334061f }, { -1.433933f, 10.197294f, -1.359993f },
			{ 12.184784f, 9.997294f, -18.941269f }, { 0.863553f, 10.197294f, -10.310320f },
			{ 18.784092f, 10.197294f, 3.054368f } };

	@Test
	public void testMoveAlongSurface() {
		QueryFilter filter = new QueryFilter();
		for (int i = 0; i < startRefs.length; i++) {
			long startRef = startRefs[i];
			float[] startPos = startPoss[i];
			float[] endPos = endPoss[i];
			MoveAlongSurfaceResult path = query.moveAlongSurface(startRef, startPos, endPos, filter);
			for (int v = 0; v < 3; v++) {
				Assert.assertEquals(moveAlongPosition[i][v], path.getResultPos()[v], 0.01f);
			}
			Assert.assertEquals(moveAlongVisited[i].length, path.getVisited().size());
			for (int j = 0; j < moveAlongPosition[i].length; j++) {
				Assert.assertEquals(moveAlongVisited[i][j], path.getVisited().get(j).longValue());
			}
		}
	}


}
