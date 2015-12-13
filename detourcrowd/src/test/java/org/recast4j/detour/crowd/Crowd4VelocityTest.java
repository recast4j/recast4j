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
package org.recast4j.detour.crowd;

import org.junit.Assert;
import org.junit.Test;

public class Crowd4VelocityTest extends AbstractCrowdTest {

	static final float[][] EXPECTED_A1Q3TVTA = {
	
	{6.107922f,10.197294f,-17.673767f,0.007285f,0.000000f,0.010299f},
	{6.012396f,10.197294f,-17.578360f,-0.128094f,0.000000f,0.116731f},
	{5.985875f,10.197294f,-17.553473f,-0.127547f,0.000000f,0.117330f},
	{5.960416f,10.197294f,-17.529953f,-0.127297f,0.000000f,0.117600f},
	{5.700428f,10.197294f,-17.288725f,-2.565432f,0.000000f,2.380874f},
	{5.218917f,10.197294f,-16.816570f,-2.496289f,0.000000f,2.453271f},
	{4.762917f,10.197294f,-16.285475f,-2.280005f,0.000000f,2.655481f},
	{4.389729f,10.197294f,-15.693249f,-1.865936f,0.000000f,2.961128f},
	{4.004944f,10.197294f,-15.108492f,-1.923926f,0.000000f,2.923783f},
	{3.626657f,10.197294f,-14.519511f,-1.891437f,0.000000f,2.944905f},
	{3.258153f,10.197294f,-13.924360f,-1.842520f,0.000000f,2.975755f},
	{2.905435f,10.197294f,-13.319720f,-1.763592f,0.000000f,3.023201f},
	{2.580974f,10.197294f,-12.699458f,-1.622302f,0.000000f,3.101312f},
	{2.317390f,10.197294f,-12.050981f,-1.317922f,0.000000f,3.242388f},
	{2.164229f,10.197294f,-11.367942f,-0.765802f,0.000000f,3.415194f},
	{2.032890f,10.197294f,-10.680373f,-0.656695f,0.000000f,3.437841f},
	{1.957141f,10.197294f,-9.984484f,-0.378744f,0.000000f,3.479447f},
	{1.652727f,10.197294f,-9.354141f,-1.522072f,0.000000f,3.151713f},
	{1.657769f,10.197294f,-8.654160f,0.025210f,0.000000f,3.499909f},
	{1.967047f,10.197294f,-8.053377f,2.042668f,0.000000f,2.842096f},
	{2.035211f,10.197294f,-7.662985f,-1.547804f,0.000000f,0.303984f},
	{2.224357f,10.197294f,-7.568842f,1.323664f,0.000000f,-0.454743f},
	{2.145242f,10.197294f,-7.649159f,-1.043294f,0.000000f,-0.822816f},
	{1.997615f,10.197294f,-7.753769f,-0.738132f,0.000000f,-0.523052f},
	{1.849989f,10.197294f,-7.858380f,-0.738132f,0.000000f,-0.523052f},
	{1.743151f,10.197294f,-8.124065f,-0.534188f,0.000000f,-1.328430f},
	{1.705214f,10.197294f,-8.524739f,-0.189685f,0.000000f,-2.003367f},
	{1.705185f,10.197294f,-9.073907f,-0.000146f,0.000000f,-2.745840f},
	{1.740830f,10.197294f,-9.670231f,0.178223f,0.000000f,-2.981619f},
	{1.821623f,10.197294f,-10.299744f,0.403966f,0.000000f,-3.147564f},
	{1.922628f,10.197294f,-10.944490f,0.505026f,0.000000f,-3.223732f},
	{2.047351f,10.197294f,-11.612516f,0.623614f,0.000000f,-3.340129f},
	{2.172074f,10.197294f,-12.280542f,0.623614f,0.000000f,-3.340129f},
	{2.296797f,10.197294f,-12.948568f,0.623614f,0.000000f,-3.340129f},
	{2.421520f,10.197294f,-13.616594f,0.623614f,0.000000f,-3.340129f},
	{2.546243f,10.197294f,-14.284620f,0.623614f,0.000000f,-3.340129f},
	{2.670966f,10.197294f,-14.952646f,0.623614f,0.000000f,-3.340129f},
	{2.812940f,10.197294f,-15.618809f,0.709870f,0.000000f,-3.330812f},
	{2.954914f,10.197294f,-16.284971f,0.709870f,0.000000f,-3.330812f},
	{3.096888f,10.197294f,-16.951134f,0.709870f,0.000000f,-3.330812f},
	{3.242095f,10.197294f,-17.629868f,0.726036f,0.000000f,-3.393668f},
	{3.416458f,10.197294f,-18.307804f,0.871815f,0.000000f,-3.389681f},
	{3.590821f,10.197294f,-18.985741f,0.871815f,0.000000f,-3.389681f},
	{3.765184f,10.197294f,-19.663677f,0.871815f,0.000000f,-3.389681f},
	{3.939547f,10.197294f,-20.341614f,0.871815f,0.000000f,-3.389681f},
	{4.113910f,10.197294f,-21.019550f,0.871815f,0.000000f,-3.389681f},
	{4.283285f,10.197294f,-21.609158f,0.846878f,0.000000f,-2.948040f},
	{4.294198f,10.197294f,-22.200867f,0.054562f,0.000000f,-2.958549f},
	{4.262735f,10.197294f,-22.810913f,-0.157313f,0.000000f,-3.050232f},
	{4.244248f,10.197294f,-23.420605f,-0.092435f,0.000000f,-3.048458f}};

	@Test
	public void testAgent1Quality3TVTA() {
		int updateFlags = CrowdAgent.DT_CROWD_ANTICIPATE_TURNS | CrowdAgent.DT_CROWD_OPTIMIZE_VIS
				| CrowdAgent.DT_CROWD_OPTIMIZE_TOPO | CrowdAgent.DT_CROWD_OBSTACLE_AVOIDANCE;

		addAgentGrid(2, 0.3f, updateFlags, 3, endPoss[0]);
		setMoveTarget(endPoss[4], false);
		for (int i = 0; i < EXPECTED_A1Q3TVTA.length; i++) {
			if (i == 20) {
				setMoveTarget(startPoss[2], true);
			}
			crowd.update(1 / 5f, null);
			CrowdAgent ag = crowd.getAgent(1);
			Assert.assertEquals(EXPECTED_A1Q3TVTA[i][0], ag.npos[0], 0.001f);
			Assert.assertEquals(EXPECTED_A1Q3TVTA[i][1], ag.npos[1], 0.001f);
			Assert.assertEquals(EXPECTED_A1Q3TVTA[i][2], ag.npos[2], 0.001f);
			Assert.assertEquals(EXPECTED_A1Q3TVTA[i][3], ag.nvel[0], 0.001f);
			Assert.assertEquals(EXPECTED_A1Q3TVTA[i][4], ag.nvel[1], 0.001f);
			Assert.assertEquals(EXPECTED_A1Q3TVTA[i][5], ag.nvel[2], 0.001f);
		}
	}

}
