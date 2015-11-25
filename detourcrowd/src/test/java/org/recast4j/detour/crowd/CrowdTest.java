package org.recast4j.detour.crowd;

import org.junit.Test;

public class CrowdTest extends AbstractCrowdTest {

	@Test
	public void test() {
		int updateFlags = 0;//CrowdAgent.DT_CROWD_ANTICIPATE_TURNS | CrowdAgent.DT_CROWD_OPTIMIZE_VIS |
		addAgentGrid(3, 0.4f, updateFlags, startPoss[0]);
		for (CrowdAgent ag : crowd.getActiveAgents()) {
			crowd.requestMoveTarget(ag.getAgentIndex(), endRefs[0], endPoss[0]);
		}
		dumpActiveAgents();
		crowd.update(0.1f, null);
		dumpActiveAgents();
		//crowd.addAgent(startPoss[0], ap);
	}

	private void dumpActiveAgents() {
		System.out.println(crowd.getActiveAgents().size());
		for (CrowdAgent ag : crowd.getActiveAgents()) {
			System.out.println(ag.state + ", " + ag.targetState);
			System.out.println(ag.npos[0] + ", " + ag.npos[1] + ", " + ag.npos[2]);
			System.out.println(ag.nvel[0] + ", " + ag.nvel[1] + ", " + ag.nvel[2]);
		}
	}

}
