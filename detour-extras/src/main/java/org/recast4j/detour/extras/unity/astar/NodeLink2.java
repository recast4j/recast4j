package org.recast4j.detour.extras.unity.astar;

import org.recast4j.detour.extras.Vector3f;

class NodeLink2 {
	final long linkID;
	final int startNode;
	final int endNode;
	final Vector3f clamped1;
	final Vector3f clamped2;

	NodeLink2(long linkID, int startNode, int endNode, Vector3f clamped1, Vector3f clamped2) {
		super();
		this.linkID = linkID;
		this.startNode = startNode;
		this.endNode = endNode;
		this.clamped1 = clamped1;
		this.clamped2 = clamped2;
	}

}
