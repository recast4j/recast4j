package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.zip.ZipFile;

import org.recast4j.detour.extras.Vector3f;

class NodeLink2Reader extends BinaryReader {

	@SuppressWarnings("unused")
	NodeLink2[] read(ZipFile file, String filename, int[] indexToNode) throws IOException {
		ByteBuffer buffer = toByteBuffer(file, filename);
		int linkCount = buffer.getInt();
		NodeLink2[] links = new NodeLink2[linkCount];
		for (int i = 0; i < linkCount; i++) {
			long linkID = buffer.getLong();
			int startNode = indexToNode[buffer.getInt()];
			int endNode = indexToNode[buffer.getInt()];
			int connectedNode1 = buffer.getInt();
			int connectedNode2 = buffer.getInt();
			Vector3f clamped1 = new Vector3f();
			clamped1.x = buffer.getFloat();
			clamped1.y = buffer.getFloat();
			clamped1.z = buffer.getFloat();
			Vector3f clamped2 = new Vector3f();
			clamped2.x = buffer.getFloat();
			clamped2.y = buffer.getFloat();
			clamped2.z = buffer.getFloat();
			boolean postScanCalled = buffer.get() != 0;
			links[i] = new NodeLink2(linkID, startNode, endNode, clamped1, clamped2);
		}
		return links;
	}
}
