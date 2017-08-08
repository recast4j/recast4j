package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.ZipFile;

class GraphConnectionReader extends BinaryReader {

	@SuppressWarnings("unused")
	List<int[]> read(ZipFile file, String filename, GraphMeta meta, int[] indexToNode) throws IOException {
		List<int[]> connections = new ArrayList<>();
		ByteBuffer buffer = toByteBuffer(file, filename);
		while (buffer.remaining() > 0) {
			int count = buffer.getInt();
			int [] nodeConnections = new int[count];
			connections.add(nodeConnections);
			for (int i = 0; i < count; i++) {
				int nodeIndex = buffer.getInt();
				nodeConnections[i] = indexToNode[nodeIndex];
				// XXX: Is there anything we can do with the cost? 
				int cost = buffer.getInt();
			}
		}
		return connections;
	}
	
}
