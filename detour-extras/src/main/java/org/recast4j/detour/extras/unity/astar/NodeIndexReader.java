package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.zip.ZipFile;

class NodeIndexReader extends BinaryReader {

	int[] read(ZipFile file, String filename) throws IOException {
		ByteBuffer buffer = toByteBuffer(file, filename);
		int maxNodeIndex = buffer.getInt();
		int[] int2Node = new int[maxNodeIndex + 1];
		int node = 0;
		while (buffer.remaining() > 0) {
			int index = buffer.getInt();
			int2Node[index] = node++;
		}
		return int2Node;
	}
	
}
