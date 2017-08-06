package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import org.recast4j.detour.io.IOUtils;

abstract class BinaryReader {

	protected ByteBuffer toByteBuffer(ZipFile file, String filename) throws IOException {
		ZipEntry graphReferences = file.getEntry(filename);
		ByteBuffer buffer = IOUtils.toByteBuffer(file.getInputStream(graphReferences));
		buffer.order(ByteOrder.LITTLE_ENDIAN);
		return buffer;
	}

}
