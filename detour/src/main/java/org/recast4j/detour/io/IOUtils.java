package org.recast4j.detour.io;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;

public class IOUtils {

	public static ByteBuffer toByteBuffer(InputStream inputStream) throws IOException {
		ByteArrayOutputStream baos = new ByteArrayOutputStream();
		byte[] buffer = new byte[4096];
		int l;
		while ((l = inputStream.read(buffer)) != -1) {
			baos.write(buffer, 0, l);
		}
		return ByteBuffer.wrap(baos.toByteArray());
	}

	
}
