package org.recast4j.detour.io;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteOrder;

public abstract class DetourWriter {

	protected void write(OutputStream stream, float value, ByteOrder order) throws IOException {
		write(stream, Float.floatToIntBits(value), order);
	}

	protected void write(OutputStream stream, short value, ByteOrder order) throws IOException {
		if (order == ByteOrder.BIG_ENDIAN) {
			stream.write((value >> 8) & 0xFF);
			stream.write(value & 0xFF);
		} else {
			stream.write(value & 0xFF);
			stream.write((value >> 8) & 0xFF);
		}
	}

	protected void write(OutputStream stream, int value, ByteOrder order) throws IOException {
		if (order == ByteOrder.BIG_ENDIAN) {
			stream.write((value >> 24) & 0xFF);
			stream.write((value >> 16) & 0xFF);
			stream.write((value >> 8) & 0xFF);
			stream.write(value & 0xFF);
		} else {
			stream.write(value & 0xFF);
			stream.write((value >> 8) & 0xFF);
			stream.write((value >> 16) & 0xFF);
			stream.write((value >> 24) & 0xFF);
		}
	}
	
}
