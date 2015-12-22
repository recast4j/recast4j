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

	protected void write(OutputStream stream, long value, ByteOrder order) throws IOException {
		if (order == ByteOrder.BIG_ENDIAN) {
			write(stream, (int)(value >>> 32), order);
			write(stream, (int)(value & 0xFFFFFFFF), order);
		} else {
			write(stream, (int)(value & 0xFFFFFFFF), order);
			write(stream, (int)(value >>> 32), order);
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
