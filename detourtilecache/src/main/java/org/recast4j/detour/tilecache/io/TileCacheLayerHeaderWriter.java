package org.recast4j.detour.tilecache.io;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteOrder;

import org.recast4j.detour.io.DetourWriter;
import org.recast4j.detour.tilecache.TileCacheLayerHeader;

public class TileCacheLayerHeaderWriter extends DetourWriter {

	public void write(OutputStream stream, TileCacheLayerHeader header, ByteOrder order) throws IOException {
		write(stream, header.magic, order);
		write(stream, header.version, order);
		write(stream, header.tx, order);
		write(stream, header.ty, order);
		write(stream, header.tlayer, order);
		for (int j = 0; j < 3; j++) {
			write(stream, header.bmin[j], order);
		}
		for (int j = 0; j < 3; j++) {
			write(stream, header.bmax[j], order);
		}
		write(stream, (short)header.hmin, order);
		write(stream, (short)header.hmax, order);
		stream.write(header.width);
		stream.write(header.height);
		stream.write(header.minx);
		stream.write(header.maxx);
		stream.write(header.miny);
		stream.write(header.maxy);
	}

}
