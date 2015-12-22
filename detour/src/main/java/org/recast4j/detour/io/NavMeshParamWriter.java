package org.recast4j.detour.io;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteOrder;

import org.recast4j.detour.NavMeshParams;

public class NavMeshParamWriter extends DetourWriter {

	public void write(OutputStream stream, NavMeshParams params, ByteOrder order) throws IOException {
		write(stream, params.orig[0], order);
		write(stream, params.orig[1], order);
		write(stream, params.orig[2], order);
		write(stream, params.tileWidth, order);
		write(stream, params.tileHeight, order);
		write(stream, params.maxTiles, order);
		write(stream, params.maxPolys, order);
	}

}
