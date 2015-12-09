package org.recast4j.detour.tilecache.io;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteOrder;

import org.recast4j.detour.MeshTile;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.io.DetourWriter;

public class MeshSetWriter extends DetourWriter {

	public void write (OutputStream stream, NavMesh mesh, ByteOrder order, boolean cCompatibility) throws IOException {
		// Read header.
		write(stream, NavMeshSetHeader.NAVMESHSET_MAGIC, order);
		write(stream, NavMeshSetHeader.NAVMESHSET_VERSION, order);
		int numTiles = 0;
		for (int i = 0; i < mesh.getMaxTiles(); ++i)
		{
			MeshTile tile = mesh.getTile(i);
			if (tile == null || tile.data == null || tile.data.header == null) continue;
			numTiles++;
		}	
		write(stream, numTiles, order);
		write(stream, mesh.getParams().orig[0], order);
		write(stream, mesh.getParams().orig[1], order);
		write(stream, mesh.getParams().orig[2], order);
		write(stream, mesh.getParams().tileWidth, order);
		write(stream, mesh.getParams().tileHeight, order);
		write(stream, mesh.getParams().maxTiles, order);
		write(stream, mesh.getParams().maxPolys, order);
		
	}
}
