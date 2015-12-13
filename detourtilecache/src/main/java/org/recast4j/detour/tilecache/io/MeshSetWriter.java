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
