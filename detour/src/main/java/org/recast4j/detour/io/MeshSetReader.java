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
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;

public class MeshSetReader {

	private final MeshDataReader meshReader = new MeshDataReader();
	private final NavMeshParamReader paramReader = new NavMeshParamReader();

	public NavMesh read(InputStream is, ByteOrder order, boolean cCompatibility) throws IOException {
		// Read header.
		ByteBuffer bb = IOUtils.toByteBuffer(is);
		bb.order(order);
		return read(bb, cCompatibility);
	}

	public NavMesh read(ByteBuffer bb, boolean cCompatibility) throws IOException {
		NavMeshSetHeader header = new NavMeshSetHeader();
		header.magic = bb.getInt();
		if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC) {
			throw new IOException("Invalid magic");
		}
		header.version = bb.getInt();
		if (header.version != NavMeshSetHeader.NAVMESHSET_VERSION) {
			throw new IOException("Invalid version");
		}
		header.numTiles = bb.getInt();
		header.params = paramReader.read(bb);
		NavMesh mesh = new NavMesh(header.params);

		// Read tiles.
		for (int i = 0; i < header.numTiles; ++i) {
			NavMeshTileHeader tileHeader = new NavMeshTileHeader();
			tileHeader.tileRef = bb.getLong();
			tileHeader.dataSize = bb.getInt();
			System.out.println("ref = " + tileHeader.tileRef + "  " + tileHeader.dataSize);
			if (tileHeader.tileRef == 0 || tileHeader.dataSize == 0) {
				break;
			}
			if (cCompatibility) {
				bb.getInt(); // C struct padding
			}
			MeshData data = meshReader.read(bb, header.params.maxVertPerPoly, cCompatibility);
			mesh.addTile(data, i, tileHeader.tileRef);
		}
		return mesh;
	}


}
