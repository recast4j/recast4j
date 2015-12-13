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
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.io.IOUtils;
import org.recast4j.detour.io.MeshReader;

public class MeshSetReader {

	private final MeshReader meshReader = new MeshReader();

	public NavMesh read(InputStream is, ByteOrder order, boolean cCompatibility) throws IOException {
		// Read header.
		ByteBuffer bb = IOUtils.toByteBuffer(is);
		bb.order(order);
		return read(bb, cCompatibility);
	}

	private NavMesh read(ByteBuffer bb, boolean cCompatibility) throws IOException {
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
		header.params.orig[0] = bb.getFloat();
		header.params.orig[1] = bb.getFloat();
		header.params.orig[2] = bb.getFloat();
		header.params.tileWidth = bb.getFloat();
		header.params.tileHeight = bb.getFloat();
		header.params.maxTiles = bb.getInt();
		header.params.maxPolys = bb.getInt();
		System.out.println(header.numTiles);
		System.out.println(header.params.tileWidth);
		System.out.println(header.params.tileHeight);
		System.out.println(header.params.maxTiles);
		System.out.println(header.params.maxPolys);
		
		NavMesh mesh = new NavMesh();
		mesh.init(header.params);
		
		// Read tiles.
		for (int i = 0; i < header.numTiles; ++i)
		{
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
			MeshData data = meshReader.read(bb, cCompatibility);
			mesh.addTile(data, i, tileHeader.tileRef);
		}
		return mesh;
	}
	
}
/*

void Sample_TileMesh::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;
	
	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;
	
	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

dtNavMesh* Sample_TileMesh::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;
	
	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}
	
	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}
		
	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
			return 0;

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
			return 0;

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	
	fclose(fp);
	
	return mesh;
}

*/
