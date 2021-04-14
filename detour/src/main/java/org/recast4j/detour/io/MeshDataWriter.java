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

import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;

public class MeshDataWriter extends DetourWriter {

	public void write(OutputStream stream, MeshData data, ByteOrder order, boolean cCompatibility) throws IOException {
		MeshHeader header = data.header;
		write(stream, header.magic, order);
		write(stream, cCompatibility ? MeshHeader.DT_NAVMESH_VERSION : MeshHeader.DT_NAVMESH_VERSION_RECAST4J_LAST, order);
		write(stream, header.x, order);
		write(stream, header.y, order);
		write(stream, header.layer, order);
		write(stream, header.userId, order);
		write(stream, header.polyCount, order);
		write(stream, header.vertCount, order);
		write(stream, header.maxLinkCount, order);
		write(stream, header.detailMeshCount, order);
		write(stream, header.detailVertCount, order);
		write(stream, header.detailTriCount, order);
		write(stream, header.bvNodeCount, order);
		write(stream, header.offMeshConCount, order);
		write(stream, header.offMeshBase, order);
		write(stream, header.walkableHeight, order);
		write(stream, header.walkableRadius, order);
		write(stream, header.walkableClimb, order);
		write(stream, header.bmin[0], order);
		write(stream, header.bmin[1], order);
		write(stream, header.bmin[2], order);
		write(stream, header.bmax[0], order);
		write(stream, header.bmax[1], order);
		write(stream, header.bmax[2], order);
		write(stream, header.bvQuantFactor, order);
		writeVerts(stream, data.verts, header.vertCount, order);
		writePolys(stream, data, order, cCompatibility);
		if (cCompatibility) {
			byte[] linkPlaceholder = new byte[header.maxLinkCount * MeshDataReader.getSizeofLink(false)];
			stream.write(linkPlaceholder);
		}
		writePolyDetails(stream, data, order, cCompatibility);
		writeVerts(stream, data.detailVerts, header.detailVertCount, order);
		writeDTris(stream, data);
		writeBVTree(stream, data, order);
		writeOffMeshCons(stream, data, order);
	}

	private void writeVerts(OutputStream stream, float[] verts, int count, ByteOrder order) throws IOException {
		for (int i = 0; i < count * 3; i++) {
			write(stream, verts[i], order);
		}
	}

	private void writePolys(OutputStream stream, MeshData data, ByteOrder order, boolean cCompatibility) throws IOException {
		for (int i = 0; i < data.header.polyCount; i++) {
		    if (cCompatibility) {
		        write(stream, 0xFFFF, order);
		    }
			for (int j = 0; j < data.polys[i].verts.length; j++) {
				write(stream, (short) data.polys[i].verts[j], order);
			}
			for (int j = 0; j < data.polys[i].neis.length; j++) {
				write(stream, (short) data.polys[i].neis[j], order);
			}
			write(stream, (short) data.polys[i].flags, order);
			stream.write(data.polys[i].vertCount);
			stream.write(data.polys[i].areaAndtype);
		}
	}

	private void writePolyDetails(OutputStream stream, MeshData data, ByteOrder order, boolean cCompatibility)
			throws IOException {
		for (int i = 0; i < data.header.detailMeshCount; i++) {
			write(stream, data.detailMeshes[i].vertBase, order);
			write(stream, data.detailMeshes[i].triBase, order);
			stream.write(data.detailMeshes[i].vertCount);
			stream.write(data.detailMeshes[i].triCount);
			if (cCompatibility) {
				write(stream, (short) 0, order);
			}
		}
	}

	private void writeDTris(OutputStream stream, MeshData data) throws IOException {
		for (int i = 0; i < data.header.detailTriCount * 4; i++) {
			stream.write(data.detailTris[i]);
		}
	}

	private void writeBVTree(OutputStream stream, MeshData data, ByteOrder order) throws IOException {
		for (int i = 0; i < data.header.bvNodeCount; i++) {
			for (int j = 0; j < 3; j++) {
				write(stream, (short) data.bvTree[i].bmin[j], order);
			}
			for (int j = 0; j < 3; j++) {
				write(stream, (short) data.bvTree[i].bmax[j], order);
			}
			write(stream, data.bvTree[i].i, order);
		}
	}

	private void writeOffMeshCons(OutputStream stream, MeshData data, ByteOrder order) throws IOException {
		for (int i = 0; i < data.header.offMeshConCount; i++) {
			for (int j = 0; j < 6; j++) {
				write(stream, data.offMeshCons[i].pos[j], order);
			}
			write(stream, data.offMeshCons[i].rad, order);
			write(stream, (short) data.offMeshCons[i].poly, order);
			stream.write(data.offMeshCons[i].flags);
			stream.write(data.offMeshCons[i].side);
			write(stream, data.offMeshCons[i].userId, order);
		}
	}

}
