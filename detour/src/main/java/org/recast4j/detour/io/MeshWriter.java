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

import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;

public class MeshWriter {

	public void write(OutputStream stream, MeshData data) throws IOException {
		MeshHeader header = data.header;
		write(stream, header.magic);
		write(stream, header.version);
		write(stream, header.x);
		write(stream, header.y);
		write(stream, header.layer);
		write(stream, header.userId);
		write(stream, header.polyCount);
		write(stream, header.vertCount);
		write(stream, header.maxLinkCount);
		write(stream, header.detailMeshCount);
		write(stream, header.detailVertCount);
		write(stream, header.detailTriCount);
		write(stream, header.bvNodeCount);
		write(stream, header.offMeshConCount);
		write(stream, header.offMeshBase);
		write(stream, header.walkableHeight);
		write(stream, header.walkableRadius);
		write(stream, header.walkableClimb);
		write(stream, header.bmin[0]);
		write(stream, header.bmin[1]);
		write(stream, header.bmin[2]);
		write(stream, header.bmax[0]);
		write(stream, header.bmax[1]);
		write(stream, header.bmax[2]);
		write(stream, header.bvQuantFactor);
		writeVerts(stream, data.verts, header.vertCount);
		writePolys(stream, data);
		writePolyDetails(stream, data);
		align4(stream, data.header.detailMeshCount * MeshReader.DT_POLY_DETAIL_SIZE);
		writeVerts(stream, data.detailVerts, header.detailVertCount);
		writeDTris(stream, data);
		writeBVTree(stream, data);
		writeOffMeshCons(stream, data);
	}

	private void write(OutputStream stream, float value) throws IOException {
		write(stream, Float.floatToIntBits(value));
	}

	private void write(OutputStream stream, short value) throws IOException {
		stream.write((value >> 8) & 0xFF);
		stream.write(value & 0xFF);
	}

	private void write(OutputStream stream, int value) throws IOException {
		stream.write((value >> 24) & 0xFF);
		stream.write((value >> 16) & 0xFF);
		stream.write((value >> 8) & 0xFF);
		stream.write(value & 0xFF);
	}

	private void writeVerts(OutputStream stream, float[] verts, int count) throws IOException {
		for (int i = 0; i < count * 3; i++) {
			write(stream, verts[i]);
		}
	}

	private void writePolys(OutputStream stream, MeshData data) throws IOException {
		for (int i = 0; i < data.header.polyCount; i++) {
			write(stream, data.polys[i].firstLink);
			for (int j = 0; j < data.polys[i].verts.length; j++) {
				write(stream, (short)data.polys[i].verts[j]);
			}
			for (int j = 0; j < data.polys[i].neis.length; j++) {
				write(stream, (short)data.polys[i].neis[j]);
			}
			write(stream, (short)data.polys[i].flags);
			stream.write(data.polys[i].vertCount);
			stream.write(data.polys[i].areaAndtype);
		}
	}

	private void writePolyDetails(OutputStream stream, MeshData data) throws IOException {
		for (int i = 0; i < data.header.detailMeshCount; i++) {
			write(stream, data.detailMeshes[i].vertBase);
			write(stream, data.detailMeshes[i].triBase);
			stream.write(data.detailMeshes[i].vertCount);
			stream.write(data.detailMeshes[i].triCount);
		}
	}

	private void writeDTris(OutputStream stream, MeshData data) throws IOException {
		for (int i = 0; i < data.header.detailTriCount * 4; i++) {
			stream.write(data.detailTris[i]);
		}
	}

	private void writeBVTree(OutputStream stream, MeshData data) throws IOException {
		for (int i = 0; i < data.header.bvNodeCount; i++) {
			for (int j = 0; j < 3; j++) {
				write(stream, (short)data.bvTree[i].bmin[j]);
			}
			for (int j = 0; j < 3; j++) {
				write(stream, (short)data.bvTree[i].bmax[j]);
			}
			write(stream, data.bvTree[i].i);
		}
	}

	private void writeOffMeshCons(OutputStream stream, MeshData data) throws IOException {
		for (int i = 0; i < data.header.offMeshConCount; i++) {
			for (int j = 0; j < 6; j++) {
				write(stream, data.offMeshCons[i].pos[j]);
			}
			write(stream, data.offMeshCons[i].rad);
			write(stream, (short)data.offMeshCons[i].poly);
			stream.write(data.offMeshCons[i].flags);
			stream.write(data.offMeshCons[i].side);
			write(stream, data.offMeshCons[i].userId);
		}
	}

	private void align4(OutputStream stream, int size) throws IOException {
		int toSkip = ((size + 3) & ~3) - size;
		for (int i = 0; i < toSkip; i++) {
			stream.write(0);
		}
	}
}
