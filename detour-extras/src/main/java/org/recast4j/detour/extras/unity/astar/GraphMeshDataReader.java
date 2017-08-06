package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.zip.ZipFile;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.MeshHeader;
import org.recast4j.detour.Poly;
import org.recast4j.detour.PolyDetail;

class GraphMeshDataReader extends BinaryReader {

	static final float INT_PRECISION_FACTOR = 1000f;

	@SuppressWarnings("unused")
	GraphMeshData read(ZipFile file, String filename, GraphMeta meta, int maxVertPerPoly) throws IOException {
		ByteBuffer buffer = toByteBuffer(file, filename);
		int tileXCount = buffer.getInt();
		if (tileXCount < 0)
			return null;
		int tileZCount = buffer.getInt();
		MeshData[] tiles = new MeshData[tileXCount * tileZCount];
		for (int z = 0; z < tileZCount; z++) {
			for (int x = 0; x < tileXCount; x++) {
				int tileIndex = x + z * tileXCount;
				int tx = buffer.getInt();
				int tz = buffer.getInt();
				if (tx != x || tz != z) {
					throw new IllegalArgumentException("Inconsistent tile positions");
				}

				tiles[tileIndex] = new MeshData();
				int width = buffer.getInt();
				int depth = buffer.getInt();

				int trisCount = buffer.getInt();
				int[] tris = new int[trisCount];
				for (int i = 0; i < tris.length; i++) {
					tris[i] = buffer.getInt();
				}

				int vertsCount = buffer.getInt();
				float[] verts = new float[3 * vertsCount];
				for (int i = 0; i < verts.length; i++) {
					verts[i] = buffer.getInt() / INT_PRECISION_FACTOR;
				}

				int[] vertsInGraphSpace = new int[3 * buffer.getInt()];
				for (int i = 0; i < vertsInGraphSpace.length; i++) {
					vertsInGraphSpace[i] = buffer.getInt();
				}

				int nodeCount = buffer.getInt();
				Poly[] nodes = new Poly[nodeCount];
				PolyDetail[] detailNodes = new PolyDetail[nodeCount];
				float[] detailVerts = new float[0];
				int[] detailTris = new int[4 * nodeCount];
				int vertMask = getVertMask(vertsCount);
				for (int i = 0; i < nodes.length; i++) {
					nodes[i] = new Poly(i, maxVertPerPoly);
					nodes[i].vertCount = 3;
					// XXX: What can we do with the penalty?
					int penalty = buffer.getInt();
					nodes[i].flags = buffer.getInt();
					nodes[i].verts[0] = buffer.getInt() & vertMask;
					nodes[i].verts[1] = buffer.getInt() & vertMask;
					nodes[i].verts[2] = buffer.getInt() & vertMask;
					// XXX: Detail mesh is not needed by recast4j, but RecastDemo will crash without it
					detailNodes[i] = new PolyDetail();
					detailNodes[i].vertBase = 0;
					detailNodes[i].vertCount = 0;
					detailNodes[i].triBase = i;
					detailNodes[i].triCount = 1;
					detailTris[4 * i] = 0;
					detailTris[4 * i + 1] = 1;
					detailTris[4 * i + 2] = 2;
					// Bit for each edge that belongs to poly boundary, basically all edges marked as boundary as it is a triangle
					detailTris[4 * i + 3] = (1 << 4) | (1 << 2) | 1;
				}

				tiles[tileIndex].verts = verts;
				tiles[tileIndex].polys = nodes;
				tiles[tileIndex].detailMeshes = detailNodes;
				tiles[tileIndex].detailVerts = detailVerts;
				tiles[tileIndex].detailTris = detailTris;
				MeshHeader header = new MeshHeader();
				header.magic = MeshHeader.DT_NAVMESH_MAGIC;
				header.version = MeshHeader.DT_NAVMESH_VERSION;
				header.x = x;
				header.y = z;
				header.polyCount = nodeCount;
				header.vertCount = vertsCount;
				header.detailMeshCount = nodeCount;
				header.detailTriCount = nodeCount;
				header.maxLinkCount = nodeCount * 3 * 2; // XXX: Needed by Recast, not needed by recast4j  
				header.bmin[0] = meta.forcedBoundsCenter.x + meta.forcedBoundsSize.x * ((float) x / tileXCount - 0.5f);
				header.bmin[1] = -0.5f * meta.forcedBoundsSize.y + meta.forcedBoundsCenter.y;
				header.bmin[2] = meta.forcedBoundsCenter.z + meta.forcedBoundsSize.z * ((float) z / tileZCount - 0.5f);
				header.bmax[0] = meta.forcedBoundsCenter.x + meta.forcedBoundsSize.x * ((x + 1f) / tileXCount - 0.5f);
				header.bmax[1] = 0.5f * meta.forcedBoundsSize.y + meta.forcedBoundsCenter.y;
				header.bmax[2] = meta.forcedBoundsCenter.z + meta.forcedBoundsSize.z * ((z + 1f) / tileZCount - 0.5f);
				header.bvQuantFactor = 1.0f / meta.cellSize;
				header.offMeshBase = nodeCount;
				tiles[tileIndex].header = header;
			}
		}
		return new GraphMeshData(tileXCount, tileZCount, tiles);
	}

	private int getVertMask(int vertsCount) {
		int vertMask = Integer.highestOneBit(vertsCount);
		if (vertMask != vertsCount) {
			vertMask *= 2;
		}
		vertMask--;
		return vertMask;
	}

}
