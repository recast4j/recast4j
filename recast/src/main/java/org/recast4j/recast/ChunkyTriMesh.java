package org.recast4j.recast;

import java.util.Arrays;
import java.util.Comparator;

public class ChunkyTriMesh {

	private static class BoundsItem {
		final float[] bmin = new float[2];
		final float[] bmax = new float[2];
		int i;
	}

	private static class ChunkyTriMeshNode {
		final float[] bmin = new float[2];
		final float[] bmax = new float[2];
		int i, n;
	}

	private static class CompareItemX implements Comparator<BoundsItem> {
		@Override
		public int compare(BoundsItem a, BoundsItem b) {
			if (a.bmin[0] < b.bmin[0])
				return -1;
			if (a.bmin[0] > b.bmin[0])
				return 1;
			return 0;
		}
	}

	private static class CompareItemY implements Comparator<BoundsItem> {
		@Override
		public int compare(BoundsItem a, BoundsItem b) {
			if (a.bmin[1] < b.bmin[1])
				return -1;
			if (a.bmin[1] > b.bmin[1])
				return 1;
			return 0;
		}
	}

	ChunkyTriMeshNode[] nodes;
	int nnodes;
	int[] tris;
	int ntris;
	int maxTrisPerChunk;

	private static void calcExtends(BoundsItem[] items, int imin, int imax, float[] bmin, float[] bmax) {
		bmin[0] = items[imin].bmin[0];
		bmin[1] = items[imin].bmin[1];

		bmax[0] = items[imin].bmax[0];
		bmax[1] = items[imin].bmax[1];

		for (int i = imin + 1; i < imax; ++i) {
			BoundsItem it = items[i];
			if (it.bmin[0] < bmin[0])
				bmin[0] = it.bmin[0];
			if (it.bmin[1] < bmin[1])
				bmin[1] = it.bmin[1];

			if (it.bmax[0] > bmax[0])
				bmax[0] = it.bmax[0];
			if (it.bmax[1] > bmax[1])
				bmax[1] = it.bmax[1];
		}
	}

	private static int longestAxis(float x, float y) {
		return y > x ? 1 : 0;
	}

	private static void subdivide(BoundsItem[] items, int nitems, int imin, int imax, int trisPerChunk, int curNode,
			ChunkyTriMeshNode[] nodes, int maxNodes, int curTri, int[] outTris, int[] inTris) {
		int inum = imax - imin;
		int icur = curNode;

		if (curNode > maxNodes)
			return;

		ChunkyTriMeshNode node = nodes[curNode++];

		if (inum <= trisPerChunk) {
			// Leaf
			calcExtends(items, imin, imax, node.bmin, node.bmax);

			// Copy triangles.
			node.i = curTri;
			node.n = inum;

			for (int i = imin; i < imax; ++i) {
				int src = items[i].i * 3;
				int dst = curTri * 3;
				curTri++;
				outTris[dst] = inTris[src];
				outTris[dst + 1] = inTris[src + 1];
				outTris[dst + 2] = inTris[src + 2];
			}
		} else {
			// Split
			calcExtends(items, imin, imax, node.bmin, node.bmax);

			int axis = longestAxis(node.bmax[0] - node.bmin[0], node.bmax[1] - node.bmin[1]);

			if (axis == 0) {
				Arrays.sort(items, imin, imax, new CompareItemX());
				// Sort along x-axis
			} else if (axis == 1) {
				Arrays.sort(items, imin, imax, new CompareItemY());
				// Sort along y-axis
			}

			int isplit = imin + inum / 2;

			// Left
			subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
			// Right
			subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

			int iescape = curNode - icur;
			// Negative index means escape.
			node.i = -iescape;
		}
	}

	public static ChunkyTriMesh createChunkyTriMesh(float[] verts, int[] tris, int ntris, int trisPerChunk) {
		ChunkyTriMesh cm = new ChunkyTriMesh();
		int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

		cm.nodes = new ChunkyTriMeshNode[nchunks * 4];
		cm.tris = new int[ntris * 3];
		cm.ntris = ntris;

		// Build tree
		BoundsItem[] items = new BoundsItem[ntris];

		for (int i = 0; i < ntris; i++) {
			int t = i * 3;
			BoundsItem it = items[i] = new BoundsItem();
			it.i = i;
			// Calc triangle XZ bounds.
			it.bmin[0] = it.bmax[0] = verts[tris[t] * 3 + 0];
			it.bmin[1] = it.bmax[1] = verts[tris[t] * 3 + 2];
			for (int j = 1; j < 3; ++j) {
				int v = tris[t + j] * 3;
				if (verts[v] < it.bmin[0])
					it.bmin[0] = verts[v];
				if (verts[v + 2] < it.bmin[1])
					it.bmin[1] = verts[v + 2];

				if (verts[v] > it.bmax[0])
					it.bmax[0] = verts[v];
				if (verts[v + 2] > it.bmax[1])
					it.bmax[1] = verts[v + 2];
			}
		}

		int curTri = 0;
		int curNode = 0;
		subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm.nodes, nchunks * 4, curTri, cm.tris, tris);

		cm.nnodes = curNode;

		// Calc max tris per node.
		cm.maxTrisPerChunk = 0;
		for (int i = 0; i < cm.nnodes; ++i) {
			ChunkyTriMeshNode node = cm.nodes[i];
			boolean isLeaf = node.i >= 0;
			if (!isLeaf)
				continue;
			if (node.n > cm.maxTrisPerChunk)
				cm.maxTrisPerChunk = node.n;
		}

		return cm;
	}

	private boolean checkOverlapRect(float[] amin, float[] amax, float[] bmin, float[] bmax) {
		boolean overlap = true;
		overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
		overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
		return overlap;
	}
}
