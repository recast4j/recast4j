package org.recast4j.detour.extras.unity.astar;

import static org.recast4j.detour.DetourCommon.clamp;
import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vMax;
import static org.recast4j.detour.DetourCommon.vMin;

import org.recast4j.detour.BVNode;
import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshBuilder.BVItem;

public class BVTreeBuilder {

	void build(GraphMeshData graphData, GraphMeta meta) {
		for (MeshData d : graphData.tiles) {
			d.bvTree = new BVNode[d.header.polyCount * 2];
			d.header.bvNodeCount = createBVTree(d, d.bvTree, meta.cellSize);
		}
	}

	private static int createBVTree(MeshData data, BVNode[] nodes, float cs) {
		float quantFactor = 1 / cs;
		BVItem[] items = new BVItem[data.header.polyCount];
		for (int i = 0; i < data.header.polyCount; i++) {
			BVItem it = new BVItem();
			items[i] = it;
			it.i = i;
			float[] bmin = new float[3];
			float[] bmax = new float[3];
			vCopy(bmin, data.verts, data.polys[i].verts[0] * 3);
			vCopy(bmax, data.verts, data.polys[i].verts[0] * 3);
			for (int j = 1; j < data.polys[i].vertCount; j++) {
				vMin(bmin, data.verts, data.polys[i].verts[j] * 3);
				vMax(bmax, data.verts, data.polys[i].verts[j] * 3);
			}
			it.bmin[0] = clamp((int) ((bmin[0] - data.header.bmin[0]) * quantFactor), 0, 0xffff);
			it.bmin[1] = clamp((int) ((bmin[1] - data.header.bmin[1]) * quantFactor), 0, 0xffff);
			it.bmin[2] = clamp((int) ((bmin[2] - data.header.bmin[2]) * quantFactor), 0, 0xffff);
			it.bmax[0] = clamp((int) ((bmax[0] - data.header.bmin[0]) * quantFactor), 0, 0xffff);
			it.bmax[1] = clamp((int) ((bmax[1] - data.header.bmin[1]) * quantFactor), 0, 0xffff);
			it.bmax[2] = clamp((int) ((bmax[2] - data.header.bmin[2]) * quantFactor), 0, 0xffff);
		}
		return NavMeshBuilder.subdivide(items, data.header.polyCount, 0, data.header.polyCount, 0, nodes);
	}

}
