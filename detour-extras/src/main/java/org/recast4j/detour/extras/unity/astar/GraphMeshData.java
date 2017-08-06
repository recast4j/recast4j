package org.recast4j.detour.extras.unity.astar;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.Poly;

class GraphMeshData {

	final int tileXCount;
	final int tileZCount;

	final MeshData[] tiles;

	GraphMeshData(int tileXCount, int tileZCount, MeshData[] tiles) {
		this.tileXCount = tileXCount;
		this.tileZCount = tileZCount;
		this.tiles = tiles;
	}

	int countNodes() {
		int polyCount = 0;
		for (MeshData t : tiles) {
			polyCount += t.header.polyCount;
		}
		return polyCount;
	}

	public Poly getNode(int node) {
		int index = 0;
		for (MeshData t : tiles) {
			if (node - index >= 0 && node - index < t.header.polyCount) {
				return t.polys[node - index];
			}
			index += t.header.polyCount;
		}
		return null;
	}

	public MeshData getTile(int node) {
		int index = 0;
		for (MeshData t : tiles) {
			if (node - index >= 0 && node - index < t.header.polyCount) {
				return t;
			}
			index += t.header.polyCount;
		}
		return null;
	}

}
