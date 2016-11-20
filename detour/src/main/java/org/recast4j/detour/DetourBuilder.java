package org.recast4j.detour;

public class DetourBuilder {

	public MeshData build(NavMeshDataCreateParams params, int tileX,int tileY) {
		MeshData data = NavMeshBuilder.createNavMeshData(params);
		if (data != null) {
			data.header.x = tileX;
			data.header.y = tileY;
		}
		return data;
	}
}
