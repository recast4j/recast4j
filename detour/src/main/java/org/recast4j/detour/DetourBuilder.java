package org.recast4j.detour;

public class DetourBuilder {

	public MeshData build(NavMeshCreateParams params, int tileX,int tileY) {
		MeshData data = NavMeshBuilder.createNavMeshData(params);
		data.header.x = tileX;
		data.header.y = tileY;
		return data;
	}
}
