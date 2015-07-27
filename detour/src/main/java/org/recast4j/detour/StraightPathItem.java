package org.recast4j.detour;

import static org.recast4j.detour.DetourCommon.vCopy;

//TODO: (PP) Add comments
public class StraightPathItem {
	float[] pos;
	int flags;
	long ref;
	public StraightPathItem(float[] pos, int flags, long ref) {
		this.pos = vCopy(pos);
		this.flags = flags;
		this.ref = ref;
	}
	public float[] getPos() {
		return pos;
	}
	public int getFlags() {
		return flags;
	}
	public long getRef() {
		return ref;
	}
	
}