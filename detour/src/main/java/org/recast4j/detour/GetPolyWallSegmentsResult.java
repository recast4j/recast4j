package org.recast4j.detour;

import java.util.List;

public class GetPolyWallSegmentsResult {

	private final List<float[]> segmentVerts;
	private final List<Long> segmentRefs;

	public GetPolyWallSegmentsResult(List<float[]> segmentVerts, List<Long> segmentRefs) {
		this.segmentVerts = segmentVerts;
		this.segmentRefs = segmentRefs;
	}

	public List<float[]> getSegmentVerts() {
		return segmentVerts;
	}

	public List<Long> getSegmentRefs() {
		return segmentRefs;
	}

}
