package org.recast4j.detour;

import java.util.List;

//TODO: (PP) Add comments
public class FindLocalNeighbourhoodResult {
	private final List<Long> refs;
	private final List<Long> parentRefs;

	public FindLocalNeighbourhoodResult(List<Long> refs, List<Long> parentRefs) {
		this.refs = refs;
		this.parentRefs = parentRefs;
	}

	public List<Long> getRefs() {
		return refs;
	}

	public List<Long> getParentRefs() {
		return parentRefs;
	}

	
}