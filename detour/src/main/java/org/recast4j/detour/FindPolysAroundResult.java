package org.recast4j.detour;

import java.util.List;

// TODO: (PP) Add comments
public class FindPolysAroundResult {
	private final List<Long> refs;
	private final List<Long> parentRefs;
	private final List<Float> costs;

	public FindPolysAroundResult(List<Long> refs, List<Long> parentRefs, List<Float> costs) {
		this.refs = refs;
		this.parentRefs = parentRefs;
		this.costs = costs;
	}

	public List<Long> getRefs() {
		return refs;
	}

	public List<Long> getParentRefs() {
		return parentRefs;
	}

	public List<Float> getCosts() {
		return costs;
	}

}