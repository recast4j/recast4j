package org.recast4j.detour;

import java.util.List;

public class FindPathResult {
	private final Status status;
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	private final List<Long> refs;

	public FindPathResult(Status status, List<Long> refs) {
		this.status = status;
		this.refs = refs;
	}

	public Status getStatus() {
		return status;
	}

	public List<Long> getRefs() {
		return refs;
	}

}