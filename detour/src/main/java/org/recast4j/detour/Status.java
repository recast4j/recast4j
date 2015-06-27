/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour;

public enum Status {
	
	FAILURE,
	SUCCSESS,
	IN_PROGRESS, INVALID_PARAM, PARTIAL_RESULT;
	
	/*
	private int status;
	
	//High level status.
	static int DT_FAILURE = 1 << 31;			// Operation failed.
	static int DT_SUCCESS = 1 << 30;			// Operation succeed.
	static int DT_IN_PROGRESS = 1 << 29;		// Operation still in progress.

	//Detail information for status.
	static int DT_STATUS_DETAIL_MASK = 0x0ffffff;
	static int DT_WRONG_MAGIC = 1 << 0;		// Input data is not recognized.
	static int DT_WRONG_VERSION = 1 << 1;	// Input data is in wrong version.
	static int DT_OUT_OF_MEMORY = 1 << 2;	// Operation ran out of memory.
	static int DT_INVALID_PARAM = 1 << 3;	// An input parameter was invalid.
	static int DT_BUFFER_TOO_SMALL = 1 << 4;	// Result buffer for the query was too small to store all results.
	static int DT_OUT_OF_NODES = 1 << 5;		// Query ran out of nodes during search.
	static int DT_PARTIAL_RESULT = 1 << 6;	// Query did not reach the end location, returning best guess. 

	public Status(int status) {
		this.status = status;
	}

	//Returns true of status is success.
	public boolean dtStatusSucceed()
	{
		return (status & DT_SUCCESS) != 0;
	}

	//Returns true of status is failure.
	public boolean dtStatusFailed()
	{
		return (status & DT_FAILURE) != 0;
	}

	//Returns true of status is in progress.
	public boolean dtStatusInProgress()
	{
		return (status & DT_IN_PROGRESS) != 0;
	}

	//Returns true if specific detail is set.
	public boolean dtStatusDetail(int detail)
	{
		return (status & detail) != 0;
	}
*/
	public boolean isFailed() {
		return this == FAILURE || this == Status.INVALID_PARAM;
	}
}

