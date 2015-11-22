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
package org.recast4j.detour.crowd;

import static org.recast4j.detour.DetourCommon.vAdd;
import static org.recast4j.detour.DetourCommon.vCopy;
import static org.recast4j.detour.DetourCommon.vDist2D;
import static org.recast4j.detour.DetourCommon.vDist2DSqr;
import static org.recast4j.detour.DetourCommon.vLen;
import static org.recast4j.detour.DetourCommon.vMad;
import static org.recast4j.detour.DetourCommon.vNormalize;
import static org.recast4j.detour.DetourCommon.vScale;
import static org.recast4j.detour.DetourCommon.vSet;
import static org.recast4j.detour.DetourCommon.vSub;

import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.VectorPtr;
import org.recast4j.detour.crowd.Crowd.CrowdNeighbour;

/// Represents an agent managed by a #dtCrowd object.
/// @ingroup crowd
class CrowdAgent {

	/// The type of navigation mesh polygon the agent is currently traversing.
	/// @ingroup crowd
	public enum CrowdAgentState
	{
		DT_CROWDAGENT_STATE_INVALID,		///< The agent is not in a valid state.
		DT_CROWDAGENT_STATE_WALKING,		///< The agent is traversing a normal navigation mesh polygon.
		DT_CROWDAGENT_STATE_OFFMESH, 		///< The agent is traversing an off-mesh connection.
	};
	
	enum MoveRequestState
	{
		DT_CROWDAGENT_TARGET_NONE,
		DT_CROWDAGENT_TARGET_FAILED,
		DT_CROWDAGENT_TARGET_VALID,
		DT_CROWDAGENT_TARGET_REQUESTING,
		DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
		DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
		DT_CROWDAGENT_TARGET_VELOCITY,
	};
	
	/// Crowd agent update flags.
	static final int DT_CROWD_ANTICIPATE_TURNS = 1;
	static final int DT_CROWD_OBSTACLE_AVOIDANCE = 2;
	static final int DT_CROWD_SEPARATION = 4;
	static final int DT_CROWD_OPTIMIZE_VIS = 8;			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	static final int DT_CROWD_OPTIMIZE_TOPO = 16;		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
	
	/// True if the agent is active, false if the agent is in an unused slot in the agent pool.
	boolean active;

	/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	CrowdAgentState state;

	/// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	boolean partial;

	/// The path corridor the agent is using.
	PathCorridor corridor;

	/// The local boundary data for the agent.
	LocalBoundary boundary;

	/// Time since the agent's path corridor was optimized.
	float topologyOptTime;

	/// The known neighbors of the agent.
	CrowdNeighbour[] neis = new CrowdNeighbour[Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS];

	/// The number of neighbors.
	int nneis;

	/// The desired speed.
	float desiredSpeed;

	float[] npos = new float[3]; ///< The current agent position. [(x, y, z)]
	float[] disp = new float[3];
	float[] dvel = new float[3]; ///< The desired velocity of the agent. [(x, y, z)]
	float[] nvel = new float[3];
	float[] vel = new float[3]; ///< The actual velocity of the agent. [(x, y, z)]

	/// The agent's configuration parameters.
	CrowdAgentParams params;

	/// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	float[] cornerVerts = new float[Crowd.DT_CROWDAGENT_MAX_CORNERS * 3];

	/// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	int[] cornerFlags = new int[Crowd.DT_CROWDAGENT_MAX_CORNERS];

	/// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	long[] cornerPolys = new long[Crowd.DT_CROWDAGENT_MAX_CORNERS];

	/// The number of corners.
	int ncorners;

	MoveRequestState targetState; ///< State of the movement request.
	long targetRef; ///< Target polyref of the movement request.
	float[] targetPos = new float[3]; ///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	long targetPathqRef; ///< Path finder ref.
	boolean targetReplan; ///< Flag indicating that the current path is being replanned.
	float targetReplanTime; /// <Time since the agent's target was replanned.

	public CrowdAgent() {
		corridor = new PathCorridor();
		boundary = new LocalBoundary();
	}
	
	void integrate(float dt) {
		// Fake dynamic constraint.
		float maxDelta = params.maxAcceleration * dt;
		float[] dv = vSub(nvel, vel);
		float ds = vLen(dv);
		if (ds > maxDelta)
			dv = vScale(dv, maxDelta / ds);
		vel = vAdd(vel, dv);

		// Integrate
		if (vLen(vel) > 0.0001f)
			npos = vMad(npos, vel, dt);
		else
			vSet(vel, 0, 0, 0);
	}

	boolean overOffmeshConnection(float radius) {
		if (ncorners == 0)
			return false;

		boolean offMeshConnection = ((cornerFlags[ncorners - 1] & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0)
				? true : false;
		if (offMeshConnection) {
			float distSq = vDist2DSqr(new VectorPtr(npos), new VectorPtr(cornerVerts, (ncorners - 1) * 3));
			if (distSq < radius * radius)
				return true;
		}

		return false;
	}

	float getDistanceToGoal(float range) {
		if (ncorners == 0)
			return range;

		boolean endOfPath = ((cornerFlags[ncorners - 1] & NavMeshQuery.DT_STRAIGHTPATH_END) != 0) ? true : false;
		if (endOfPath)
			return Math.min(vDist2D(new VectorPtr(npos), new VectorPtr(cornerVerts, (ncorners - 1) * 3)), range);

		return range;
	}

	public float[] calcSmoothSteerDirection() {
		float[] dir = new float[3];
		if (ncorners != 0) {

			int ip0 = 0;
			int ip1 = Math.min(1, ncorners - 1);
			VectorPtr p0 = new VectorPtr(cornerVerts, ip0 * 3);
			VectorPtr p1 = new VectorPtr(cornerVerts, ip1 * 3);
			VectorPtr vnpos = new VectorPtr(npos);

			float[] dir0 = vSub(p0, vnpos);
			float[] dir1 = vSub(p1, vnpos);
			dir0[1] = 0;
			dir1[1] = 0;

			float len0 = vLen(dir0);
			float len1 = vLen(dir1);
			if (len1 > 0.001f)
				dir1 = vScale(dir1, 1.0f / len1);

			dir[0] = dir0[0] - dir1[0] * len0 * 0.5f;
			dir[1] = 0;
			dir[2] = dir0[2] - dir1[2] * len0 * 0.5f;

			vNormalize(dir);
		}
		return dir;
	}

	public float[] calcStraightSteerDirection() {
		float[] dir = new float[3];
		if (ncorners != 0) {
			dir = vSub(cornerVerts, npos);
			dir[1] = 0;
			vNormalize(dir);
		}
		return dir;
	}

	
	void setTarget(long ref, float[] pos) {
		targetRef = ref;
		vCopy(targetPos, pos);
		targetPathqRef = PathQueue.DT_PATHQ_INVALID;
		if (targetRef != 0)
			targetState = MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING;
		else
			targetState = MoveRequestState.DT_CROWDAGENT_TARGET_FAILED;
	}

}