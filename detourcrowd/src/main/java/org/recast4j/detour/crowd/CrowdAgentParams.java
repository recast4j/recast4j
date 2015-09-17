package org.recast4j.detour.crowd;

/// Configuration parameters for a crowd agent.
/// @ingroup crowd
public class CrowdAgentParams
{
	float radius;						///< Agent radius. [Limit: >= 0]
	float height;						///< Agent height. [Limit: > 0]
	float maxAcceleration;				///< Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;						///< Maximum allowed speed. [Limit: >= 0]

	/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	float collisionQueryRange;

	float pathOptimizationRange;		///< The path visibility optimization range. [Limit: > 0]

	/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	float separationWeight;

	/// Flags that impact steering behavior. (See: #UpdateFlags)
	int updateFlags;

	/// The index of the avoidance configuration to use for the agent. 
	/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	int obstacleAvoidanceType;	

	/// The index of the query filter used by this agent.
	int queryFilterType;

	/// User defined data attached to the agent.
	Object userData;
}