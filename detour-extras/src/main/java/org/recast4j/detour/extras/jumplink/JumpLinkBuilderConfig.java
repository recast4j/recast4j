package org.recast4j.detour.extras.jumplink;

public class JumpLinkBuilderConfig {

    final float cellSize;
    final float cellHeight;
    final float agentClimb;
    final float agentRadius;
    final float groundTolerance;
    final float agentHeight;
    final float startDistance;
    final float endDistance;
    final float jumpHeight;
    final float minHeight;
    final float heightRange;

    public JumpLinkBuilderConfig(float cellSize, float cellHeight, float agentRadius, float agentHeight,
            float agentClimb, float groundTolerance, float startDistance, float endDistance, float minHeight,
            float maxHeight, float jumpHeight) {
        this.cellSize = cellSize;
        this.cellHeight = cellHeight;
        this.agentRadius = agentRadius;
        this.agentClimb = agentClimb;
        this.groundTolerance = groundTolerance;
        this.agentHeight = agentHeight;
        this.startDistance = startDistance;
        this.endDistance = endDistance;
        this.minHeight = minHeight;
        heightRange = maxHeight - minHeight;
        this.jumpHeight = jumpHeight;
    }

}
