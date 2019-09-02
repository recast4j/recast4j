package org.recast4j.demo.tool.jumplink;

public class AnnotationBuilderConfig {

    final float cellSize;
    final float cellHeight;
    final float agentRadius;
    final float groundTolerance;
    final float agentHeight;
    final float startDistance;
    final float endDistance;
    final float averageDownDistance;
    final float groundRange;
    final float jumpHeight;

    public AnnotationBuilderConfig(float cellSize, float cellHeight, float agentRadius, float agentClimb,
            float agentHeight, float startDistance, float endDistance, float maxDownDistance, float minDownDistance,
            float jumpHeight) {
        this.cellSize = cellSize;
        this.cellHeight = cellHeight;
        this.agentRadius = agentRadius;
        groundTolerance = agentClimb;
        this.agentHeight = agentHeight;
        this.startDistance = startDistance;
        this.endDistance = endDistance;
        averageDownDistance = -(minDownDistance + maxDownDistance) / 2;
        groundRange = Math.abs(maxDownDistance - minDownDistance) / 2;
        this.jumpHeight = jumpHeight;
    }

}
