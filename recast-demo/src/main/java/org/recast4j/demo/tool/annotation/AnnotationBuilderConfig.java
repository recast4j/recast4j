package org.recast4j.demo.tool.annotation;

public class AnnotationBuilderConfig {

    final float cellSize;
    final float agentRadius;
    final float agentClimb;
    final float agentHeight;
    final float startDistance;
    final float endDistance;
    final float jumpDownDistance;
    final float groundRange;

    public AnnotationBuilderConfig(float cellSize, float agentRadius, float agentClimb, float agentHeight,
            float startDistance, float endDistance, float jumpDownDistance, float groundRange) {
        this.cellSize = cellSize;
        this.agentRadius = agentRadius;
        this.agentClimb = agentClimb;
        this.agentHeight = agentHeight;
        this.startDistance = startDistance;
        this.endDistance = endDistance;
        this.jumpDownDistance = -jumpDownDistance;
        this.groundRange = groundRange;
    }

}
