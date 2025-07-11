import { MockFlowNode } from '../nodes/mock_flow_node'; // adjust path as needed
import type { Edge } from "@xyflow/react";

function checkSameDataType(
    sourceNode: MockFlowNode,
    targetNode: MockFlowNode,
    sourceHandle: string | null | undefined,
    targetHandle: string | null | undefined
): boolean {
    if (
        !(sourceNode as any).data?.config ||
        !(targetNode as any).data?.config
    ) {
        return false;
    }

    const sourceConfig = sourceNode.data.config;
    const targetConfig = targetNode.data.config;

    const sourceMatch = sourceConfig.output.find((o) => o.id === sourceHandle);
    const targetMatch = targetConfig.input.find((i) => i.id === targetHandle);

    return sourceMatch?.data_type === targetMatch?.data_type;
}

function AlreadyConnectedEdge(
    edges: any[],
    targetNodeId: string,
    targetHandle: string | null
): Edge | undefined {

    return edges.find(
        (edge) =>
            edge.target === targetNodeId &&
            edge.targetHandle === targetHandle
    );
}

function getNewEdges(edges: Edge[], edge: Edge | undefined): Edge[] {
    if (!edge) {
        return edges;
    }
    return edges.filter(edge => edge.id !== edge.id);
}

export function updateEdgesIfnecessary(
    edges: any[],
    targetNode: any,
    targetHandle: string | null
): Edge[] {
    let already_connected_edge = AlreadyConnectedEdge(
        edges,
        targetNode.id,
        targetHandle
    );

    let new_edges = getNewEdges(edges, already_connected_edge);
    return new_edges;
}



export function isValidMockFlowConnection(
    edges: any[],
    sourceNode: any,
    targetNode: any,
    sourceHandle: string | null,
    targetHandle: string | null
): boolean | null {
    if (
        !sourceNode ||
        !targetNode ||
        !sourceHandle ||
        !targetHandle
    ) {
        return false;
    }

    let isValid = checkSameDataType(
        sourceNode,
        targetNode,
        sourceHandle,
        targetHandle
    );

    return isValid;
}