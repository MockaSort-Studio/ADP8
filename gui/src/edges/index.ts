import { MarkerType } from "@xyflow/react";
import type { Edge, EdgeTypes } from "@xyflow/react";

export const initialEdges = [
    { id: "a->c", source: "a", target: "c", animated: true },
    { id: "b->d", source: "b", target: "d" },
    { id: "c->d", source: "c", target: "d", animated: true },
    {
        id: "e->f_x", source: "e", target: "f", sourceHandle: "x", targetHandle: "x", type: 'smoothstep', markerEnd: {
            type: MarkerType.ArrowClosed,
        },
    },
    {
        id: "e->f_y", source: "e", target: "f", sourceHandle: "y", targetHandle: "y", type: 'smoothstep', markerEnd: {
            type: MarkerType.ArrowClosed,
        },
    },
] satisfies Edge[];

export const edgeTypes = {
    // Add your custom edge types here!
} satisfies EdgeTypes;
