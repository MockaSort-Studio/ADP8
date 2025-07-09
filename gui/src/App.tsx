import { useCallback } from "react";
import {
    Background,
    Controls,
    MiniMap,
    ReactFlow,
    addEdge,
    useNodesState,
    useEdgesState,
    type OnConnect,
} from "@xyflow/react";

import { isValidMockFlowConnection } from "./utils/connection_utils";

import "@xyflow/react/dist/style.css";

import { initialNodes, nodeTypes, type AppNode } from "./nodes";
import { initialEdges, edgeTypes } from "./edges";

export default function App() {
    const [nodes, , onNodesChange] = useNodesState<AppNode>(initialNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
    const onConnect: OnConnect = useCallback(
        (connection) => setEdges((edges) => addEdge(connection, edges)),
        [setEdges]
    );

    return (
        <ReactFlow
            nodes={nodes}
            nodeTypes={nodeTypes}
            onNodesChange={onNodesChange}
            edges={edges}
            edgeTypes={edgeTypes}
            onEdgesChange={onEdgesChange}
            onConnect={onConnect}
            isValidConnection={(connection) => {
                const sourceNode = nodes.find((node) => node.id === connection.source);
                const targetNode = nodes.find((node) => node.id === connection.target);

                return isValidMockFlowConnection(
                    sourceNode,
                    targetNode,
                    connection.sourceHandle,
                    connection.targetHandle
                );
            }}
            fitView
        >
            <Background />
            <MiniMap />
            <Controls />
        </ReactFlow>
    );
}
