import { useCallback } from "react";
import {
    Background,
    Controls,
    MiniMap,
    ReactFlow,
    ReactFlowProvider,
    addEdge,
    useNodesState,
    useEdgesState,
    type OnConnect,
} from "@xyflow/react";

import { isValidMockFlowConnection } from "./utils/connection_utils";
import { Sidebar } from "./sidebar/sidebar"
import { DnDProvider } from "./sidebar/drag_and_drop_context"

import "@xyflow/react/dist/style.css";

import { initialNodes, nodeTypes, type AppNode } from "./nodes";
import { initialEdges, edgeTypes } from "./edges";

function AppInternal() {
    const [nodes, , onNodesChange] = useNodesState<AppNode>(initialNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
    const onConnect: OnConnect = useCallback(
        (connection) => setEdges((edges) => addEdge(connection, edges)),
        [setEdges]
    );

    return (
        <div className="dndflow">
            <div className="reactflow-wrapper" >
                <ReactFlow
                    nodes={nodes}
                    nodeTypes={nodeTypes}
                    onNodesChange={onNodesChange}
                    edges={edges}
                    edgeTypes={edgeTypes}
                    onEdgesChange={onEdgesChange}
                    onConnect={onConnect}
                    deleteKeyCode={['Backspace', 'Delete']}
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

            </div>

            <Sidebar />
        </div>
    );
}

function ProviderWrapper({ children }: { children: React.ReactNode }) {
    return (
        <ReactFlowProvider>
            <DnDProvider>
                {children}
            </DnDProvider>
        </ReactFlowProvider>
    );
}

export default () => (
    <ProviderWrapper>
        <AppInternal />
    </ProviderWrapper>
);