import { useCallback } from "react";
import { useDnD } from './sidebar/drag_and_drop_context';
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
    useReactFlow,
} from "@xyflow/react";

import { isValidMockFlowConnection, updateEdgesIfnecessary } from "./utils/connection_utils";
import { Sidebar } from "./sidebar/sidebar"
import { DnDProvider } from "./sidebar/drag_and_drop_context"

import "@xyflow/react/dist/style.css";

import { initialNodes, nodeTypes, type AppNode } from "./nodes";
import { initialEdges, edgeTypes } from "./edges";
import { nodesCatalog } from "./nodes/nodes_catalog";
import { mockFlowNodeType } from "./nodes/mock_flow_node"

let id = 0;
const getId = () => `dndnode_${id++}`;

function AppInternal() {
    const [nodes, setNodes, onNodesChange] = useNodesState<AppNode>(initialNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
    const [type, setType] = useDnD();
    const { screenToFlowPosition } = useReactFlow();
    const onConnect: OnConnect = useCallback(
        (connection) => {
            const targetNode = nodes.find((node) => node.id === connection.target);
            const targetHandle = connection.targetHandle;
            const new_edges = updateEdgesIfnecessary(edges, targetNode, targetHandle);
            setEdges((eds) => updateEdgesIfnecessary(eds, targetNode, targetHandle));
            setEdges((eds) => addEdge(connection, eds));
            console.log(`Connected: ${connection.source} to ${connection.target} with handle ${targetHandle}`);
        },
        [nodes, edges, setEdges]
    );

    const onDragOver = useCallback((event) => {
        event.preventDefault();
        event.dataTransfer.dropEffect = 'move';
    }, []);

    const onDrop = useCallback(
        (event) => {
            event.preventDefault();

            if (!type) {
                return;
            }

            const position = screenToFlowPosition({
                x: event.clientX,
                y: event.clientY,
            });

            let nodeConfig = nodesCatalog[type];
            if (!nodeConfig) {
                return;
            }
            let id = getId();
            console.log(`Creating node with id: ${id} and type: ${type}`);
            const newNode = {
                id: id,
                type: mockFlowNodeType,
                position,
                data: { config: nodeConfig },
            };

            setNodes((nds) => nds.concat(newNode));
            console.log(`Dropped node type: ${type}`);
        },
        [screenToFlowPosition, type],
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
                    onDragOver={onDragOver}
                    onDrop={onDrop}
                    deleteKeyCode={['Backspace', 'Delete']}
                    isValidConnection={(connection) => {
                        const sourceNode = nodes.find((node) => node.id === connection.source);
                        const targetNode = nodes.find((node) => node.id === connection.target);

                        return isValidMockFlowConnection(
                            edges,
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