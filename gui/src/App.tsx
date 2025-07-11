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

let id = 0;
const getId = () => `dndnode_${id++}`;

function AppInternal() {
    const [nodes, onNodesChange, setNodes] = useNodesState<AppNode>(initialNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
    const [type] = useDnD();
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
        [setEdges]
    );
    /*
        const onDragOver = useCallback((event) => {
            event.preventDefault();
            event.dataTransfer.dropEffect = 'move';
            console.log(`Drag over event: ${event.clientX}, ${event.clientY}`);
        }, []);
    
        const onDrop = useCallback(
            (event) => {
                event.preventDefault();
    
                // check if the dropped element is valid
                if (!type) {
                    return;
                }
    
                // project was renamed to screenToFlowPosition
                // and you don't need to subtract the reactFlowBounds.left/top anymore
                // details: https://reactflow.dev/whats-new/2023-11-10
                const position = screenToFlowPosition({
                    x: event.clientX,
                    y: event.clientY,
                });
                const newNode = {
                    id: getId(),
                    type,
                    position,
                    data: { label: `${type} node` },
                };
    
                setNodes((nds) => nds.concat(newNode));
                console.log(`Dropped node type: ${type}`);
            },
            [screenToFlowPosition, type],
        );
    
        const onDragStart = (event, nodeType) => {
            setType(nodeType);
            event.dataTransfer.setData('text/plain', nodeType);
            event.dataTransfer.effectAllowed = 'move';
            console.log(`Dragging node type: ${nodeType}`);
        };
    */
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
                    //onDragOver={onDragOver}
                    //onDrop={onDrop}
                    //onDragStart={onDragStart}
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