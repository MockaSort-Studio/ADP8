import { memo } from 'react';
import { Node, Handle, Position, NodeResizer, type NodeProps } from '@xyflow/react';

export type MockFlowNode = Node<
    {
        label?: string;
    },
    "mock-flow"
>;


export function MockFlowNode({
    data,
    selected
}: NodeProps<MockFlowNode>) {
    return (
        <>
            <NodeResizer
                color="#ff0071"
                isVisible={selected}
                minWidth={100}
                minHeight={30}
            />
            <Handle type="target" position={Position.Left} />
            <div style={{ padding: 10 }}>{data.label}</div>
            <Handle type="source" position={Position.Right} />
        </>
    );
}