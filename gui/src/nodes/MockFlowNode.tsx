import { memo } from 'react';
import { Node, Handle, Position, NodeResizer, type NodeProps } from '@xyflow/react';
import { MockFlowNodeConfig } from './MockFlowNodeConfig'

export type MockFlowNode = Node<
    {
        config: MockFlowNodeConfig;
    },
    "mock-flow"
>;


export function MockFlowNode({
    data,
    selected
}: NodeProps<MockFlowNode>) {

    const config: MockFlowNodeConfig = data.config;

    // Map input handles
    const inputHandles = config.input.map(input => (
        <Handle
            key={input.id}
            type="target"
            position={Position.Left}
            id={input.id}
        />
    ));

    // Map output handles
    const outputHandles = config.output.map(output => (
        <Handle
            key={output.id}
            type="source"
            position={Position.Right}
            id={output.id}
        />
    ));

    return (
        <>
            <NodeResizer
                color="#ff0071"
                isVisible={selected}
                minWidth={100}
                minHeight={30}
            />
            {inputHandles}
            <div style={{ padding: 10 }}>{data.config.name}</div>
            {outputHandles}
        </>
    );
}