import { memo } from 'react';
import { Node, Handle, Position, NodeResizer, type NodeProps } from '@xyflow/react';
import { MockFlowNodeConfig } from './MockFlowNodeConfig'

export type MockFlowNode = Node<
    {
        config: MockFlowNodeConfig;
    },
    "mock-flow"
>;

function getHandleBounds(n: number): [number, number] {
    // improve handles positioning spacing based on the number of handles
    const f = 35 * Math.tanh((n - 1) / 2);
    return [50 - f, 50 + f];
}

function generateEvenHandlesPositioning(idx: number, total: number): string {
    const [min_position, max_position] = getHandleBounds(total);
    const step = (max_position - min_position) / (total - 1);
    return `${min_position + idx * step}%`; // Evenly distribute handles
}

export function MockFlowNode({
    data,
    selected
}: NodeProps<MockFlowNode>) {

    const config: MockFlowNodeConfig = data.config;

    // Map input handles
    const inputHandles = config.input.map((input, idx) => (
        <Handle
            key={input.id}
            type="target"
            position={Position.Left}
            id={input.id}
            style={{ top: generateEvenHandlesPositioning(idx, config.input.length) }}
        >
            <div className="handle-label">{input.name}</div>
        </Handle>
    ));

    // Map output handles
    const outputHandles = config.output.map((output, idx) => (
        <Handle
            key={output.id}
            type="source"
            position={Position.Right}
            id={output.id}
            style={{ top: generateEvenHandlesPositioning(idx, config.output.length) }}
        >
            <div className="handle-label">{output.name}</div>
        </Handle>
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