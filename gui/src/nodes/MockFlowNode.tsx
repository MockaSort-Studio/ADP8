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

function generateIOHandles(io_data, idx, type, n_handles) {
    return (
        <Handle
            key={io_data.id}
            type={type}
            position={type === "source" ? Position.Right : Position.Left}
            id={io_data.id}
            style={{ top: generateEvenHandlesPositioning(idx, n_handles) }}
        >
            <div className="handle-label">{io_data.name}</div>
        </Handle>
    );
}

export function MockFlowNode({
    data,
    selected
}: NodeProps<MockFlowNode>) {

    const config: MockFlowNodeConfig = data.config;

    const inputHandles = config.input.map((input, idx) =>
        generateIOHandles(input, idx, "target", config.input.length)
    );

    const outputHandles = config.output.map((output, idx) =>
        generateIOHandles(output, idx, "source", config.output.length)
    );

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