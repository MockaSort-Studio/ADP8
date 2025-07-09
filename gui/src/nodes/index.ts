import type { Node, NodeTypes, BuiltInNode } from "@xyflow/react";
import { MockFlowNode } from "./mock_flow_node";
import { MockFlowNodeConfig, IODataConfig } from "./mock_flow_node_config"; // <-- import

export type AppNode = BuiltInNode | MockFlowNode;

const randomMockFlowConfig = new MockFlowNodeConfig(
    "MockFlow Node 1",
    "mock-1",
    [
        new IODataConfig("InputA", "mock-1-a", "float"),
        new IODataConfig("InputB", "mock-1-b", "int"),
        new IODataConfig("InputC", "mock-1-c", "string"),
    ],
    [
        new IODataConfig("OutputX", "mock-1-x", "float"),
        new IODataConfig("OutputY", "mock-1-y", "int"),
    ],
    [{ param: "foo", value: 42 }]
);
const randomMockFlowConfig2 = new MockFlowNodeConfig(
    "MockFlow Node 2",
    "mock-2",
    [
        new IODataConfig("InputX", "mock-2-x", "float"),
        new IODataConfig("InputY", "mock-2-y", "int"),
    ],
    [],
    [{ param: "foo", value: 42 }]
);


export const initialNodes: AppNode[] = [
    {
        id: "e",
        type: "mock-flow",
        position: { x: -100, y: -100 },
        data: { config: randomMockFlowConfig },
    },
    {
        id: "f",
        type: "mock-flow",
        position: { x: +200, y: -100 },
        data: { config: randomMockFlowConfig2 },
    },
];

export const nodeTypes = {
    "mock-flow": MockFlowNode,
    // Add any of your custom nodes here!
} satisfies NodeTypes;
