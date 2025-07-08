import type { Node, NodeTypes, BuiltInNode } from "@xyflow/react";
import { PositionLoggerNode } from "./PositionLoggerNode";
import { MockFlowNode } from "./MockFlowNode";
import { MockFlowNodeConfig, IODataConfig } from "./MockFlowNodeConfig"; // <-- import

export type AppNode = BuiltInNode | PositionLoggerNode | MockFlowNode;

const randomMockFlowConfig = new MockFlowNodeConfig(
  "MockFlow Node 1",
  "mock-1",
  [
    new IODataConfig("InputA", "a"),
    new IODataConfig("InputB", "b"),
    new IODataConfig("InputC", "c"),
  ],
  [
    new IODataConfig("OutputX", "x"),
    new IODataConfig("OutputY", "y"),
  ],
  [{ param: "foo", value: 42 }]
);
const randomMockFlowConfig2 = new MockFlowNodeConfig(
  "MockFlow Node 2",
  "mock-2",
  [
    new IODataConfig("InputX", "x"),
    new IODataConfig("InputY", "y"),
  ],
  [],
  [{ param: "foo", value: 42 }]
);


export const initialNodes: AppNode[] = [
  { id: "a", type: "input", position: { x: 0, y: 0 }, data: { label: "wire" } },
  {
    id: "b",
    type: "position-logger",
    position: { x: -100, y: 100 },
    data: { label: "drag me!" },
  },
  { id: "c", position: { x: 100, y: 100 }, data: { label: "your ideas" } },
  {
    id: "d",
    type: "output",
    position: { x: 0, y: 200 },
    data: { label: "with React Flow" },
  },
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
  "position-logger": PositionLoggerNode,
  "mock-flow": MockFlowNode,
  // Add any of your custom nodes here!
} satisfies NodeTypes;
