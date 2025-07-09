import { MockFlowNode } from '../nodes/mock_flow_node'; // adjust path as needed

export function isValidMockFlowConnection(
    sourceNode: any,
    targetNode: any,
    sourceHandle: string | null | undefined,
    targetHandle: string | null | undefined
): boolean {
    if (
        !sourceNode ||
        !targetNode ||
        !sourceHandle ||
        !targetHandle ||
        !(sourceNode as any).data?.config ||
        !(targetNode as any).data?.config
    ) {
        return false;
    }

    const sourceConfig = (sourceNode as MockFlowNode).data.config;
    const targetConfig = (targetNode as MockFlowNode).data.config;

    const sourceMatch = sourceConfig.output.find((o) => o.id === sourceHandle);
    const targetMatch = targetConfig.input.find((i) => i.id === targetHandle);

    return sourceMatch?.data_type === targetMatch?.data_type;
}