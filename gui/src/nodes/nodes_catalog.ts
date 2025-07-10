import { IODataConfig, MockFlowNodeConfig } from './mock_flow_node_config';

type NodesCatalogMap = Record<string, MockFlowNodeConfig>;


export const nodesCatalog: NodesCatalogMap = {
    "float_source": new MockFlowNodeConfig(
        "Float Source",
        "float_source",
        [],
        [new IODataConfig("Output", "float_source_output", "float")],
        []
    ),
    "int_source": new MockFlowNodeConfig(
        "Int Source",
        "int_source",
        [],
        [new IODataConfig("Output", "int_source_output", "int")],
        []
    ),
    "integrator": new MockFlowNodeConfig(
        "Integrator",
        "integrator",
        [new IODataConfig("Input", "integrator_input", "float")],
        [new IODataConfig("Output", "integrator_output", "float")],
        []
    )
}