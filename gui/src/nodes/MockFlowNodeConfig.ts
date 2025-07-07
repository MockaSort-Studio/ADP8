export class IODataConfig {
    name: string;
    id: string;
    constructor(name: string, id: string) {
        this.name = name;
        this.id = id;
    }
}

export class MockFlowNodeConfig {
    name: string;
    id: string;
    input: IODataConfig[];
    output: IODataConfig[];
    parameters: any[];

    constructor(
        name: string,
        id: string,
        input: IODataConfig[] = [],
        output: IODataConfig[] = [],
        parameters: any[] = []
    ) {
        this.name = name;
        this.id = id;
        this.input = input;
        this.output = output;
        this.parameters = parameters;
    }
}