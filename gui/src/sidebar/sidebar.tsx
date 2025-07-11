import React from 'react';
import { useDnD } from './drag_and_drop_context';
import { nodesCatalog } from '../nodes/nodes_catalog';

export function Sidebar() {
    const [_, setType] = useDnD();

    const onDragStart = (event: React.DragEvent<HTMLDivElement>, nodeType: string) => {
        console.log(`Dragging node type: ${nodeType}`);
        setType(nodeType);
        event.dataTransfer.effectAllowed = 'move';
    };

    // Dynamically generate catalog nodes
    const catalogNodes = Object.entries(nodesCatalog).map(([key, nodeConfig]) => (
        <div
            key={key}
            className="dndnode"
            onDragStart={(event) => onDragStart(event, key)}
            draggable
        >
            {nodeConfig.name}
        </div>
    ));

    return (
        <aside>
            <div className="description">You can drag these nodes to the pane on the right.</div>
            {catalogNodes}
        </aside>
    );
}