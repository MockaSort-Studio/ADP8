import React from 'react';
import { useDnD } from './drag_and_drop_context';

export function Sidebar() {
    const [_, setType] = useDnD();

    const onDragStart = (event, nodeType) => {
        console.log(`Dragging node type: ${nodeType}`);
        setType(nodeType);
        event.dataTransfer.effectAllowed = 'move';
    };

    return (
        <aside>
            <div className="description">You can drag these nodes to the pane on the right.</div>
            <div className="dndnode input" onDragStart={(event) => onDragStart(event, 'float_source')} draggable>
                Float Source
            </div>
            <div className="dndnode" onDragStart={(event) => onDragStart(event, 'int_source')} draggable>
                Int Source
            </div>
            <div className="dndnode output" onDragStart={(event) => onDragStart(event, 'integrator')} draggable>
                Integrator
            </div>
        </aside>
    );
}
