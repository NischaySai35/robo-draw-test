/** Mount-once viewport for draw-to-build mode. Same lifecycle contract as `ModelEditor`. */
import { useEffect, useRef } from 'react';
import { DrawModeController } from './DrawModeController';

export function DrawModeEditor() {
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const controller = new DrawModeController(container);
    return () => controller.dispose();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <div ref={containerRef} className="viewport" />;
}
