/** Mount-once viewport for the cube-builder mode. Same lifecycle contract as `ModelEditor`. */
import { useEffect, useRef } from 'react';
import { VoxelModeController } from './VoxelModeController';

export function VoxelModeEditor() {
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const controller = new VoxelModeController(container);
    return () => controller.dispose();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <div ref={containerRef} className="viewport" />;
}
