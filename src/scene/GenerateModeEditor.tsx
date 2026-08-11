/** Mount-once viewport for the prompt-to-build tab. Same lifecycle contract as `ModelEditor`. */
import { useEffect, useRef } from 'react';
import { GenerateModeController } from './GenerateModeController';

export function GenerateModeEditor() {
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const controller = new GenerateModeController(container);
    return () => controller.dispose();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <div ref={containerRef} className="viewport" />;
}
