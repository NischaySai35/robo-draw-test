/**
 * Mount-once 3D viewport. Creates the `EditModeController` exactly once per
 * mount and never hot-patches it from React re-renders -- the controller
 * subscribes to the Zustand stores itself and updates the live Three.js
 * scene imperatively. This component's only job is DOM/lifecycle plumbing.
 */
import { useEffect, useRef } from 'react';
import { EditModeController } from './EditModeController';

export function ModelEditor() {
  const containerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const controller = new EditModeController(container);
    return () => controller.dispose();
    // Intentionally empty deps: mount once, full reload during dev is expected
    // per project convention rather than HMR-patching a live Three.js scene.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <div ref={containerRef} className="viewport" />;
}
