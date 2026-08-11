/** Inline warning/error toasts -- the app's "clear inline warning" surface for illegal actions. */
import { useUIStore } from '../state/uiStore';

export function ToastList() {
  const warnings = useUIStore((s) => s.warnings);
  const dismissWarning = useUIStore((s) => s.dismissWarning);

  if (warnings.length === 0) return null;

  return (
    <div className="toast-list">
      {warnings.map((warning) => (
        <div key={warning.id} className={`toast toast--${warning.level}`}>
          <span>{warning.message}</span>
          <button className="toast__dismiss" onClick={() => dismissWarning(warning.id)} aria-label="Dismiss">
            ×
          </button>
        </div>
      ))}
    </div>
  );
}
