/**
 * Modal choice dialog used for every destructive/ambiguous action (delete,
 * disconnect, or a cube deletion that would split the voxel graph and needs
 * an explicit orphan-vs-cascade decision).
 */
import { useEffect } from 'react';
import { useUIStore } from '../state/uiStore';

export function ConfirmDialogHost() {
  const choiceRequest = useUIStore((s) => s.choiceRequest);
  const resolveChoice = useUIStore((s) => s.resolveChoice);

  useEffect(() => {
    if (!choiceRequest) return;
    function handleKey(event: KeyboardEvent) {
      if (event.key === 'Escape') resolveChoice(null);
    }
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, [choiceRequest, resolveChoice]);

  if (!choiceRequest) return null;

  return (
    <div className="modal-overlay" onClick={() => resolveChoice(null)}>
      <div className="modal" onClick={(e) => e.stopPropagation()}>
        <p className="modal__message">{choiceRequest.message}</p>
        <div className="modal__actions">
          {choiceRequest.options.map((option) => (
            <button
              key={option.value}
              className={`btn${option.danger ? ' btn--danger' : ''}`}
              onClick={() => resolveChoice(option.value)}
            >
              {option.label}
            </button>
          ))}
        </div>
      </div>
    </div>
  );
}
