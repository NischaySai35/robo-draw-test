/**
 * Control panel for the prompt-to-build test tab.
 *
 * Laid out in the order a run actually happens -- request, then controls, then
 * the pipeline, then what came out -- so the panel reads top to bottom as the
 * run progresses rather than making you hunt for the part that changed.
 *
 * The raw model output is shown in full, deliberately. This tab exists to
 * evaluate a model, and a summary of what it said is not evidence of what it
 * said; the failures worth seeing (a refusal, prose wrapped around the JSON, a
 * solid instead of a wireframe) are all invisible in a summary.
 */
import { useEffect, useRef, useState } from 'react';
import { useGenerateStore, type Stage } from '../state/generateStore';
import { useAssemblyStore } from '../state/assemblyStore';
import { useUIStore } from '../state/uiStore';
import { hasApiKey, setApiKey } from '../ai/anthropicProvider';
import { listOllamaModels } from '../ai/ollamaProvider';
import { AVAILABLE_MODELS, OLLAMA_MODELS, type ProviderId } from '../ai/provider';

const KEY_STORAGE = 'modulink.anthropicKey';

const EXAMPLES = ['a chair', 'a car', 'a house', 'a bicycle', 'a rocket', 'a dog', 'a bridge', 'a ladder'];

const STATUS_GLYPH: Record<Stage['status'], string> = {
  pending: '.',
  running: '>',
  done: 'ok',
  warned: '!',
  failed: 'x',
};

function StageRow({ stage }: { stage: Stage }) {
  return (
    <li className={`gen-stage gen-stage--${stage.status}`}>
      <span className="gen-stage__glyph">{STATUS_GLYPH[stage.status]}</span>
      <span className="gen-stage__body">
        <span className="gen-stage__label">
          {stage.label}
          {stage.ms !== null && <em className="gen-stage__ms">{stage.ms < 1000 ? `${Math.round(stage.ms)}ms` : `${(stage.ms / 1000).toFixed(2)}s`}</em>}
        </span>
        {stage.detail && <span className="gen-stage__detail">{stage.detail}</span>}
      </span>
    </li>
  );
}

export function GeneratePanel() {
  const store = useGenerateStore();
  const importAssembly = useAssemblyStore((s) => s.importAssembly);
  const setMode = useUIStore((s) => s.setMode);
  const pushWarning = useUIStore((s) => s.pushWarning);
  const [keyInput, setKeyInput] = useState('');
  const [showAdvanced, setShowAdvanced] = useState(false);
  const streamRef = useRef<HTMLPreElement | null>(null);
  // What Ollama actually has pulled, so a stale model picker doesn't send a
  // request that can only ever 404.
  const [pulledModels, setPulledModels] = useState<string[] | null>(null);

  useEffect(() => {
    if (store.providerId !== 'ollama') return;
    let cancelled = false;
    listOllamaModels().then((names) => {
      if (!cancelled) setPulledModels(names);
    });
    return () => {
      cancelled = true;
    };
  }, [store.providerId, store.running]);

  // Restore a previously pasted key. Kept in localStorage only when the user
  // pastes one -- the proxy path never puts a key in the browser at all.
  useEffect(() => {
    const stored = localStorage.getItem(KEY_STORAGE);
    if (stored) {
      setApiKey(stored);
      setKeyInput(stored);
    }
  }, []);

  // Follow the stream as it arrives; the interesting part is always the tail.
  useEffect(() => {
    const element = streamRef.current;
    if (element) element.scrollTop = element.scrollHeight;
  }, [store.streamText]);

  const applyKey = (value: string) => {
    setKeyInput(value);
    setApiKey(value);
    if (value.trim()) localStorage.setItem(KEY_STORAGE, value.trim());
    else localStorage.removeItem(KEY_STORAGE);
  };

  const sendToEditor = () => {
    if (!store.fit) return;
    importAssembly(store.fit.assembly);
    setMode('manual');
    pushWarning(`Sent ${store.fit.moduleCount} modules to the manual editor.`);
  };

  const { stats } = store;

  return (
    <div className="gen-panel">
      <div className="panel__header">Prompt to build</div>

      <div className="inspector__section">
        <div className="inspector__section-title">What should it build?</div>
        <textarea
          className="gen-panel__prompt"
          rows={3}
          value={store.request}
          placeholder="a chair with armrests"
          onChange={(e) => store.setRequest(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) void store.run();
          }}
        />
        <div className="gen-panel__examples">
          {EXAMPLES.map((example) => (
            <button key={example} className="gen-chip" disabled={store.running} onClick={() => store.setRequest(example)}>
              {example}
            </button>
          ))}
        </div>
        <div className="gen-panel__actions">
          {store.running ? (
            <button className="gen-panel__run" onClick={() => store.cancel()}>
              Cancel
            </button>
          ) : (
            <button className="gen-panel__run gen-panel__run--go" onClick={() => void store.run()}>
              Build it
            </button>
          )}
          <span className="gen-panel__hint">Ctrl+Enter</span>
        </div>
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Model</div>
        <label className="field">
          <span>Source</span>
          <select
            value={store.providerId}
            disabled={store.running}
            onChange={(e) => store.setProvider(e.target.value as ProviderId)}
          >
            <option value="anthropic">Anthropic API</option>
            <option value="ollama">Local (Ollama)</option>
            <option value="template">Local templates (no AI)</option>
          </select>
        </label>

        {store.providerId === 'ollama' && (
          <>
            <label className="field">
              <span>Model</span>
              <select value={store.model} disabled={store.running} onChange={(e) => store.setModel(e.target.value)}>
                {OLLAMA_MODELS.map((model) => (
                  <option key={model.id} value={model.id}>
                    {model.label}
                    {pulledModels !== null && !pulledModels.some((n) => n === model.id || n.startsWith(`${model.id}-`))
                      ? ' (not pulled)'
                      : ''}
                  </option>
                ))}
              </select>
            </label>
            {pulledModels !== null && pulledModels.length === 0 && (
              <p className="gen-panel__note">
                Ollama has no models pulled yet. In a terminal: <code>ollama pull {store.model}</code>. This runs
                entirely on your CPU (no GPU detected) -- a 1-3B model, so expect real but slow thinking, not
                instant answers.
              </p>
            )}
            {pulledModels === null && (
              <p className="gen-panel__note">
                Checking <code>localhost:11434</code>... if this doesn't clear, run <code>ollama serve</code>.
              </p>
            )}
            <p className="gen-panel__note">
              Kept light on purpose, for this machine: 2K-token context (not the model's 32K default), capped to{' '}
              {Math.max(1, (typeof navigator !== 'undefined' ? navigator.hardwareConcurrency || 4 : 4) - 2)} CPU
              threads, unloaded from RAM ~5s after each reply, and only ever one request in flight -- switching
              models mid-run isn't possible, and won't hold two resident at once.
            </p>
          </>
        )}

        {store.providerId === 'anthropic' && (
          <>
            <label className="field">
              <span>Model</span>
              <select value={store.model} disabled={store.running} onChange={(e) => store.setModel(e.target.value)}>
                {AVAILABLE_MODELS.map((model) => (
                  <option key={model.id} value={model.id}>
                    {model.label}
                  </option>
                ))}
              </select>
            </label>
            <label className="field">
              <span>API key</span>
              <input
                type="password"
                value={keyInput}
                placeholder={hasApiKey() ? '' : 'blank = use dev-server proxy'}
                onChange={(e) => applyKey(e.target.value)}
              />
            </label>
            <p className="gen-panel__note">
              Leave blank and the request goes through the Vite dev server, which adds{' '}
              <code>ANTHROPIC_API_KEY</code> from <code>.env.local</code> -- the key stays off the browser. Pasting one
              here calls the API directly from this page instead, where anyone looking at the screen can read it.
            </p>
          </>
        )}

        <button className="gen-panel__disclosure" onClick={() => setShowAdvanced((v) => !v)}>
          {showAdvanced ? 'Hide' : 'Show'} sampling controls
        </button>
        {showAdvanced && (
          <>
            <label className="field">
              <span>Max tokens</span>
              <input
                className="field__number"
                type="number"
                min={256}
                max={16000}
                step={256}
                value={store.maxTokens}
                onChange={(e) => store.setMaxTokens(Number(e.target.value))}
              />
            </label>
            <label className="field">
              <span>Temperature</span>
              <input
                type="range"
                min={0}
                max={1}
                step={0.05}
                value={store.temperature}
                onChange={(e) => store.setTemperature(Number(e.target.value))}
              />
              <span className="field__unit">{store.temperature.toFixed(2)}</span>
            </label>
          </>
        )}
      </div>

      <div className="inspector__section">
        <div className="inspector__section-title">Pipeline</div>
        <ul className="gen-stages">
          {store.stages.map((stage) => (
            <StageRow key={stage.id} stage={stage} />
          ))}
        </ul>
        {store.metrics && (
          <div className="gen-metrics">
            <div>
              <em>first token</em>
              {store.metrics.firstTokenMs === null ? '--' : `${Math.round(store.metrics.firstTokenMs)} ms`}
            </div>
            <div>
              <em>rate</em>
              {store.metrics.tokensPerSecond.toFixed(1)} tok/s
            </div>
            <div>
              <em>out</em>
              {store.metrics.outputTokens} tok
            </div>
            <div>
              <em>in</em>
              {store.metrics.inputTokens} tok
            </div>
          </div>
        )}
        {store.error && <div className="status-line status-line--error">{store.error}</div>}
      </div>

      {store.reasoning && (
        <div className="inspector__section">
          <div className="inspector__section-title">How it broke the object down</div>
          <p className="gen-panel__reasoning">{store.reasoning}</p>
        </div>
      )}

      <div className="inspector__section">
        <div className="inspector__section-title">Raw output</div>
        <pre className="gen-stream" ref={streamRef}>
          {store.streamText || 'Nothing yet.'}
          {store.running && <span className="gen-stream__caret">|</span>}
        </pre>
      </div>

      {store.repairs.length > 0 && (
        <div className="inspector__section">
          <div className="inspector__section-title">Repairs applied</div>
          <ul className="gen-repairs">
            {store.repairs.map((repair) => (
              <li key={repair}>{repair}</li>
            ))}
          </ul>
        </div>
      )}

      {stats && (
        <div className="inspector__section">
          <div className="inspector__section-title">Result</div>
          <div className="gen-metrics gen-metrics--wide">
            <div>
              <em>modules</em>
              {stats.moduleCount}
            </div>
            <div>
              <em>beams</em>
              {stats.beamCount}
            </div>
            <div>
              <em>joints</em>
              {stats.jointCount}
            </div>
            <div>
              <em>loops</em>
              {stats.loopCount}
            </div>
          </div>
          <div className={`status-line${stats.loopsClosed === false ? ' status-line--warn' : ''}`}>
            {stats.loopsClosed === null
              ? 'Tree structure -- no loops to close.'
              : stats.loopsClosed
                ? `Loops closed to ${stats.loopError?.toExponential(1)}.`
                : `Loops did NOT close (worst ${stats.loopError?.toFixed(3)}) -- this shape is not reachable as designed.`}
          </div>
          <div className={`status-line${stats.collides ? ' status-line--warn' : ''}`}>
            {stats.collides
              ? `${stats.collisionPairs} part(s) overlap, worst ${stats.worstPenetration.toFixed(3)}.`
              : 'Nothing overlaps -- physically buildable.'}
          </div>
          {stats.unanchored > 0 && (
            <div className="status-line status-line--warn">
              {stats.unanchored} branch(es) found no free connector and float unattached.
            </div>
          )}
          <label className="field field--toggle">
            <span>Show modules</span>
            <input type="checkbox" checked={store.showModules} onChange={(e) => store.setShowModules(e.target.checked)} />
          </label>
          <button className="gen-panel__run" onClick={sendToEditor} disabled={!store.fit}>
            Send to manual editor
          </button>
        </div>
      )}
    </div>
  );
}
