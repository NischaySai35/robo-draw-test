/**
 * State for the "Prompt to Build" test tab: one run of text -> structure,
 * broken into stages that report as they happen.
 *
 * The stage list is the point of this tab. "Build me a chair" can fail in at
 * least six different places -- the model refuses, the JSON is truncated, the
 * shape is a solid rather than a wireframe, it rescales to something absurd,
 * the loops will not close, the result collides with itself -- and until each
 * of those is visible separately, a bad chair is just a bad chair with no way
 * to tell whether the model or the solver is at fault. Each stage records its
 * own duration and its own outcome, so a failure names itself.
 *
 * Stages run with a yield between them so the UI paints each one as it starts.
 * `fitSkeleton` in particular blocks for seconds on a shape with loops, and a
 * frozen panel with no explanation looks exactly like a hang.
 */
import { create } from 'zustand';
import { anthropicProvider } from '../ai/anthropicProvider';
import { templateProvider } from '../ai/templateProvider';
import { extractJson, normalizeShape } from '../ai/normalizeShape';
import { SHAPE_SYSTEM_PROMPT, buildUserPrompt } from '../ai/prompt';
import { DEFAULT_MODEL, type GenerationMetrics, type ProviderId, type ShapeProvider } from '../ai/provider';
import { fitSkeleton, type ShapeSpec, type SkeletonFitResult } from '../kinematics/skeletonFit';
import { checkSelfCollision } from '../kinematics/collision';

export type StageId = 'model' | 'parse' | 'normalize' | 'fit' | 'verify';
export type StageStatus = 'pending' | 'running' | 'done' | 'failed' | 'warned';

export interface Stage {
  id: StageId;
  label: string;
  status: StageStatus;
  detail: string;
  ms: number | null;
}

export interface BuildStats {
  moduleCount: number;
  beamCount: number;
  jointCount: number;
  loopCount: number;
  loopError: number | null;
  loopsClosed: boolean | null;
  collides: boolean;
  collisionPairs: number;
  worstPenetration: number;
  worstResidual: number;
  unanchored: number;
}

const STAGE_TEMPLATE: Stage[] = [
  { id: 'model', label: 'Model writes a structure', status: 'pending', detail: '', ms: null },
  { id: 'parse', label: 'Parse JSON', status: 'pending', detail: '', ms: null },
  { id: 'normalize', label: 'Rescale + repair', status: 'pending', detail: '', ms: null },
  { id: 'fit', label: 'Fit modules, close loops', status: 'pending', detail: '', ms: null },
  { id: 'verify', label: 'Check for self-collision', status: 'pending', detail: '', ms: null },
];

const PROVIDERS: Record<ProviderId, ShapeProvider> = {
  anthropic: anthropicProvider,
  template: templateProvider,
};

interface GenerateState {
  request: string;
  providerId: ProviderId;
  model: string;
  maxTokens: number;
  temperature: number;
  /** True while a run is in flight. */
  running: boolean;
  /** Raw model output so far, appended live. */
  streamText: string;
  /** The model's one-line rationale, pulled out of the response once parsed. */
  reasoning: string;
  stages: Stage[];
  metrics: GenerationMetrics | null;
  repairs: string[];
  error: string | null;
  spec: ShapeSpec | null;
  fit: SkeletonFitResult | null;
  stats: BuildStats | null;
  /** Whether the viewport shows the module assembly or just the skeleton. */
  showModules: boolean;
  /** Bumps whenever the viewport needs to redraw. */
  revision: number;

  setRequest: (value: string) => void;
  setProvider: (id: ProviderId) => void;
  setModel: (model: string) => void;
  setMaxTokens: (value: number) => void;
  setTemperature: (value: number) => void;
  setShowModules: (value: boolean) => void;
  run: () => Promise<void>;
  cancel: () => void;
  reset: () => void;
}

/** Lets the browser paint between stages so progress is visible, not retroactive. */
const yieldToPaint = (): Promise<void> =>
  new Promise((resolve) => {
    requestAnimationFrame(() => setTimeout(resolve, 0));
  });

let abortController: AbortController | null = null;

export const useGenerateStore = create<GenerateState>((set, get) => {
  const patchStage = (id: StageId, patch: Partial<Stage>) => {
    set((state) => ({ stages: state.stages.map((s) => (s.id === id ? { ...s, ...patch } : s)) }));
  };

  const fail = (id: StageId, message: string) => {
    patchStage(id, { status: 'failed', detail: message });
    set({ error: message, running: false });
  };

  return {
    request: 'a chair',
    providerId: 'anthropic',
    model: DEFAULT_MODEL,
    maxTokens: 4000,
    temperature: 1,
    running: false,
    streamText: '',
    reasoning: '',
    stages: STAGE_TEMPLATE,
    metrics: null,
    repairs: [],
    error: null,
    spec: null,
    fit: null,
    stats: null,
    showModules: true,
    revision: 0,

    setRequest: (value) => set({ request: value }),
    setProvider: (id) => set({ providerId: id }),
    setModel: (model) => set({ model }),
    setMaxTokens: (value) => set({ maxTokens: Math.max(256, Math.min(16000, Math.round(value))) }),
    setTemperature: (value) => set({ temperature: Math.max(0, Math.min(1, value)) }),
    setShowModules: (value) => set((state) => ({ showModules: value, revision: state.revision + 1 })),

    cancel: () => {
      abortController?.abort();
      abortController = null;
      set({ running: false });
    },

    reset: () =>
      set((state) => ({
        streamText: '',
        reasoning: '',
        stages: STAGE_TEMPLATE,
        metrics: null,
        repairs: [],
        error: null,
        spec: null,
        fit: null,
        stats: null,
        revision: state.revision + 1,
      })),

    run: async () => {
      const { request, providerId, model, maxTokens, temperature, running } = get();
      if (running) return;
      if (!request.trim()) {
        set({ error: 'Describe something to build first.' });
        return;
      }

      get().reset();
      set({ running: true });
      abortController = new AbortController();
      const provider = PROVIDERS[providerId];

      // --- Stage 1: the model ------------------------------------------------
      patchStage('model', { status: 'running', detail: `${provider.label} - ${providerId === 'anthropic' ? model : 'keyword match'}` });
      await yieldToPaint();

      let text = '';
      let metrics: GenerationMetrics;
      try {
        const result = await provider.generate({
          systemPrompt: SHAPE_SYSTEM_PROMPT,
          userPrompt: buildUserPrompt(request),
          model,
          maxTokens,
          temperature,
          signal: abortController.signal,
          onDelta: (delta) => set((state) => ({ streamText: state.streamText + delta })),
        });
        text = result.text;
        metrics = result.metrics;
        set({ metrics });

        if (result.stopReason === 'max_tokens') {
          // Worth calling out separately: the JSON will almost certainly fail to
          // parse next, and the fix is the token limit, not the prompt.
          patchStage('model', {
            status: 'warned',
            detail: `Hit the ${maxTokens}-token limit -- the response is cut off. Raise max tokens.`,
            ms: metrics.totalMs,
          });
        } else {
          patchStage('model', {
            status: 'done',
            detail: `${metrics.outputTokens} tokens at ${metrics.tokensPerSecond.toFixed(1)}/s`,
            ms: metrics.totalMs,
          });
        }
      } catch (error) {
        if ((error as Error).name === 'AbortError') {
          patchStage('model', { status: 'failed', detail: 'Cancelled.' });
          set({ running: false });
          return;
        }
        fail('model', (error as Error).message);
        return;
      }

      // --- Stage 2: parse ----------------------------------------------------
      patchStage('parse', { status: 'running', detail: '' });
      await yieldToPaint();
      const parseStart = performance.now();
      const { value, error: parseError } = extractJson(text);
      if (parseError || value === null) {
        patchStage('parse', { status: 'failed', detail: parseError ?? 'No JSON found.', ms: performance.now() - parseStart });
        set({ error: parseError ?? 'No JSON found.', running: false });
        return;
      }
      const reasoning = typeof (value as Record<string, unknown>).reasoning === 'string'
        ? ((value as Record<string, unknown>).reasoning as string)
        : '';
      const rawName = typeof (value as Record<string, unknown>).name === 'string'
        ? ((value as Record<string, unknown>).name as string)
        : request.slice(0, 32);
      set({ reasoning });
      patchStage('parse', { status: 'done', detail: 'Valid JSON object.', ms: performance.now() - parseStart });

      // --- Stage 3: normalize ------------------------------------------------
      patchStage('normalize', { status: 'running', detail: '' });
      await yieldToPaint();
      const normalizeStart = performance.now();
      const normalized = normalizeShape(value, rawName);
      set({ repairs: normalized.repairs });
      if (!normalized.spec) {
        const message = normalized.errors.join(' ');
        patchStage('normalize', { status: 'failed', detail: message, ms: performance.now() - normalizeStart });
        set({ error: message, running: false });
        return;
      }
      const spec = normalized.spec;
      set((state) => ({ spec, revision: state.revision + 1 }));
      patchStage('normalize', {
        status: normalized.repairs.length > 1 ? 'warned' : 'done',
        detail: `${spec.nodes.length} joints, ${spec.edges.length} beams, ${normalized.repairs.length} repair(s).`,
        ms: performance.now() - normalizeStart,
      });

      // --- Stage 4: fit ------------------------------------------------------
      patchStage('fit', { status: 'running', detail: 'Placing modules and closing loops...' });
      await yieldToPaint();
      const fitStart = performance.now();
      let fit: SkeletonFitResult;
      try {
        fit = fitSkeleton(spec);
      } catch (error) {
        fail('fit', `The solver threw: ${(error as Error).message}`);
        return;
      }
      const fitMs = performance.now() - fitStart;
      const worstResidual = fit.chains.reduce((worst, chain) => Math.max(worst, chain.residual), 0);
      const loopsClosed = fit.loopReport ? fit.loopReport.converged : null;
      set((state) => ({ fit, revision: state.revision + 1 }));

      const loopText =
        fit.loopReport === null
          ? 'no loops'
          : loopsClosed
            ? `${fit.loopCount} loop(s) closed`
            : `${fit.loopCount} loop(s) did NOT close`;
      patchStage('fit', {
        status: loopsClosed === false || fit.unanchored.length > 0 ? 'warned' : 'done',
        detail: `${fit.moduleCount} modules, ${loopText}.`,
        ms: fitMs,
      });

      // --- Stage 5: verify ---------------------------------------------------
      patchStage('verify', { status: 'running', detail: '' });
      await yieldToPaint();
      const verifyStart = performance.now();
      const collision = checkSelfCollision(fit.assembly);
      const stats: BuildStats = {
        moduleCount: fit.moduleCount,
        beamCount: spec.edges.length,
        jointCount: spec.nodes.length,
        loopCount: fit.loopCount,
        loopError: fit.loopReport?.maxPositionError ?? null,
        loopsClosed,
        collides: collision.collides,
        collisionPairs: collision.pairs.length,
        worstPenetration: collision.worstPenetration,
        worstResidual,
        unanchored: fit.unanchored.length,
      };
      set({ stats, running: false });
      patchStage('verify', {
        status: collision.collides ? 'warned' : 'done',
        detail: collision.collides
          ? `${collision.pairs.length} overlapping pair(s), worst ${collision.worstPenetration.toFixed(3)}.`
          : 'No parts overlap.',
        ms: performance.now() - verifyStart,
      });

      abortController = null;
    },
  };
});
