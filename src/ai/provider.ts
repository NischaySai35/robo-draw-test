/**
 * Provider interface for the text-to-structure step, plus the streaming
 * bookkeeping the test tab displays.
 *
 * Streaming is not decoration here. The whole point of this tab is to find out
 * whether a model can do this job at all, and the two questions that answers --
 * is it fast enough to sit in a control loop, and does it think structurally or
 * just pattern-match a chair -- are both only visible while tokens arrive.
 * Time-to-first-token and tokens/sec are measured separately because they fail
 * separately: a slow first token is the network, a slow rate is the model.
 */
export type ProviderId = 'anthropic' | 'ollama' | 'template';

export interface GenerationMetrics {
  /** ms from request start to the first content token. Network + queue + prefill. */
  firstTokenMs: number | null;
  /** ms from request start to the last token. */
  totalMs: number;
  /** Output tokens, as reported by the API. Estimated for providers that don't say. */
  outputTokens: number;
  inputTokens: number;
  /** Output tokens per second, measured from first token to last -- excludes prefill. */
  tokensPerSecond: number;
}

export interface GenerationResult {
  /** Everything the model emitted, joined. */
  text: string;
  metrics: GenerationMetrics;
  /** Set when the model stopped for a reason other than finishing. */
  stopReason: string | null;
}

export interface GenerateOptions {
  systemPrompt: string;
  userPrompt: string;
  model: string;
  maxTokens: number;
  temperature: number;
  /** Called with each text delta as it arrives. */
  onDelta: (delta: string) => void;
  signal: AbortSignal;
}

export interface ShapeProvider {
  id: ProviderId;
  label: string;
  /** False when the provider needs a key it hasn't been given. */
  isConfigured: () => boolean;
  generate: (options: GenerateOptions) => Promise<GenerationResult>;
}

/**
 * Models offered in the tab. Deliberately spans the range: whether the cheapest
 * model can do this is the interesting question, because a structure generator
 * that needs the largest model is one that cannot run per-command on hardware.
 */
export const AVAILABLE_MODELS = [
  { id: 'claude-haiku-4-5-20251001', label: 'Haiku 4.5 -- fastest, cheapest' },
  { id: 'claude-sonnet-5', label: 'Sonnet 5 -- balanced' },
  { id: 'claude-opus-5', label: 'Opus 5 -- most capable' },
] as const;

export const DEFAULT_MODEL = 'claude-sonnet-5';

/**
 * Small local models worth pulling for this job: the whole point of the local
 * path is finding out whether something this size can hold the wireframe
 * rule and the JSON shape at once. Picked for a 1B-3B spread with strong
 * instruction-following at that size, on Ollama's registry by default so a
 * plain `ollama pull <name>` works with no extra flags.
 */
export const OLLAMA_MODELS = [
  { id: 'llama3.2:1b', label: 'Llama 3.2 1B -- fastest, weakest reasoning' },
  { id: 'qwen2.5:1.5b', label: 'Qwen 2.5 1.5B -- fast, decent structure' },
  { id: 'llama3.2:3b', label: 'Llama 3.2 3B -- balanced' },
  { id: 'qwen2.5:3b', label: 'Qwen 2.5 3B -- strongest of this size' },
] as const;

export const DEFAULT_OLLAMA_MODEL = 'qwen2.5:3b';
