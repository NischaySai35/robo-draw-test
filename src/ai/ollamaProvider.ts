/**
 * Local inference via Ollama -- no API key, no network egress, runs on this
 * machine. This is the "real thinking, slowly" path: a 1-3B model on a CPU
 * with no GPU (Intel UHD 620 here) has no fast path, so the token-by-token
 * stream in the panel is not a UI flourish, it is what the model is actually
 * doing, at the rate it is actually doing it.
 *
 * Talks to Ollama's `/api/chat` streaming endpoint, which sends one JSON
 * object per line (NDJSON), not SSE -- different framing from the Anthropic
 * provider, so this parses newline-delimited frames instead of `data:` lines.
 *
 * Routed through the same-origin Vite proxy (see vite.config.ts) rather than
 * `http://localhost:11434` directly, purely to dodge the browser's CORS
 * preflight against a bare local port; there is no key to protect here.
 */
import type { GenerateOptions, GenerationResult, ShapeProvider } from './provider';

const PROXY_URL = '/api/ollama/api/chat';

/**
 * Context window sent to Ollama, capped hard. Qwen/Llama default to 32K+
 * tokens of context, and Ollama allocates the KV-cache for the FULL window
 * up front regardless of how much of it is used -- so an uncapped request
 * for a ~15-line JSON reply can still claim gigabytes of RAM. Our system
 * prompt + a generous reply fits in well under 2K tokens, so this is capped
 * there, not left at the model's default.
 */
const CONTEXT_WINDOW = 2048;

/**
 * Unload the model from RAM moments after each reply instead of Ollama's
 * default 5-minute keep-alive. This machine is a CPU-only laptop with one
 * model's worth of headroom, not several -- holding qwen resident while the
 * user then tries llama is exactly the "running all at once" to avoid. The
 * cost is a reload (several seconds) on the next request, which is a fair
 * trade against holding multiple GB pinned in the background.
 */
const KEEP_ALIVE = '5s';

/**
 * Leaves headroom for the OS and the browser instead of letting Ollama claim
 * every logical core, which is what starves the rest of the laptop while a
 * CPU-only model is thinking. `navigator.hardwareConcurrency` is only a
 * logical-core count, not a guarantee of physical performance cores, but
 * it's what the browser exposes -- good enough for "leave a couple free".
 */
function threadBudget(): number {
  const logical = typeof navigator !== 'undefined' ? navigator.hardwareConcurrency || 4 : 4;
  return Math.max(1, logical - 2);
}

interface ChatFrame {
  message?: { content?: string };
  done?: boolean;
  done_reason?: string;
  prompt_eval_count?: number;
  eval_count?: number;
  /** Nanoseconds, per Ollama's convention. */
  eval_duration?: number;
}

async function readErrorBody(response: Response): Promise<string> {
  const body = await response.text().catch(() => '');
  return body.slice(0, 300) || response.statusText;
}

export const ollamaProvider: ShapeProvider = {
  id: 'ollama',
  label: 'Local (Ollama)',
  // Always reports configured -- there's no key to check. A model that isn't
  // pulled or a server that isn't running both surface as a fetch failure with
  // a specific, actionable message instead.
  isConfigured: () => true,

  async generate(options: GenerateOptions): Promise<GenerationResult> {
    const startedAt = performance.now();
    let response: Response;
    try {
      response = await fetch(PROXY_URL, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        signal: options.signal,
        body: JSON.stringify({
          model: options.model,
          stream: true,
          // Ollama's structured-output mode: constrains sampling to valid
          // JSON. Doesn't guarantee it matches OUR shape, but it means
          // `normalizeShape` never has to fight stray prose or a markdown
          // fence -- exactly the failure a 1-3B model hits most often.
          format: 'json',
          keep_alive: KEEP_ALIVE,
          options: {
            temperature: options.temperature,
            num_predict: options.maxTokens,
            num_ctx: CONTEXT_WINDOW,
            num_thread: threadBudget(),
          },
          messages: [
            { role: 'system', content: options.systemPrompt },
            { role: 'user', content: options.userPrompt },
          ],
        }),
      });
    } catch (error) {
      if ((error as Error).name === 'AbortError') throw error;
      throw new Error(
        `Could not reach Ollama at ${PROXY_URL}. Is "ollama serve" running? ` +
          `(${(error as Error).message})`,
      );
    }

    if (!response.ok) {
      const detail = await readErrorBody(response);
      if (response.status === 404) {
        throw new Error(
          `Ollama has no model called "${options.model}". Pull it first: ollama pull ${options.model}`,
        );
      }
      throw new Error(`Ollama returned ${response.status}: ${detail}`);
    }
    if (!response.body) throw new Error('Ollama returned no response body to stream.');

    let text = '';
    let firstTokenMs: number | null = null;
    let outputTokens = 0;
    let inputTokens = 0;
    let evalNanos = 0;
    let stopReason: string | null = null;

    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';
    try {
      for (;;) {
        const { done, value } = await reader.read();
        if (done) break;
        buffer += decoder.decode(value, { stream: true });
        let newline = buffer.indexOf('\n');
        while (newline !== -1) {
          const line = buffer.slice(0, newline).trim();
          buffer = buffer.slice(newline + 1);
          newline = buffer.indexOf('\n');
          if (!line) continue;

          let frame: ChatFrame;
          try {
            frame = JSON.parse(line) as ChatFrame;
          } catch {
            continue; // A partial line the reader will complete.
          }

          const delta = frame.message?.content ?? '';
          if (delta) {
            if (firstTokenMs === null) firstTokenMs = performance.now() - startedAt;
            text += delta;
            options.onDelta(delta);
          }
          if (frame.done) {
            outputTokens = frame.eval_count ?? outputTokens;
            inputTokens = frame.prompt_eval_count ?? inputTokens;
            evalNanos = frame.eval_duration ?? evalNanos;
            stopReason = frame.done_reason === 'length' ? 'max_tokens' : (frame.done_reason ?? 'stop');
          }
        }
      }
    } finally {
      reader.cancel().catch(() => undefined);
    }

    const totalMs = performance.now() - startedAt;
    // Ollama reports its own generation-only duration (excludes prompt eval),
    // which is a truer tokens/sec than deriving it from wall-clock deltas.
    const evalMs = evalNanos / 1e6;
    const tokensPerSecond = evalMs > 0 ? (outputTokens / evalMs) * 1000 : 0;

    return {
      text,
      stopReason,
      metrics: {
        firstTokenMs,
        totalMs,
        outputTokens: outputTokens || Math.round(text.length / 4),
        inputTokens,
        tokensPerSecond,
      },
    };
  },
};

/** Models known to be pulled locally, discovered via `/api/tags`. */
export async function listOllamaModels(): Promise<string[]> {
  try {
    const response = await fetch('/api/ollama/api/tags');
    if (!response.ok) return [];
    const data = (await response.json()) as { models?: { name: string }[] };
    return (data.models ?? []).map((m) => m.name);
  } catch {
    return [];
  }
}
