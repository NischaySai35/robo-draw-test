/**
 * Streaming Anthropic Messages API client.
 *
 * Two routes to the API, and the choice is about where the key lives:
 *
 *  - PROXY (preferred): POST to `/api/anthropic/v1/messages`, which the Vite
 *    dev server forwards, attaching `ANTHROPIC_API_KEY` from `.env.local`. The
 *    key never enters the browser, so it cannot leak into devtools, a screen
 *    share, or a bundled build.
 *  - DIRECT: the browser calls api.anthropic.com itself with a key pasted into
 *    the UI. This needs `anthropic-dangerous-direct-browser-access`, and the
 *    header is named that way because it is: anyone who can see the page can
 *    read the key. Offered only because it needs no restart, and gated behind
 *    an explicit choice.
 *
 * The key is held in memory and (optionally) localStorage, never sent anywhere
 * but Anthropic.
 */
import type { GenerateOptions, GenerationResult, ShapeProvider } from './provider';

const PROXY_URL = '/api/anthropic/v1/messages';
const DIRECT_URL = 'https://api.anthropic.com/v1/messages';
const API_VERSION = '2023-06-01';

let apiKey = '';

export function setApiKey(key: string): void {
  apiKey = key.trim();
}

export function hasApiKey(): boolean {
  return apiKey.length > 0;
}

/**
 * Rough token estimate, used only to keep the live counter moving. The real
 * count replaces it the moment `message_delta` reports usage. ~4 chars/token is
 * close enough for a progress readout and wrong enough not to be quoted.
 */
const estimateTokens = (text: string): number => Math.max(1, Math.round(text.length / 4));

interface StreamState {
  text: string;
  firstTokenMs: number | null;
  outputTokens: number;
  inputTokens: number;
  stopReason: string | null;
}

/** Handles one SSE `data:` payload. Unknown event types are ignored by design. */
function handleEvent(raw: string, state: StreamState, startedAt: number, onDelta: (delta: string) => void): void {
  let event: Record<string, unknown>;
  try {
    event = JSON.parse(raw) as Record<string, unknown>;
  } catch {
    return; // A partial frame; the reader will deliver the rest.
  }

  switch (event.type) {
    case 'message_start': {
      const usage = (event.message as Record<string, unknown> | undefined)?.usage as
        | Record<string, number>
        | undefined;
      if (usage) state.inputTokens = usage.input_tokens ?? 0;
      break;
    }
    case 'content_block_delta': {
      const delta = event.delta as Record<string, unknown> | undefined;
      const text = typeof delta?.text === 'string' ? delta.text : '';
      if (!text) break;
      if (state.firstTokenMs === null) state.firstTokenMs = performance.now() - startedAt;
      state.text += text;
      state.outputTokens = estimateTokens(state.text);
      onDelta(text);
      break;
    }
    case 'message_delta': {
      const delta = event.delta as Record<string, unknown> | undefined;
      if (typeof delta?.stop_reason === 'string') state.stopReason = delta.stop_reason;
      const usage = event.usage as Record<string, number> | undefined;
      if (usage?.output_tokens) state.outputTokens = usage.output_tokens;
      break;
    }
    case 'error': {
      const error = event.error as Record<string, unknown> | undefined;
      throw new Error(typeof error?.message === 'string' ? error.message : 'The API reported an error mid-stream.');
    }
    default:
      break;
  }
}

async function readErrorBody(response: Response): Promise<string> {
  const body = await response.text().catch(() => '');
  try {
    const parsed = JSON.parse(body) as { error?: { message?: string } };
    if (parsed.error?.message) return parsed.error.message;
  } catch {
    // Not JSON -- fall through to the raw body.
  }
  return body.slice(0, 300) || response.statusText;
}

export const anthropicProvider: ShapeProvider = {
  id: 'anthropic',
  label: 'Anthropic API',
  // Always "configured": with no key in the UI it tries the proxy, which is the
  // path where the key lives server-side. A missing key surfaces as a 401 with
  // the API's own message, which is more useful than a guess made up front.
  isConfigured: () => true,

  async generate(options: GenerateOptions): Promise<GenerationResult> {
    const direct = hasApiKey();
    const headers: Record<string, string> = {
      'content-type': 'application/json',
      'anthropic-version': API_VERSION,
    };
    if (direct) {
      headers['x-api-key'] = apiKey;
      headers['anthropic-dangerous-direct-browser-access'] = 'true';
    }

    const startedAt = performance.now();
    let response: Response;
    try {
      response = await fetch(direct ? DIRECT_URL : PROXY_URL, {
        method: 'POST',
        headers,
        signal: options.signal,
        body: JSON.stringify({
          model: options.model,
          max_tokens: options.maxTokens,
          temperature: options.temperature,
          stream: true,
          system: options.systemPrompt,
          messages: [{ role: 'user', content: options.userPrompt }],
        }),
      });
    } catch (error) {
      if ((error as Error).name === 'AbortError') throw error;
      throw new Error(
        direct
          ? `Could not reach the Anthropic API: ${(error as Error).message}`
          : `Could not reach the dev proxy at ${PROXY_URL}. Set ANTHROPIC_API_KEY in .env.local and restart the dev server, or paste a key to call the API directly.`,
      );
    }

    if (!response.ok) {
      const detail = await readErrorBody(response);
      if (response.status === 401 || response.status === 403) {
        throw new Error(
          direct
            ? `API rejected the key (${response.status}): ${detail}`
            : `The proxy has no valid key (${response.status}): ${detail}. Put ANTHROPIC_API_KEY in .env.local and restart the dev server.`,
        );
      }
      if (response.status === 404 && !direct) {
        throw new Error(
          'The dev proxy is not running. Restart the Vite dev server after adding ANTHROPIC_API_KEY to .env.local, or paste a key to call the API directly.',
        );
      }
      throw new Error(`API returned ${response.status}: ${detail}`);
    }
    if (!response.body) throw new Error('The API returned no response body to stream.');

    const state: StreamState = { text: '', firstTokenMs: null, outputTokens: 0, inputTokens: 0, stopReason: null };
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    try {
      for (;;) {
        const { done, value } = await reader.read();
        if (done) break;
        buffer += decoder.decode(value, { stream: true });
        // SSE frames are separated by a blank line; a frame may span chunks.
        let split = buffer.indexOf('\n\n');
        while (split !== -1) {
          const frame = buffer.slice(0, split);
          buffer = buffer.slice(split + 2);
          for (const line of frame.split('\n')) {
            if (line.startsWith('data:')) handleEvent(line.slice(5).trim(), state, startedAt, options.onDelta);
          }
          split = buffer.indexOf('\n\n');
        }
      }
    } finally {
      reader.cancel().catch(() => undefined);
    }

    const totalMs = performance.now() - startedAt;
    // Rate is measured from the first token, not from the request: including
    // prefill would make a fast model on a slow link look like a slow model.
    const streamingMs = state.firstTokenMs === null ? totalMs : totalMs - state.firstTokenMs;
    return {
      text: state.text,
      stopReason: state.stopReason,
      metrics: {
        firstTokenMs: state.firstTokenMs,
        totalMs,
        outputTokens: state.outputTokens,
        inputTokens: state.inputTokens,
        tokensPerSecond: streamingMs > 0 ? (state.outputTokens / streamingMs) * 1000 : 0,
      },
    };
  },
};
